#include "lib_serial_servo_h12h.h"

#include <stdbool.h>
#include <string.h>

#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
#if defined(__has_include)
#if __has_include("bsp_srn_log.h")
#include "bsp_srn_log.h"
#define SERIAL_SERVO_H12H_DIAG_LOGI(...) LOGI(__VA_ARGS__)
#define SERIAL_SERVO_H12H_DIAG_LOGW(...) LOGW(__VA_ARGS__)
#endif
#endif
#ifndef SERIAL_SERVO_H12H_DIAG_LOGI
#define SERIAL_SERVO_H12H_DIAG_LOGI(...) ((void)0)
#endif
#ifndef SERIAL_SERVO_H12H_DIAG_LOGW
#define SERIAL_SERVO_H12H_DIAG_LOGW(...) ((void)0)
#endif
#endif

#ifndef UART_FLAG_RXNE_RXFNE
#define UART_FLAG_RXNE_RXFNE UART_FLAG_RXNE
#endif

#ifndef SERIAL_SERVO_H12H_MAX_INSTANCES
#define SERIAL_SERVO_H12H_MAX_INSTANCES 4u
#endif

#define SERIAL_SERVO_H12H_RX_CANARY_HEAD 0xA5u
#define SERIAL_SERVO_H12H_RX_CANARY_TAIL 0x5Au

static serial_servo_h12h_t* g_serial_servo_h12h_instances[SERIAL_SERVO_H12H_MAX_INSTANCES] = {0};

static inline uint16_t serial_servo_h12h_arg_to_u16(const uint8_t* p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void serial_servo_h12h_u16_to_arg(uint8_t* p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static void serial_servo_h12h_rx_canary_reset(serial_servo_h12h_t* self)
{
  self->rx_dma_guard_head = SERIAL_SERVO_H12H_RX_CANARY_HEAD;
  self->rx_dma_guard_tail = SERIAL_SERVO_H12H_RX_CANARY_TAIL;
}

#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
static bool serial_servo_h12h_rx_canary_ok(const serial_servo_h12h_t* self)
{
  return (self->rx_dma_guard_head == SERIAL_SERVO_H12H_RX_CANARY_HEAD) &&
         (self->rx_dma_guard_tail == SERIAL_SERVO_H12H_RX_CANARY_TAIL);
}
#endif

static uint8_t serial_servo_h12h_checksum(const uint8_t* frame)
{
  uint16_t sum = 0;
  uint8_t i = 0;
  for (i = 2; i < (uint8_t)(frame[3] + 2u); ++i) {
    sum += frame[i];
  }
  return (uint8_t)(~sum);
}

static void serial_servo_h12h_flush_rx(UART_HandleTypeDef* huart)
{
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE_RXFNE) != RESET) {
#if defined(USART_RDR_RDR)
    (void)huart->Instance->RDR;
#else
    (void)huart->Instance->DR;
#endif
  }
}

static serial_servo_h12h_t* serial_servo_h12h_find_instance(UART_HandleTypeDef* huart)
{
  uint8_t i = 0;
  for (i = 0; i < SERIAL_SERVO_H12H_MAX_INSTANCES; ++i) {
    serial_servo_h12h_t* inst = g_serial_servo_h12h_instances[i];
    if (inst != NULL && inst->huart == huart) {
      return inst;
    }
  }
  return NULL;
}

static void serial_servo_h12h_register_instance(serial_servo_h12h_t* self)
{
  uint8_t i = 0;
  uint8_t free_idx = SERIAL_SERVO_H12H_MAX_INSTANCES;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (i = 0; i < SERIAL_SERVO_H12H_MAX_INSTANCES; ++i) {
    if (g_serial_servo_h12h_instances[i] == self) {
      __set_PRIMASK(primask);
      return;
    }
    if (g_serial_servo_h12h_instances[i] != NULL && g_serial_servo_h12h_instances[i]->huart == self->huart) {
      g_serial_servo_h12h_instances[i] = self;
      __set_PRIMASK(primask);
      return;
    }
    if (g_serial_servo_h12h_instances[i] == NULL && free_idx == SERIAL_SERVO_H12H_MAX_INSTANCES) {
      free_idx = i;
    }
  }
  if (free_idx < SERIAL_SERVO_H12H_MAX_INSTANCES) {
    g_serial_servo_h12h_instances[free_idx] = self;
  }
  __set_PRIMASK(primask);
}

static void serial_servo_h12h_delay_ms(uint32_t ms)
{
  if (ms == 0u) {
    return;
  }
  if (osKernelGetState() == osKernelRunning) {
    (void)osDelay(ms);
  } else {
    HAL_Delay(ms);
  }
}

static void serial_servo_h12h_sem_drain(osSemaphoreId_t sem)
{
  if (sem == NULL) {
    return;
  }
  while (osSemaphoreAcquire(sem, 0u) == osOK) {
  }
}

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
static bool serial_servo_h12h_dcache_enabled(void)
{
  return ((SCB->CCR & SCB_CCR_DC_Msk) != 0u);
}

static void serial_servo_h12h_cache_region(const uint8_t* buf, uint32_t len, uint32_t** aligned_addr, int32_t* aligned_len)
{
  uintptr_t start = (uintptr_t)buf;
  uintptr_t aligned_start = start & ~(uintptr_t)0x1Fu;
  uintptr_t aligned_end = (start + (uintptr_t)len + 31u) & ~(uintptr_t)0x1Fu;
  *aligned_addr = (uint32_t*)aligned_start;
  *aligned_len = (int32_t)(aligned_end - aligned_start);
}

static void serial_servo_h12h_dma_rx_buf_prepare(serial_servo_h12h_t* self)
{
  uint32_t* aligned_addr = NULL;
  int32_t aligned_len = 0;
  if (!serial_servo_h12h_dcache_enabled()) {
    return;
  }
  serial_servo_h12h_cache_region(self->rx_dma_buf, SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE, &aligned_addr, &aligned_len);
  SCB_CleanInvalidateDCache_by_Addr(aligned_addr, aligned_len);
}

static void serial_servo_h12h_dma_rx_buf_invalidate(serial_servo_h12h_t* self)
{
  uint32_t* aligned_addr = NULL;
  int32_t aligned_len = 0;
  if (!serial_servo_h12h_dcache_enabled()) {
    return;
  }
  serial_servo_h12h_cache_region(self->rx_dma_buf, SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE, &aligned_addr, &aligned_len);
  SCB_InvalidateDCache_by_Addr(aligned_addr, aligned_len);
}
#else
static void serial_servo_h12h_dma_rx_buf_prepare(serial_servo_h12h_t* self)
{
  (void)self;
}

static void serial_servo_h12h_dma_rx_buf_invalidate(serial_servo_h12h_t* self)
{
  (void)self;
}
#endif

static int serial_servo_h12h_lock(serial_servo_h12h_t* self)
{
  if (self->lock == NULL) {
    return SERIAL_SERVO_H12H_OK;
  }
  if (osKernelGetState() != osKernelRunning) {
    return SERIAL_SERVO_H12H_OK;
  }
  return (osOK == osMutexAcquire(self->lock, self->timeout_ms)) ? SERIAL_SERVO_H12H_OK : SERIAL_SERVO_H12H_ERR_LOCK;
}

static void serial_servo_h12h_unlock(serial_servo_h12h_t* self)
{
  if (self->lock == NULL) {
    return;
  }
  if (osKernelGetState() != osKernelRunning) {
    return;
  }
  (void)osMutexRelease(self->lock);
}

static void serial_servo_h12h_hal_rx_event_callback(UART_HandleTypeDef* huart, uint16_t size)
{
#if (SERIAL_SERVO_H12H_RX_USE_DMA == 1u)
  serial_servo_h12h_t* self = serial_servo_h12h_find_instance(huart);
  if (self == NULL) {
    return;
  }
  if (self->rx_dma_active == 0u) {
    return;
  }
  if (size > SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE) {
    size = SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE;
  }
  self->rx_size = size;
  self->rx_ready = 1u;
  self->rx_dma_active = 0u;
  if (self->rx_sem != NULL) {
    (void)osSemaphoreRelease(self->rx_sem);
  }
#else
  (void)huart;
  (void)size;
#endif
}

static void serial_servo_h12h_hal_error_callback(UART_HandleTypeDef* huart)
{
#if (SERIAL_SERVO_H12H_RX_USE_DMA == 1u)
  serial_servo_h12h_t* self = serial_servo_h12h_find_instance(huart);
  if (self == NULL) {
    return;
  }
  if (self->rx_dma_active == 0u) {
    return;
  }
  self->rx_error = 1u;
  self->rx_dma_active = 0u;
  if (self->rx_sem != NULL) {
    (void)osSemaphoreRelease(self->rx_sem);
  }
#else
  (void)huart;
#endif
}

static uint8_t serial_servo_h12h_count_headers(const uint8_t* buf, uint16_t size, int16_t* first_idx)
{
  uint16_t i = 0;
  uint8_t hits = 0u;
  int16_t first = -1;
  for (i = 0; (uint16_t)(i + 1u) < size; ++i) {
    if (buf[i] == SERIAL_SERVO_H12H_FRAME_HEADER && buf[i + 1u] == SERIAL_SERVO_H12H_FRAME_HEADER) {
      if (first < 0) {
        first = (int16_t)i;
      }
      if (hits < 0xFFu) {
        ++hits;
      }
    }
  }
  if (first_idx != NULL) {
    *first_idx = first;
  }
  return hits;
}

#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
static bool serial_servo_h12h_diag_need_dump(int ret)
{
  return (ret == SERIAL_SERVO_H12H_ERR_RX_TIMEOUT) ||
         (ret == SERIAL_SERVO_H12H_ERR_LEN) ||
         (ret == SERIAL_SERVO_H12H_ERR_CHECKSUM) ||
         (ret == SERIAL_SERVO_H12H_ERR_ID) ||
         (ret == SERIAL_SERVO_H12H_ERR_CMD);
}

static bool serial_servo_h12h_diag_should_dump(serial_servo_h12h_t* self)
{
  uint32_t now = HAL_GetTick();
  if ((now - self->diag_last_dump_tick) < SERIAL_SERVO_H12H_DIAG_STATS_WINDOW) {
    return false;
  }
  self->diag_last_dump_tick = now;
  return true;
}

static char serial_servo_h12h_diag_hex_nibble(uint8_t v)
{
  return (v < 10u) ? (char)('0' + v) : (char)('A' + (v - 10u));
}

static void serial_servo_h12h_diag_log_hex_dump(const uint8_t* buf, uint16_t size)
{
  uint16_t n = size;
  uint16_t i = 0;
  char line[SERIAL_SERVO_H12H_DIAG_HEX_DUMP_MAX * 3u + 1u] = {0};
  if (n > SERIAL_SERVO_H12H_DIAG_HEX_DUMP_MAX) {
    n = SERIAL_SERVO_H12H_DIAG_HEX_DUMP_MAX;
  }
  for (i = 0; i < n; ++i) {
    uint8_t b = buf[i];
    uint16_t idx = (uint16_t)(i * 3u);
    line[idx] = serial_servo_h12h_diag_hex_nibble((uint8_t)(b >> 4));
    line[idx + 1u] = serial_servo_h12h_diag_hex_nibble((uint8_t)(b & 0x0Fu));
    line[idx + 2u] = (i + 1u < n) ? ' ' : '\0';
  }
  SERIAL_SERVO_H12H_DIAG_LOGW("[SERVO_DIAG_LIB] raw[%u/%u]=%s",
                         (unsigned int)n,
                         (unsigned int)size,
                         line);
}

static void serial_servo_h12h_diag_mark_error(serial_servo_h12h_t* self, int ret)
{
  switch (ret) {
    case SERIAL_SERVO_H12H_ERR_RX_TIMEOUT:
      ++self->diag_err_timeout;
      break;
    case SERIAL_SERVO_H12H_ERR_CHECKSUM:
      ++self->diag_err_checksum;
      break;
    case SERIAL_SERVO_H12H_ERR_LEN:
      ++self->diag_err_len;
      break;
    case SERIAL_SERVO_H12H_ERR_ID:
      ++self->diag_err_id;
      break;
    case SERIAL_SERVO_H12H_ERR_CMD:
      ++self->diag_err_cmd;
      break;
    default:
      break;
  }
}

static bool serial_servo_h12h_diag_refresh_callbacks(serial_servo_h12h_t* self)
{
  if (HAL_UART_RegisterRxEventCallback(self->huart, serial_servo_h12h_hal_rx_event_callback) != HAL_OK ||
      HAL_UART_RegisterCallback(self->huart, HAL_UART_ERROR_CB_ID, serial_servo_h12h_hal_error_callback) != HAL_OK) {
    SERIAL_SERVO_H12H_DIAG_LOGW("[SERVO_DIAG_LIB] callback refresh failed uart=0x%08lX",
                           (unsigned long)self->huart->Instance);
    return false;
  }
  return true;
}

static void serial_servo_h12h_diag_capture(serial_servo_h12h_t* self,
                                      int ret,
                                      uint16_t rx_size,
                                      uint8_t hdr_hits,
                                      int16_t hdr_index,
                                      uint8_t tx_id,
                                      uint8_t cmd,
                                      uint8_t expected_id,
                                      const uint8_t* dump_buf,
                                      uint16_t dump_size)
{
  self->diag_last_err = (int16_t)ret;
  self->diag_last_rx_size = rx_size;
  self->diag_last_rx_ready = self->rx_ready;
  self->diag_last_rx_error = self->rx_error;
  self->diag_last_rx_dma_active = self->rx_dma_active;
  self->diag_last_hdr_hits = hdr_hits;
  self->diag_last_hdr_index = hdr_index;
  serial_servo_h12h_diag_mark_error(self, ret);

  if (serial_servo_h12h_diag_need_dump(ret) && serial_servo_h12h_diag_should_dump(self)) {
    SERIAL_SERVO_H12H_DIAG_LOGW(
        "[SERVO_DIAG_LIB] id=%u cmd=%u exp_id=%u ret=%d rx_size=%u ready=%u err=%u dma=%u hdr_hits=%u hdr_idx=%d",
        tx_id,
        cmd,
        expected_id,
        ret,
        rx_size,
        self->rx_ready,
        self->rx_error,
        self->rx_dma_active,
        hdr_hits,
        hdr_index);
    if (dump_buf != NULL && dump_size > 0u) {
      serial_servo_h12h_diag_log_hex_dump(dump_buf, dump_size);
    }
  }
}
#endif

static int serial_servo_h12h_parse_frame_from_buffer(const uint8_t* buf,
                                                uint16_t size,
                                                uint8_t expected_id,
                                                uint8_t expected_cmd,
                                                uint8_t* out_id,
                                                uint8_t* out_args,
                                                uint8_t* out_arg_len,
                                                uint8_t* out_hdr_hits,
                                                int16_t* out_hdr_first_idx)
{
  uint16_t i = 0;
  int last_err = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
  bool saw_frame = false;
  uint8_t hdr_hits = 0u;
  int16_t hdr_first = -1;

  if (out_hdr_hits != NULL || out_hdr_first_idx != NULL) {
    hdr_hits = serial_servo_h12h_count_headers(buf, size, &hdr_first);
  }

  for (i = 0; (uint16_t)(i + 5u) <= size; ++i) {
    uint8_t length = 0;
    uint16_t frame_len = 0;
    uint8_t got_id = 0;
    uint8_t got_cmd = 0;
    uint8_t args_len = 0;
    uint8_t got_checksum = 0;
    uint8_t expected_checksum = 0;
    bool id_ok = false;
    bool cmd_ok = false;
    if (buf[i] != SERIAL_SERVO_H12H_FRAME_HEADER || buf[i + 1u] != SERIAL_SERVO_H12H_FRAME_HEADER) {
      continue;
    }
    length = buf[i + 3u];
    if (length < 3u || length > 11u) {
      last_err = SERIAL_SERVO_H12H_ERR_LEN;
      saw_frame = true;
      continue;
    }
    frame_len = (uint16_t)(length + 3u);
    if ((uint16_t)(i + frame_len) > size) {
      saw_frame = true;
      continue;
    }
    got_id = buf[i + 2u];
    got_cmd = buf[i + 4u];
    args_len = (uint8_t)(length - 3u);
    got_checksum = buf[i + frame_len - 1u];
    expected_checksum = serial_servo_h12h_checksum(&buf[i]);
    id_ok = (expected_id == 0xFFu) || (expected_id == got_id);
    cmd_ok = (expected_cmd == got_cmd);
    if (!cmd_ok) {
      last_err = SERIAL_SERVO_H12H_ERR_CMD;
      saw_frame = true;
      continue;
    }
    if (!id_ok) {
      last_err = SERIAL_SERVO_H12H_ERR_ID;
      saw_frame = true;
      continue;
    }
    if (got_checksum != expected_checksum) {
      last_err = SERIAL_SERVO_H12H_ERR_CHECKSUM;
      saw_frame = true;
      continue;
    }
    if (out_id != NULL) {
      *out_id = got_id;
    }
    if (out_arg_len != NULL) {
      *out_arg_len = args_len;
    }
    if (out_args != NULL && args_len > 0u) {
      memcpy(out_args, &buf[i + 5u], args_len);
    }
    if (out_hdr_hits != NULL) {
      *out_hdr_hits = hdr_hits;
    }
    if (out_hdr_first_idx != NULL) {
      *out_hdr_first_idx = hdr_first;
    }
    return SERIAL_SERVO_H12H_OK;
  }
  if (out_hdr_hits != NULL) {
    *out_hdr_hits = hdr_hits;
  }
  if (out_hdr_first_idx != NULL) {
    *out_hdr_first_idx = hdr_first;
  }
  return saw_frame ? last_err : SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
}

static int serial_servo_h12h_recv_frame(serial_servo_h12h_t* self,
                                   uint8_t expected_id,
                                   uint8_t expected_cmd,
                                   uint8_t* out_id,
                                   uint8_t* out_args,
                                   uint8_t* out_arg_len,
                                   uint8_t* out_hdr_hits,
                                   int16_t* out_hdr_first_idx)
{
  uint8_t buf[16] = {0};
  uint8_t state = 0;
  uint8_t index = 0;
  uint8_t length = 0;
  int last_err = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
  bool saw_frame = false;
  uint32_t start = HAL_GetTick();
  uint8_t hdr_hits = 0u;
  int16_t hdr_first = -1;

  while ((HAL_GetTick() - start) < self->timeout_ms) {
    uint8_t b = 0;
    if (HAL_UART_Receive(self->huart, &b, 1, 1) != HAL_OK) {
      continue;
    }

    switch (state) {
      case 0:
        if (b == SERIAL_SERVO_H12H_FRAME_HEADER) {
          state = 1;
          buf[0] = b;
        }
        break;
      case 1:
        if (b == SERIAL_SERVO_H12H_FRAME_HEADER) {
          if (hdr_first < 0) {
            hdr_first = 0;
          }
          if (hdr_hits < 0xFFu) {
            ++hdr_hits;
          }
          state = 2;
          buf[1] = b;
        } else {
          state = 0;
        }
        break;
      case 2:
        buf[2] = b;
        state = 3;
        break;
      case 3:
        length = b;
        if (length < 3u || length > 11u) {
          last_err = SERIAL_SERVO_H12H_ERR_LEN;
          saw_frame = true;
          state = 0;
          break;
        }
        buf[3] = b;
        state = 4;
        break;
      case 4:
        buf[4] = b;
        index = 5;
        state = 5;
        break;
      case 5:
        buf[index++] = b;
        if (index >= (uint8_t)(length + 3u)) {
          uint8_t got_id = buf[2];
          uint8_t got_cmd = buf[4];
          uint8_t args_len = (uint8_t)(length - 3u);
          uint8_t got_checksum = buf[length + 2u];
          uint8_t expected_checksum = serial_servo_h12h_checksum(buf);
          bool id_ok = (expected_id == 0xFFu) || (expected_id == got_id);
          bool cmd_ok = (expected_cmd == got_cmd);
          if (!cmd_ok) {
            last_err = SERIAL_SERVO_H12H_ERR_CMD;
            saw_frame = true;
            state = 0;
            break;
          }
          if (!id_ok) {
            last_err = SERIAL_SERVO_H12H_ERR_ID;
            saw_frame = true;
            state = 0;
            break;
          }
          if (got_checksum != expected_checksum) {
            last_err = SERIAL_SERVO_H12H_ERR_CHECKSUM;
            saw_frame = true;
            state = 0;
            break;
          }
          {
            if (out_id != NULL) {
              *out_id = got_id;
            }
            if (out_arg_len != NULL) {
              *out_arg_len = args_len;
            }
            if (out_args != NULL && args_len > 0u) {
              memcpy(out_args, &buf[5], args_len);
            }
            if (out_hdr_hits != NULL) {
              *out_hdr_hits = hdr_hits;
            }
            if (out_hdr_first_idx != NULL) {
              *out_hdr_first_idx = hdr_first;
            }
            return 0;
          }
          state = 0;
        }
        break;
      default:
        state = 0;
        break;
    }
  }

  if (out_hdr_hits != NULL) {
    *out_hdr_hits = hdr_hits;
  }
  if (out_hdr_first_idx != NULL) {
    *out_hdr_first_idx = hdr_first;
  }
  return saw_frame ? last_err : SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
}

static bool serial_servo_h12h_prepare_async_rx(serial_servo_h12h_t* self)
{
  if (osKernelGetState() != osKernelRunning) {
    return false;
  }
  if (self->rx_sem == NULL) {
    self->rx_sem = osSemaphoreNew(1u, 0u, NULL);
    if (self->rx_sem == NULL) {
      return false;
    }
    if (HAL_UART_RegisterRxEventCallback(self->huart, serial_servo_h12h_hal_rx_event_callback) != HAL_OK ||
        HAL_UART_RegisterCallback(self->huart, HAL_UART_ERROR_CB_ID, serial_servo_h12h_hal_error_callback) != HAL_OK) {
      (void)osSemaphoreDelete(self->rx_sem);
      self->rx_sem = NULL;
      return false;
    }
  }
  return true;
}

static int serial_servo_h12h_transceive(serial_servo_h12h_t* self,
                                   uint8_t tx_id,
                                   uint8_t tx_cmd,
                                   const uint8_t* tx_args,
                                   uint8_t tx_arg_len,
                                   bool need_resp,
                                   uint8_t expected_rx_id,
                                   uint8_t* rx_id,
                                   uint8_t* rx_args,
                                   uint8_t* rx_arg_len)
{
  uint8_t tx[16] = {0};
  uint8_t tx_len = (uint8_t)(tx_arg_len + 6u);
  int ret = SERIAL_SERVO_H12H_OK;
  bool async_started = false;
  bool use_dma = false;
  bool rtos_running = (osKernelGetState() == osKernelRunning);
  uint16_t diag_rx_size = 0u;
  uint8_t diag_hdr_hits = 0u;
  int16_t diag_hdr_index = -1;
  const uint8_t* diag_dump_buf = NULL;
  uint16_t diag_dump_size = 0u;

  tx[0] = SERIAL_SERVO_H12H_FRAME_HEADER;
  tx[1] = SERIAL_SERVO_H12H_FRAME_HEADER;
  tx[2] = tx_id;
  tx[3] = (uint8_t)(tx_arg_len + 3u);
  tx[4] = tx_cmd;
  if (tx_args != NULL && tx_arg_len > 0u) {
    memcpy(&tx[5], tx_args, tx_arg_len);
  }
  tx[5u + tx_arg_len] = serial_servo_h12h_checksum(tx);

  if (serial_servo_h12h_lock(self) != SERIAL_SERVO_H12H_OK) {
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
    serial_servo_h12h_diag_capture(self,
                              SERIAL_SERVO_H12H_ERR_LOCK,
                              0u,
                              0u,
                              -1,
                              tx_id,
                              tx_cmd,
                              expected_rx_id,
                              NULL,
                              0u);
#endif
    return SERIAL_SERVO_H12H_ERR_LOCK;
  }

  serial_servo_h12h_flush_rx(self->huart);
  HAL_HalfDuplex_EnableTransmitter(self->huart);
  if (HAL_UART_Transmit(self->huart, tx, tx_len, self->timeout_ms) != HAL_OK) {
    ret = SERIAL_SERVO_H12H_ERR_TX;
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
    serial_servo_h12h_diag_capture(self,
                              ret,
                              0u,
                              0u,
                              -1,
                              tx_id,
                              tx_cmd,
                              expected_rx_id,
                              NULL,
                              0u);
#endif
    HAL_HalfDuplex_EnableReceiver(self->huart);
    serial_servo_h12h_delay_ms(SERIAL_SERVO_H12H_INTER_FRAME_GAP_MS);
    serial_servo_h12h_unlock(self);
    return ret;
  }

  HAL_HalfDuplex_EnableReceiver(self->huart);
  if (need_resp && serial_servo_h12h_prepare_async_rx(self)) {
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
    if (!serial_servo_h12h_diag_refresh_callbacks(self)) {
      ret = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
    } else
#endif
    {
#if (SERIAL_SERVO_H12H_RX_USE_DMA == 1u)
      self->rx_ready = 0u;
      self->rx_error = 0u;
      self->rx_size = 0u;
      self->rx_dma_active = 1u;
      serial_servo_h12h_rx_canary_reset(self);
      serial_servo_h12h_sem_drain(self->rx_sem);
      if (self->huart->hdmarx != NULL) {
        serial_servo_h12h_dma_rx_buf_prepare(self);
        if (HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->rx_dma_buf, SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE) == HAL_OK) {
          __HAL_DMA_DISABLE_IT(self->huart->hdmarx, DMA_IT_HT);
          async_started = true;
          use_dma = true;
        }
      }
#endif
      if (!async_started) {
        if (HAL_UARTEx_ReceiveToIdle_IT(self->huart, self->rx_dma_buf, SERIAL_SERVO_H12H_RX_DMA_BUF_SIZE) == HAL_OK) {
          async_started = true;
          use_dma = false;
        } else {
          ret = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
          self->rx_dma_active = 0u;
        }
      }
      if (async_started) {
        if (osSemaphoreAcquire(self->rx_sem, self->timeout_ms) == osOK &&
            self->rx_ready != 0u &&
            self->rx_error == 0u) {
          uint16_t rx_size = self->rx_size;
          diag_rx_size = rx_size;
#if (SERIAL_SERVO_H12H_RX_USE_DMA == 1u)
          if (use_dma) {
            serial_servo_h12h_dma_rx_buf_invalidate(self);
          }
#endif
          ret = serial_servo_h12h_parse_frame_from_buffer(self->rx_dma_buf,
                                                     rx_size,
                                                     expected_rx_id,
                                                     tx_cmd,
                                                     rx_id,
                                                     rx_args,
                                                     rx_arg_len,
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
                                                     &diag_hdr_hits,
                                                     &diag_hdr_index
#else
                                                     NULL,
                                                     NULL
#endif
                                                     );
          diag_dump_buf = self->rx_dma_buf;
          diag_dump_size = rx_size;
        } else {
          ret = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
          diag_rx_size = self->rx_size;
          diag_dump_buf = self->rx_dma_buf;
          diag_dump_size = self->rx_size;
        }
        self->rx_dma_active = 0u;
        (void)HAL_UART_AbortReceive(self->huart);
      }
    }
  }

  if (need_resp && !async_started) {
    if (rtos_running) {
      ret = SERIAL_SERVO_H12H_ERR_RX_TIMEOUT;
    } else {
      ret = serial_servo_h12h_recv_frame(self,
                                    expected_rx_id,
                                    tx_cmd,
                                    rx_id,
                                    rx_args,
                                    rx_arg_len,
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
                                    &diag_hdr_hits,
                                    &diag_hdr_index
#else
                                    NULL,
                                    NULL
#endif
                                    );
    }
  }

#if (SERIAL_SERVO_H12H_DIAG_ENABLE != 1u)
  (void)diag_rx_size;
  (void)diag_hdr_hits;
  (void)diag_hdr_index;
  (void)diag_dump_buf;
  (void)diag_dump_size;
#endif

#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
  if (!serial_servo_h12h_rx_canary_ok(self)) {
    SERIAL_SERVO_H12H_DIAG_LOGW("[SERVO_DIAG_LIB] dma canary broken id=%u cmd=%u head=0x%02X tail=0x%02X",
                           tx_id,
                           tx_cmd,
                           self->rx_dma_guard_head,
                           self->rx_dma_guard_tail);
  }
  serial_servo_h12h_diag_capture(self,
                            ret,
                            diag_rx_size,
                            diag_hdr_hits,
                            diag_hdr_index,
                            tx_id,
                            tx_cmd,
                            expected_rx_id,
                            diag_dump_buf,
                            diag_dump_size);
#endif

  serial_servo_h12h_delay_ms(SERIAL_SERVO_H12H_INTER_FRAME_GAP_MS);
  serial_servo_h12h_unlock(self);
  return ret;
}

int serial_servo_h12h_init(serial_servo_h12h_t* self, UART_HandleTypeDef* huart, uint32_t timeout_ms)
{
  if (self == NULL || huart == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  memset(self, 0, sizeof(*self));
  self->huart = huart;
  self->timeout_ms = (timeout_ms == 0u) ? 20u : timeout_ms;
  serial_servo_h12h_rx_canary_reset(self);
#if (SERIAL_SERVO_H12H_DIAG_ENABLE == 1u)
  self->diag_last_err = 0;
  self->diag_last_hdr_index = -1;
#endif
  self->lock = osMutexNew(NULL);
#if (SERIAL_SERVO_H12H_RX_USE_DMA == 1u)
  if (self->huart->hdmarx != NULL) {
    if (osKernelGetState() == osKernelRunning) {
      self->rx_sem = osSemaphoreNew(1u, 0u, NULL);
    }
    if (HAL_UART_RegisterRxEventCallback(self->huart, serial_servo_h12h_hal_rx_event_callback) != HAL_OK ||
        HAL_UART_RegisterCallback(self->huart, HAL_UART_ERROR_CB_ID, serial_servo_h12h_hal_error_callback) != HAL_OK) {
      if (self->rx_sem != NULL) {
        (void)osSemaphoreDelete(self->rx_sem);
        self->rx_sem = NULL;
      }
    }
  }
#endif
  serial_servo_h12h_register_instance(self);
  HAL_HalfDuplex_EnableReceiver(self->huart);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_id(serial_servo_h12h_t* self, uint8_t query_id, uint8_t* out_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_id = 0;
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  uint8_t expect_id = (query_id == SERIAL_SERVO_H12H_BROADCAST_ID) ? 0xFFu : query_id;
  int ret = serial_servo_h12h_transceive(self, query_id, SERIAL_SERVO_H12H_CMD_ID_READ, NULL, 0u, true, expect_id, &rx_id, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  if (out_id != NULL) {
    *out_id = rx_args[0];
  }
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_write_id(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t new_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = new_id;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ID_WRITE, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_set_position(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4];
  if (position > SERIAL_SERVO_H12H_POS_MAX) {
    position = SERIAL_SERVO_H12H_POS_MAX;
  }
  serial_servo_h12h_u16_to_arg(&args[0], position);
  serial_servo_h12h_u16_to_arg(&args[2], duration_ms);
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_TIME_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_move_time_read(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* position, uint16_t* duration_ms)
{
  if (self == NULL || position == NULL || duration_ms == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_TIME_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 4u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *position = serial_servo_h12h_arg_to_u16(&rx_args[0]);
  *duration_ms = serial_servo_h12h_arg_to_u16(&rx_args[2]);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_move_time_wait_write(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4];
  if (position > SERIAL_SERVO_H12H_POS_MAX) {
    position = SERIAL_SERVO_H12H_POS_MAX;
  }
  serial_servo_h12h_u16_to_arg(&args[0], position);
  serial_servo_h12h_u16_to_arg(&args[2], duration_ms);
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_TIME_WAIT_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_move_time_wait_read(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* position, uint16_t* duration_ms)
{
  if (self == NULL || position == NULL || duration_ms == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_TIME_WAIT_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 4u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *position = serial_servo_h12h_arg_to_u16(&rx_args[0]);
  *duration_ms = serial_servo_h12h_arg_to_u16(&rx_args[2]);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_move_start(serial_servo_h12h_t* self, uint8_t servo_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_START, NULL, 0u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_move_stop(serial_servo_h12h_t* self, uint8_t servo_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_MOVE_STOP, NULL, 0u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_position(serial_servo_h12h_t* self, uint8_t servo_id, int16_t* position)
{
  if (self == NULL || position == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_POS_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 2u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *position = (int16_t)serial_servo_h12h_arg_to_u16(rx_args);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_deviation(serial_servo_h12h_t* self, uint8_t servo_id, int8_t* deviation)
{
  if (self == NULL || deviation == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ANGLE_OFFSET_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *deviation = (int8_t)rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_angle_offset_adjust(serial_servo_h12h_t* self, uint8_t servo_id, int8_t offset)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = (uint8_t)offset;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ANGLE_OFFSET_ADJUST, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_angle_offset_write(serial_servo_h12h_t* self, uint8_t servo_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ANGLE_OFFSET_WRITE, NULL, 0u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_angle_limit_write(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t min, uint16_t max)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4];
  serial_servo_h12h_u16_to_arg(&args[0], min);
  serial_servo_h12h_u16_to_arg(&args[2], max);
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ANGLE_LIMIT_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_angle_limit(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* min, uint16_t* max)
{
  if (self == NULL || min == NULL || max == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_ANGLE_LIMIT_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 4u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *min = serial_servo_h12h_arg_to_u16(&rx_args[0]);
  *max = serial_servo_h12h_arg_to_u16(&rx_args[2]);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_vin_limit_write(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t min_mv, uint16_t max_mv)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4];
  serial_servo_h12h_u16_to_arg(&args[0], min_mv);
  serial_servo_h12h_u16_to_arg(&args[2], max_mv);
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_VIN_LIMIT_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_vin_limit(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* min_mv, uint16_t* max_mv)
{
  if (self == NULL || min_mv == NULL || max_mv == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_VIN_LIMIT_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 4u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *min_mv = serial_servo_h12h_arg_to_u16(&rx_args[0]);
  *max_mv = serial_servo_h12h_arg_to_u16(&rx_args[2]);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_vin(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* vin_mv)
{
  if (self == NULL || vin_mv == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_VIN_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 2u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *vin_mv = serial_servo_h12h_arg_to_u16(rx_args);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_temp_limit_write(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t temp_limit)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = temp_limit;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_TEMP_MAX_LIMIT_WRITE, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_temp_limit(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t* temp_limit)
{
  if (self == NULL || temp_limit == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_TEMP_MAX_LIMIT_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *temp_limit = rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_temp(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t* temp_c)
{
  if (self == NULL || temp_c == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_TEMP_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *temp_c = rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_load_write(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t load_unload)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = load_unload;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LOAD_OR_UNLOAD_WRITE, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_load(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t* load_unload)
{
  if (self == NULL || load_unload == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LOAD_OR_UNLOAD_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *load_unload = rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_led_ctrl_write(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t led_ctrl)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = led_ctrl;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LED_CTRL_WRITE, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_led_ctrl_read(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t* led_ctrl)
{
  if (self == NULL || led_ctrl == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LED_CTRL_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *led_ctrl = rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_led_error_write(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t led_error)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[1];
  args[0] = led_error;
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LED_ERROR_WRITE, args, 1u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_led_error_read(serial_servo_h12h_t* self, uint8_t servo_id, uint8_t* led_error)
{
  if (self == NULL || led_error == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_LED_ERROR_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 1u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *led_error = rx_args[0];
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_distance(serial_servo_h12h_t* self, uint8_t servo_id, uint16_t* distance)
{
  if (self == NULL || distance == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_DIS_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 2u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *distance = serial_servo_h12h_arg_to_u16(rx_args);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_set_mode_position(serial_servo_h12h_t* self, uint8_t servo_id)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4] = {0, 0, 0, 0};
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_OR_MOTOR_MODE_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_set_mode_motor(serial_servo_h12h_t* self, uint8_t servo_id, int16_t speed)
{
  if (self == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t args[4];
  args[0] = 1u;
  args[1] = 0u;
  serial_servo_h12h_u16_to_arg(&args[2], (uint16_t)speed);
  return serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_OR_MOTOR_MODE_WRITE, args, 4u, false, 0u, NULL, NULL, NULL);
}

int serial_servo_h12h_read_mode(serial_servo_h12h_t* self, uint8_t servo_id, serial_servo_h12h_mode_t* mode, int16_t* speed)
{
  if (self == NULL || mode == NULL || speed == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  uint8_t rx_args[8] = {0};
  uint8_t rx_len = 0;
  int ret = serial_servo_h12h_transceive(self, servo_id, SERIAL_SERVO_H12H_CMD_OR_MOTOR_MODE_READ, NULL, 0u, true, servo_id, NULL, rx_args, &rx_len);
  if (ret != SERIAL_SERVO_H12H_OK) {
    return ret;
  }
  if (rx_len < 4u) {
    return SERIAL_SERVO_H12H_ERR_LEN;
  }
  *mode = (rx_args[0] == 0u) ? SERIAL_SERVO_H12H_MODE_POSITION : SERIAL_SERVO_H12H_MODE_MOTOR;
  *speed = (int16_t)serial_servo_h12h_arg_to_u16(&rx_args[2]);
  return SERIAL_SERVO_H12H_OK;
}

int serial_servo_h12h_read_all_info(serial_servo_h12h_t* self, uint8_t servo_id, serial_servo_h12h_info_t* info)
{
  if (self == NULL || info == NULL) {
    return SERIAL_SERVO_H12H_ERR_ARG;
  }
  memset(info, 0, sizeof(*info));

  int ret = SERIAL_SERVO_H12H_OK;
  ret = serial_servo_h12h_read_id(self, servo_id, &info->id);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_move_time_read(self, servo_id, &info->move_time_pos, &info->move_time_ms);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_move_time_wait_read(self, servo_id, &info->wait_time_pos, &info->wait_time_ms);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_position(self, servo_id, &info->position);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_deviation(self, servo_id, &info->deviation);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_angle_limit(self, servo_id, &info->angle_limit_min, &info->angle_limit_max);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_vin_limit(self, servo_id, &info->vin_limit_min, &info->vin_limit_max);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_vin(self, servo_id, &info->vin_mv);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_temp_limit(self, servo_id, &info->temp_limit);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_temp(self, servo_id, &info->temp_c);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_load(self, servo_id, &info->load_unload);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_mode(self, servo_id, &info->mode, &info->motor_speed);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_led_ctrl_read(self, servo_id, &info->led_ctrl);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_led_error_read(self, servo_id, &info->led_error);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;
  ret = serial_servo_h12h_read_distance(self, servo_id, &info->distance);
  if (ret != SERIAL_SERVO_H12H_OK) return ret;

  return SERIAL_SERVO_H12H_OK;
}

float serial_servo_h12h_position_to_deg(int16_t position)
{
  return ((float)position) * 240.0f / 1000.0f;
}



