/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c4_rx;
DMA_HandleTypeDef hdma_i2c4_tx;

/* I2C4 init function */
void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x107075B0;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspInit 0 */

  /* USER CODE END I2C4_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C4 GPIO Configuration
    PB7     ------> I2C4_SDA
    PB8     ------> I2C4_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C4 clock enable */
    __HAL_RCC_I2C4_CLK_ENABLE();

    /* I2C4 DMA Init */
    /* I2C4_RX Init */
    hdma_i2c4_rx.Instance = BDMA_Channel0;
    hdma_i2c4_rx.Init.Request = BDMA_REQUEST_I2C4_RX;
    hdma_i2c4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c4_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c4_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_i2c4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c4_rx);

    /* I2C4_TX Init */
    hdma_i2c4_tx.Instance = BDMA_Channel1;
    hdma_i2c4_tx.Init.Request = BDMA_REQUEST_I2C4_TX;
    hdma_i2c4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c4_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c4_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_i2c4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c4_tx);

    /* I2C4 interrupt Init */
    HAL_NVIC_SetPriority(I2C4_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
    HAL_NVIC_SetPriority(I2C4_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);
  /* USER CODE BEGIN I2C4_MspInit 1 */

  /* USER CODE END I2C4_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspDeInit 0 */

  /* USER CODE END I2C4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C4_CLK_DISABLE();

    /**I2C4 GPIO Configuration
    PB7     ------> I2C4_SDA
    PB8     ------> I2C4_SCL
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    /* I2C4 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C4_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C4_ER_IRQn);
  /* USER CODE BEGIN I2C4_MspDeInit 1 */

  /* USER CODE END I2C4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
