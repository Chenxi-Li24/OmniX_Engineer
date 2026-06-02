/* Lnk/dma_nc.ld — fragment only, no MEMORY/ENTRY here */

/* Place non-cacheable DMA descriptors/buffers.
   NOLOAD: do not bloat image or zero-init in .data/.bss.
   Align to cache line (32B on H7) to avoid DCache aliasing issues. */

SECTIONS
{
  /* Generic non-cacheable section for descriptors (small structs) */
  .dma_nc (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_nc_start__ = .;

    KEEP(*(.dma_nc .dma_nc.*))    /* user-marked symbols */
    *(.dma_nc_static .dma_nc_static.*)

    . = ALIGN(32);
    __dma_nc_end__ = .;
  } > RAM_D2

  /* Optional: bigger ring buffers go here */
  .dma_nc_buf (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_nc_buf_start__ = .;

    *(.dma_nc_buf .dma_nc_buf.*)

    . = ALIGN(32);
    __dma_nc_buf_end__ = .;
  } > RAM_D2
}

/* Provide handy sizes (not mandatory) */
__dma_nc_size__     = __dma_nc_end__     - __dma_nc_start__;
__dma_nc_buf_size__ = __dma_nc_buf_end__ - __dma_nc_buf_start__;