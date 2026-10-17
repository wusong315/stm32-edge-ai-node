/* brick.ld —— 把「启动即复位」镜像放在 Slot B（0x08060000）。 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08060000, LENGTH = 384K
}

SECTIONS
{
  .text :
  {
    KEEP(*(.isr_vector))     /* 必须排第一：Bootloader 只读头 8 字节 */
    *(.text*)
    *(.rodata*)
  } > FLASH
}
