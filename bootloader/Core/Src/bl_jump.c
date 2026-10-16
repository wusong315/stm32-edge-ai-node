/**
  ******************************************************************************
  * @file    bl_jump.c
  * @brief   镜像校验 + 跳转实现。
  ******************************************************************************
  */
#include "main.h"
#include "bl_config.h"
#include "bl_jump.h"

#include <stdio.h>

/* 与 bl_boot.c 同一个理由：`huart1` 定义在 main.c，没有集中 extern。 */
extern UART_HandleTypeDef huart1;

/* RAM 边界（F407VG = 128 KB @ 0x20000000）。
 * 🔴 写死而不是读链接脚本符号：这里判的是"**别人**（App）的栈顶"，
 *    而 App 的栈顶由**它自己**的链接脚本决定，与 Bootloader 的 _estack 无关。
 *    两者都是同一个芯片的 SRAM1，所以用同一个常数。 */
#define BL_RAM_BASE     (0x20000000UL)
#define BL_RAM_END      (0x20020000UL)

/* =====================================================================
 * 一、镜像校验
 * =====================================================================
 * 三条判据，能不能跳。注意：**这些检查全部是只读的**，不改 Flash、不改寄存器
 * —— 所以 S3 阶段在空白板上反复跑也不会留下任何副作用。 */
BL_ImgStatus BL_ImageCheck(uint32_t addr, uint32_t size, uint32_t *sp, uint32_t *rv)
{
  uint32_t s = *(volatile uint32_t *)addr;
  uint32_t r = *(volatile uint32_t *)(addr + 4UL);

  if (sp != NULL) { *sp = s; }
  if (rv != NULL) { *rv = r; }

  /* ① 空白判据：擦除态 = 0xFFFFFFFF。这是"这一槽还没编程"最省事的证据。 */
  if (s == 0xFFFFFFFFUL)
  {
    return BL_IMG_BLANK;
  }

  /* ② 栈顶必须落在 RAM 里，且 8 字节对齐（Cortex-M 的栈按 8 字节对齐要求，
   *    越界/不对齐的栈顶跳进去就是立刻 HardFault）。 */
  if ((s < BL_RAM_BASE) || (s > BL_RAM_END) || ((s % 8UL) != 0UL))
  {
    return BL_IMG_BAD_SP;
  }

  /* ③ 复位向量必须带 Thumb 位（奇数），且落在本槽区间内。
   *    🔴 硬件不会替你检查这个：跳一个偶数地址进去，内核在 BX 时就会
   *       直接锁死（UsageFault 都来不及进），现象是"板子彻底没反应"。 */
  if (((r & 1UL) == 0UL) || (r < addr) || (r >= (addr + size)))
  {
    return BL_IMG_BAD_RV;
  }

  return BL_IMG_OK;
}

const char *BL_ImgStatusStr(BL_ImgStatus st)
{
  switch (st)
  {
    case BL_IMG_OK:      return "valid";
    case BL_IMG_BLANK:   return "blank (never programmed)";
    case BL_IMG_BAD_SP:  return "invalid: initial stack pointer not in RAM";
    case BL_IMG_BAD_RV:  return "invalid: reset vector not Thumb/out of slot";
    default:             return "invalid: unknown";
  }
}

/* =====================================================================
 * 二、跳转
 * ===================================================================== */

/* 最后一步：换栈 + 跳。用内联汇编**保证**这两个动作之间不碰栈。
 *
 * 🔴 为什么不用 `__set_MSP(sp); entry();`：
 *    那是 ST 官方例程的写法，绝大多数情况也能跑 —— 但 `entry()` 是普通函数调用，
 *    编译后会先往**新栈**压一个返回地址（在 SP-4 处写 4 字节）。虽然 App 的
 *    Reset_Handler 并不依赖 `_estack` 处的原始内容，但"跳转过程本身动了 App 的栈"
 *    是个应该消掉的不确定性。两条指令解决的事，不值得留悬念。 */
static void BL_DoJump(uint32_t sp, uint32_t entry) __attribute__((noreturn));
static void BL_DoJump(uint32_t sp, uint32_t entry)
{
  __asm volatile(
      "msr msp, %0  \n\t"
      "bx  %1      \n\t"
      :
      : "r"(sp), "r"(entry)
      : "memory");

  for (;;) { }   /* 不会到达；留着让编译器满意 */
}

/* 关掉本工程开过的一切。清单来自推演稿 §11.4（那张表是给 App 用的，
 * Bootloader 自己只开了 USART1 + 一个 GPIO，所以实际比它短）。 */
static void BL_DeInitPeripherals(void)
{
  (void)HAL_UART_DeInit(&huart1);                    /* 关 USART1（含 PA9/PA10 复用） */
  (void)HAL_GPIO_DeInit(LED_GPIO_Port, LED_Pin);     /* 松开 PA1 */

  (void)HAL_DeInit();        /* HAL 层复位：走 MspDeInit 关掉 USART1 时钟、清 SysTick */
  (void)HAL_RCC_DeInit();    /* RCC 回到复位态（SYSCLK = HSI 16 MHz）
                              * ⇒ 从这一行之后**不能再 printf**（波特率变了会吐乱码），
                              *   所以"正在跳转"那句话必须在调用本函数**之前**打印。 */
}

void BL_JumpToApp(uint32_t addr, uint32_t sp, uint32_t rv)
{
  uint32_t i;

  /* ---- 1) 先断外设与中断源，再动向量表 -------------------------------
   * 顺序很重要：VTOR 一旦指向 App 的向量表，任何**还能触发**的中断都会
   * 拿着 App 的入口地址、在 Bootloader 的上下文里执行。所以要先把中断源
   * 全部掐死（NVIC 使能位 + SysTick），再改 VTOR，最后才放开 PRIMASK。 */
  BL_DeInitPeripherals();

  __disable_irq();

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL  = 0U;

  for (i = 0U; i < 8U; i++)          /* F4 有 8 组 NVIC 寄存器（82 个中断） */
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;    /* 全部关掉 */
    NVIC->ICPR[i] = 0xFFFFFFFFUL;    /* 清掉所有挂起 */
  }
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

  /* ---- 2) 分区表里的第 3 处地址一致性：App 的向量表基址 ---------------- */
  SCB->VTOR = addr;

  __DSB();
  __ISB();

  /* ---- 3) 🔴 放开 PRIMASK，然后才跳 ---------------------------------
   * 参考仓库（以及 ST 的多数例程）在这里是 `__disable_irq()` 收尾就跳了。
   * 对它那个 App 没问题 —— 那个 App 根本没用中断。
   *
   * 我们的 App 不行：它靠 **SysTick**（HAL_Delay / HAL_GetTick）和
   * **TIM2 的 1 kHz 中断**活着。而 CPU 复位后的 PRIMASK 是 0，
   * App 的 `main()` 里**没有** `__enable_irq()`（已 grep 核对：
   * `Core/Src/main.c` 只在 Error_Handler 里出现 `__disable_irq()`）。
   *
   * ⇒ 如果带着 PRIMASK=1 跳进去，App 会：时钟配好、外设配好、然后
   *    卡在第一个 `HAL_Delay()` 里**永远不动**，串口一行不出、LED 不闪。
   *    现场表现是"App 根本没起来"，而它会把你引向"是不是固件坏了"，
   *    真正的原因却在 Bootloader 的最后两条指令里。
   *
   * ⚠️ 此时已经没有任何中断源被使能（上一步全清了），所以放开 PRIMASK
   *    本身不会让中断立刻打进来。 */
  __set_PRIMASK(0U);

  __DSB();
  __ISB();

  /* ---- 4) 换栈 + 跳（不返回） --------------------------------------- */
  BL_DoJump(sp, rv);

  for (;;) { }   /* 不可达 */
}
