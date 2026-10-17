/**
 * brick.c —— 最小「启动即复位」镜像，**只为测 S8 的回滚**。
 *
 * 为什么需要它：回滚的判据是"新固件起不来 ⇒ 连续几次复位后自动切回旧槽"。
 * 所以要有一个**每次启动都立刻请求复位**的镜像：
 *   复位 → Bootloader 进 trial 槽 → boot_attempts+1 → 跳过来 → 它又复位 …
 * 计数器爬满 max_boot_attempts 后，Bootloader 必须回滚到旧槽。
 *
 * ⚠️ 它必须**通过** Bootloader 的镜像校验（栈顶在 RAM、复位向量奇数且在槽内），
 *    否则测的就不是"回滚"而是"拒收"了。
 */
#include <stdint.h>

void Reset_Handler(void);

/* 向量表：只放前两项就够 —— 初始 MSP + 复位向量（其余中断不会用到）。
 * 🔴 不要写成 `((uint32_t)&Reset_Handler) | 1u`：静态初始化要求常量表达式，
 *    那样会报 `initializer element is not constant`。
 *    而 Thumb 函数的符号值**本来就带最低位 1**（链接器给的），直接用即可，
 *    编完再用 python 复核一次"是不是奇数"就行。 */
__attribute__((section(".isr_vector"), used))
const uint32_t g_vectors[2] = {
    0x20020000u,                          /* F407VG 128 KB SRAM 的顶 */
    (uint32_t)(uintptr_t)&Reset_Handler,  /* 复位向量：Thumb ⇒ 必须是奇数 */
};

/* 让镜像足够大：设备侧要求 len >= 256 B，太小的 .bin 会被直接拒收。 */
__attribute__((used))
const uint8_t g_padding[512] = { 0x00 };

void Reset_Handler(void)
{
    /* AIRCR: VECTKEY = 0x05FA，SYSRESETREQ = 1 ⇒ 立刻复位整片。 */
    *(volatile uint32_t *)0xE000ED0Cu = 0x05FA0004u;

    for (;;)
    {
        /* 正常情况到不了这里；万一复位请求被挂起，也别跑飞。 */
    }
}
