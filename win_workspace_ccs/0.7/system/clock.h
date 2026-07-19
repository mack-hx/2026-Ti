/* ============================================================================
 * @file    clock.h
 * @brief   SysTick 1ms 时基 (tick_ms 跨模块时间基准)
 *
 *   提供:
 *     - tick_ms                全局 volatile, 每 1ms 自增
 *     - SysTick_Init()         初始化 SysTick (CPUCLK/1000 中断)
 *     - mspm0_delay_ms(ms)     阻塞延时
 *     - mspm0_get_clock_ms()   取当前 tick_ms 快照
 *
 *   KEY_Tick() 在本文件同目录的 clock.c 的 SysTick_Handler 里自动调,
 *   应用层不需要再主动调用. LED_Heartbeat() / 按键事件等待都基于 tick_ms.
 * ============================================================================
 */
#ifndef _CLOCK_H_
#define _CLOCK_H_

extern volatile unsigned long tick_ms;  /* SysTick 1ms 计数器, 上电后持续自增
                                             *
                                             * 注意: tick_ms 不是从 0 开始, 而是芯片上电/复位到
                                             * main() 入口之间已经累加了一些值 (硬件启动延迟).
                                             * 所以计算"从现在起等 N 毫秒"必须用:
                                             *   deadline = tick_ms + N
                                             * 而不是直接 while(tick_ms < N).
                                             * (详见 main.c wait_until 注释) */

/* 阻塞延时 num_ms 毫秒 */
int mspm0_delay_ms(unsigned long num_ms);

/* 取当前 tick_ms 快照到 count[0]; 返回 0=成功, 1=count==NULL */
int mspm0_get_clock_ms(unsigned long *count);

/* SysTick 初始化 (1ms 中断, 优先级 0).
 *
 *   一行 DL_SYSTICK_config + NVIC_SetPriority 即可 — SysConfig 自动生成的
 *   SYSCFG_DL_SYSTICK_init 已经把 ISER bit 15 设好 (跟 0.3 一致, 0.3 是已知稳定基线).
 *
 *   历史 (2026-07-16): 0.5 调试期曾用两阶段 SysTick_PreConfig + SysTick_EnableAndIRQ
 *   加主循环 ISER 直写, 想"避开" SysTick bit 15 被 NVIC_EnableIRQ 整字写覆盖,
 *   实际却把 race 放大到每帧一遇, 导致冷启动后随机 IRQ 死 — 0.5 调试后期已删.
 */
void SysTick_Init(void);

#endif  /* #ifndef _CLOCK_H_ */
