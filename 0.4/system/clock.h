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

extern volatile unsigned long tick_ms;

/* 阻塞延时 num_ms 毫秒 */
int mspm0_delay_ms(unsigned long num_ms);

/* 取当前 tick_ms 快照到 count[0]; 返回 0=成功, 1=count==NULL */
int mspm0_get_clock_ms(unsigned long *count);

/* SysTick 初始化: CPUCLK_FREQ/1000 = 1ms 中断. main() 上电调一次. */
void SysTick_Init(void);

#endif  /* #ifndef _CLOCK_H_ */
