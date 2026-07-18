/* ============================================================================
 * @file    clock.c
 * @brief   SysTick 1ms 时基 + KEY_Tick 推进 (按键扫描状态机)
 *
 * ============================================================================
 * 调用方法
 * ============================================================================
 *
 *   上电 (main() 里调一次):
 *     SysTick_Init();
 *
 *   之后自动:
 *     SysTick_Handler 每 1ms 自增 tick_ms + KEY_Tick() 推进按键状态机
 *
 *   延时 (主循环或初始化阻塞时):
 *     mspm0_delay_ms(10);                // 阻塞延时 10ms
 *
 *   取当前时间:
 *     uint32_t now;
 *     mspm0_get_clock_ms(&now);          // now = tick_ms 快照
 *
 *   应用层不应直接读写 SysTick, 必须用 tick_ms (跨模块时间基准)。
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "clock.h"  /* 同目录头文件 */
#include "Hardware/KEY/key.h"

volatile unsigned long tick_ms = 0;

int mspm0_delay_ms(unsigned long num_ms)
{
    uint32_t start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler(void)
{
    tick_ms++;
    KEY_Tick();   /* 推进按键状态机 (1ms 一次) */
}