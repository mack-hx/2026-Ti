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
