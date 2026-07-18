/* ============================================================================
 * @file    led.c
 * @brief   LED (PB22) 驱动
 *
 * ============================================================================
 * 命令调用方式 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 里调一次):
 *     LED_Init();                        // PB22 初始化为低电平 (灭)
 *
 *   while(1):
 *     LED_On();                          // 点亮 (高电平)
 *     LED_Off();                         // 熄灭 (低电平)
 *     LED_Toggle();                      // 翻转
 *     LED_Heartbeat(tick_ms, 500);       // 心跳: 每 500ms 自动翻转
 *                                        // (tick_ms 来自 clock.h)
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/LED/led.h"

#define LED_PIN   DL_GPIO_PIN_22

static uint32_t s_last_toggle_ms = 0;

/* ============================================================================
 * API
 * ============================================================================ */
void LED_Init(void) {
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void LED_On(void) {
    DL_GPIO_setPins(LED_PORT, LED_PIN);
}

void LED_Off(void) {
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void LED_Toggle(void) {
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

/**
 * @brief  心跳：每 interval_ms 翻转一次
 * @param  now_ms     当前系统时间 (ms)
 * @param  interval_ms 翻转间隔
 */
void LED_Heartbeat(uint32_t now_ms, uint32_t interval_ms) {
    if (now_ms - s_last_toggle_ms >= interval_ms) {
        LED_Toggle();
        s_last_toggle_ms = now_ms;
    }
}
