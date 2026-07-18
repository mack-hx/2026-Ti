/* ============================================================================
 * @file    led.h
 * @brief   LED (PB22) 驱动 - 单 GPIO 高/低控制 + 心跳翻转
 *
 *   硬件: PB22, 输出推挽 (syscfg 已配)
 *   电平: 高电平点亮, 低电平熄灭 (用户板上为低电平灭 → 高电平亮)
 *
 *   应用层用法 (主循环):
 *     LED_Init();                        // 上电调一次
 *     LED_On(); / LED_Off(); / LED_Toggle();
 *     LED_Heartbeat(tick_ms, 500);       // tick_ms 来自 clock.h, 500ms 自动翻转
 *
 * ============================================================================
 */
#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

/* ============================================================================
 * API
 * ============================================================================ */

/* LED 初始化 */
void LED_Init(void);

/* 点亮/熄灭/切换 */
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

/* 心跳: 每次调用，LED 每 interval_ms 翻转一次 */
void LED_Heartbeat(uint32_t now_ms, uint32_t interval_ms);

#endif
