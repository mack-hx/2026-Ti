/**
 * @file    led.h
 * @brief   LED (PB22) 驱动
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
