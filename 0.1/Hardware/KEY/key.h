/**
 * @file    key.h
 * @brief   5 个按键 (K1=PB0 / K2=PB1 / K3=PA22 / K4=PB24 / K5=PB21)
 *
 */
#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 硬件定义
 * ============================================================================
 * K1/K2/K4/K5 在 GPIOB, K3 在 GPIOA
 * ============================================================================
 */
#define KEY_PORT     key_K1_B00_PORT

#define KEY1_PIN     key_K1_B00_PIN
#define KEY2_PIN     key_K2B_01_PIN
#define KEY4_PIN     key_K4_B24_PIN
#define KEY5_PIN     key_USE_key_B21_PIN

#define KEY3_PORT    key_K3_A22_PORT
#define KEY3_PIN     key_K3_A22_PIN

/* ============================================================================
 * 事件类型 (key() 的第二个参数)
 * ============================================================================ */
typedef enum {
    down = 1,   /* 下降沿: 刚从松开变成按下 */
    up,         /* 上升沿: 刚从按下变成松开 */
    long_press, /* 长按: 按下后保持 ≥ KEY_LONG_MS (1500ms, 只触发一次) */
} key_type_t;

/* 长按阈值 (ms) — 按住多久算 long */
#define KEY_LONG_MS     1500U

/* 初始化: 给 K5 (PB21) 开内部上拉, 读一次基线电平 */
void KEY_Init(void);

/* 每 1ms 调一次 (建议在 SysTick_Handler 里): 推进内部状态机, 产生 down/up/long */
void KEY_Tick(void);

/* 主查询函数: 查第 id 个键是否刚刚发生了 type 类型的事件.
 *   id: 1~5 (1=K1, 2=K2, 3=K3, 4=K4, 5=K5)
 *   type: down / up / long_press
 *   返回: true = 该事件刚发生 (一次性, 读后自动清)
 *         false = 没发生 (或已经消费过) */
bool key(uint8_t id, key_type_t type);

/* 当前物理电平: true=按下 (低电平). 用于 LCD 显示 "按了哪个" */
bool key_pressed(uint8_t id);

#endif