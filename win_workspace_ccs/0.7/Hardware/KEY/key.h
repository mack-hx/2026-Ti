/* ============================================================================
 * @file    key.h
 * @brief   5 个按键 (K1=PB0 / K2=PB1 / K3=PA22 / K4=PB24 / K5=PB21) 事件型驱动
 *
 *   模块命名含义:
 *     - key = 按键 (事件型)
 *     - KEY_* = 按键相关宏定义
 *     - key_* = 按键状态/事件
 *     - key_pressed = 当前物理电平
 *
 *   事件类型 (命名: down=按下沿 / up=松开沿 / long_press=长按):
 *     - down = 下降沿: 刚从松开变成按下 (短按的瞬间)
 *     - up = 上升沿: 刚从按下变成松开 (松开的瞬间)
 *     - long_press = 长按: 按下后保持 ≥ KEY_LONG_MS (1500ms, 只触发一次)
 *
 *   应用层用法:
 *     1. main() 上电调一次 KEY_Init();
 *     2. 主循环用 key(id, type) 查事件 (一次性消费)
 *     3. 用 key_pressed(id) 看当前电平 (LCD 显示用)
 *     4. KEY_Tick() 已在 clock.c 的 SysTick_Handler 里自动调
 *
 * ============================================================================
 */
#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 硬件定义 (KEY_* = 按键宏定义)
 * ============================================================================ */
/* K1/K2/K4/K5 在 GPIOB, K3 在 GPIOA */
#define KEY_PORT     key_K1_B00_PORT

#define KEY1_PIN     key_K1_B00_PIN      /* K1 = PB0 */
#define KEY2_PIN     key_K2B_01_PIN      /* K2 = PB1 */
#define KEY4_PIN     key_K4_B24_PIN      /* K4 = PB24 */
#define KEY5_PIN     key_USE_key_B21_PIN  /* K5 = PB21 */

#define KEY3_PORT    key_K3_A22_PORT     /* K3 = PA22 */
#define KEY3_PIN     key_K3_A22_PIN

/* ============================================================================
 * 事件类型 (key_type_t)
 * ============================================================================ */
typedef enum {
    down = 1,   /* down: 下降沿 (松开 → 按下) */
    up,         /* up: 上升沿 (按下 → 松开) */
    long_press, /* long_press: 长按 (按下 ≥ 1500ms, 只触发一次) */
} key_type_t;

/* 长按阈值 (ms) */
#define KEY_LONG_MS     1500U

/**
 * @brief   KEY_Init - 初始化按键驱动
 * @note    给 K5 (PB21) 开内部上拉 (syscfg 配的是 RESISTOR_NONE)
 * @usage   main() 里上电调一次
 */
void KEY_Init(void);

/**
 * @brief   KEY_Tick - 推进按键状态机
 * @note    每 1ms 调一次, 产生 down/up/long_press 事件
 * @usage   SysTick_Handler 里自动调用, 应用层无需关心
 */
void KEY_Tick(void);

/**
 * @brief   key - 查询按键事件
 * @param   id    按键 ID (1~5)
 * @param   type  事件类型 (down/up/long_press)
 * @return  true=事件刚发生 (一次性, 读后自动清) / false=无事件
 * @usage   主循环里查询, 一次性消费
 */
bool key(uint8_t id, key_type_t type);

/**
 * @brief   key_pressed - 查询当前物理电平
 * @param   id  按键 ID (1~5)
 * @return  true=按下 (低电平) / false=松开 (高电平)
 * @usage   LCD 显示当前按键状态
 */
bool key_pressed(uint8_t id);

#endif
