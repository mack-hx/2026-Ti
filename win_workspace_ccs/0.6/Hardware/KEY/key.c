/* ============================================================================
 * @file    key.c
 * @brief   5 个按键事件型驱动: KEY_Tick() 推进内部状态机, key() 查询事件
 *
 * ============================================================================
 * 模块命名含义:
 *   s_keys = 按键状态数组 (5 个按键)
 *   key_run_t = 按键运行态结构
 *   KS_* = 按键状态 (Key State)
 *
 * ============================================================================
 * 内部状态机 (key_state_t):
 *   KS_IDLE     = 松开态, 无事件, 等下降沿
 *   KS_PRESSED  = 按下态, 已发 down, 等 long 或 up
 *   KS_LONGED   = 长按态, 已发 long, 等 up
 *   KS_RELEASED = 释放态, 已发 up, 等下一次按下
 *
 * 状态转换:
 *   KS_IDLE → (下降沿) → KS_PRESSED → 发 down
 *   KS_PRESSED → (松开) → KS_RELEASED → 发 up → KS_IDLE
 *   KS_PRESSED → (≥1500ms) → KS_LONGED → 发 long → 等 up
 *   KS_LONGED → (松开) → KS_RELEASED → 发 up → KS_IDLE
 *
 * ============================================================================
 * 调用方法:
 *   上电: KEY_Init();
 *   运行: KEY_Tick() 在 SysTick_Handler 每 1ms 自动调用
 *   查询: key(id, type) 在主循环查询 (一次性消费)
 *         key_pressed(id) 查询当前物理电平
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "system/clock.h"   /* tick_ms */
#include "Hardware/KEY/key.h"

/* SysTick 节拍: 1ms (clock.c 里 DL_SYSTICK_config(CPUCLK_FREQ/1000) +
 *   SysTick_Handler 每 ms 增 tick_ms). 如果以后改 SysTick 频率, 这里同步改 */

/* 按键状态 (Key State) */
typedef enum {
    KS_IDLE = 0,     /* KS_IDLE: 松开态, 无事件, 等下降沿 */
    KS_PRESSED,      /* KS_PRESSED: 按下态, 已发 down, 等 long 或 up */
    KS_LONGED,       /* KS_LONGED: 长按态, 已发 long, 等 up */
    KS_RELEASED,     /* KS_RELEASED: 释放态, 已发 up, 等下一次按下 */
} key_state_t;

/* 按键运行态结构 (key_run_t) */
typedef struct {
    key_state_t   state;       /* 当前状态 */
    uint32_t      down_ms;     /* 进入 PRESSED/LONGED 的时刻 (ms) */
    key_type_t    pending;     /* 一次性未消费事件 (down/up/long_press), key() 读后清零 */
} key_run_t;

static key_run_t s_keys[5];

/* ============================================================================
 * 硬件读取
 * ============================================================================ */

/* read_pin: 读取指定按键的 GPIO 引脚电平
 * @param id 按键 ID (1~5)
 * @return 引脚电平 (0=按下, 1=松开) */
static inline uint32_t read_pin(uint8_t id) {
    switch (id) {
        case 1: return DL_GPIO_readPins(KEY_PORT,  KEY1_PIN);
        case 2: return DL_GPIO_readPins(KEY_PORT,  KEY2_PIN);
        case 3: return DL_GPIO_readPins(KEY3_PORT, KEY3_PIN);  /* K3 在 GPIOA */
        case 4: return DL_GPIO_readPins(KEY_PORT,  KEY4_PIN);
        case 5: return DL_GPIO_readPins(KEY_PORT,  KEY5_PIN);  /* PB21, 同 PORT */
        default: return 1U;  /* 兜底: 报为松开 */
    }
}

/* now_ms: 获取当前时间 (tick_ms 快照) */
static inline uint32_t now_ms(void) {
    return (uint32_t)tick_ms;
}

/* ============================================================================
 * API 实现
 * ============================================================================ */

/* KEY_Init: 初始化按键驱动 */
void KEY_Init(void) {
    /* PB21 强制启用内部上拉 (覆盖 syscfg 的 RESISTOR_NONE) */
    DL_GPIO_initDigitalInputFeatures(key_USE_key_B21_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    for (uint32_t i = 0; i < 5; i++) {
        s_keys[i].state   = KS_IDLE;
        s_keys[i].down_ms = 0U;
        s_keys[i].pending = 0;  /* 0 = 无事件 (down=1/up=2/long=3) */
    }
}

/* KEY_Tick: 推进按键状态机 (每 1ms 调用一次) */
void KEY_Tick(void) {
    uint32_t t = now_ms();

    for (uint32_t i = 0; i < 5; i++) {
        key_run_t *k = &s_keys[i];
        bool pressed = (read_pin((uint8_t)(i + 1)) == 0U);  /* 低电平 = 按下 */

        switch (k->state) {
        case KS_IDLE:
            /* 松开态, 等下一个下降沿 */
            if (pressed) {
                k->state   = KS_PRESSED;
                k->down_ms = t;
                k->pending = down;  /* 发 down 事件 */
            }
            break;

        case KS_PRESSED:
            /* 按下态, down 已挂出. 看是否到长按阈值, 或提前松开 */
            if (!pressed) {
                /* 提前松开, 没攒到 long → 正常 down/up, 不补 long */
                k->state   = KS_RELEASED;
                k->pending = up;  /* 发 up 事件 */
            } else if ((t - k->down_ms) >= KEY_LONG_MS) {
                /* 攒到 long → 发 long, 进入 LONGED 等松开 */
                k->state   = KS_LONGED;
                k->pending = long_press;  /* 发 long_press 事件 */
            }
            /* 还在按, 没到 long, 啥也不做 */
            break;

        case KS_LONGED:
            /* 长按态, long 已挂出, 等松开 */
            if (!pressed) {
                k->state   = KS_RELEASED;
                k->pending = up;  /* 发 up 事件 */
            }
            break;

        case KS_RELEASED:
            /* 释放态, up 已挂出, 等回到 IDLE */
            if (!pressed) {
                k->state = KS_IDLE;
            }
            break;

        default:
            k->state = KS_IDLE;
            break;
        }
    }
}

/* key: 查询按键事件 (一次性消费) */
bool key(uint8_t id, key_type_t type) {
    if (id < 1 || id > 5) {
        return false;
    }
    key_run_t *k = &s_keys[id - 1];
    if (k->pending == type) {
        k->pending = 0;  /* 一次性消费 */
        return true;
    }
    return false;
}

/* key_pressed: 查询当前物理电平 */
bool key_pressed(uint8_t id) {
    if (id < 1 || id > 5) {
        return false;
    }
    return read_pin(id) == 0U;
}
