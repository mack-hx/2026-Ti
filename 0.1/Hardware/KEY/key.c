/**
 * @file    key.c
 * @brief   5 个按键事件型驱动: KEY_Tick() 推进内部状态机, key() 查询事件
 *
 *  内部状态机 (每个键独立):
 *    S0_IDLE      松开, 无事件
 *    S1_PRESSED   按下, 已发 down, 等待 long (达到 KEY_LONG_MS) 或 up
 *    S2_LONGED    按下, 已发 long, 等待 up (按住期间不再发任何事件)
 *    S3_UP        松开, 已发 up, 等待下一次按下
 *
*  使用方式:
 *    1. main() 里 KEY_Init();                       // 上电一次
 *    2. SysTick_Handler 每 1ms 调 KEY_Tick();      // 推进内部状态机
 *    3. while(1) 里:
 *         if (key(1, down)) ...   // K1 下降沿 (松开→按下)
 *         if (key(1, up))   ...   // K1 上升沿 (按下→松开)
 *         if (key(1, long)) ...   // K1 长按 (按住 ≥1500ms)
 *
 *  按键低电平有效. K1~K4 板载上拉, K5/PB21 在 KEY_Init 里手动开内部上拉.
 *  短按一下: S0 -> (下降沿) S1 -> 发 down -> (松开) -> S3 -> 发 up -> S0
 *  长按一下: S0 -> (下降沿) S1 -> 发 down -> (按住 KEY_LONG_MS) -> 发 long -> S2
 *          -> (松开) -> S3 -> 发 up -> S0
 *
 *  注意: K5 (PB21) syscfg 配的是 RESISTOR_NONE, 在 KEY_Init 里手动开内部上拉.
 *        其他 K1~K4 板子上有上拉, 保持 RESISTOR_NONE 不动.
 */
#include "ti_msp_dl_config.h"
#include "system/clock.h"   /* tick_ms */
#include "Hardware/KEY/key.h"

/* SysTick 节拍: 1ms (clock.c 里 DL_SYSTICK_config(CPUCLK_FREQ/1000) +
 *   SysTick_Handler 每 ms 增 tick_ms). 如果以后改 SysTick 频率, 这里同步改 */

/* 内部状态机阶段 */
typedef enum {
    KS_IDLE = 0,     /* 松开, 无事件 */
    KS_PRESSED,      /* 按下, 已发 down, 等 long/up */
    KS_LONGED,       /* 按下, 已发 long, 等 up */
    KS_RELEASED,     /* 松开, 已发 up, 等下一次按下 */
} key_state_t;

/* 每个键的运行态 (下标 = id - 1) */
typedef struct {
    key_state_t   state;
    uint32_t      down_ms;     /* 进入 PRESSED/LONGED 的时刻 (ms) */
    key_type_t    pending;     /* 一次性未消费事件 (down/up/long_press), key() 读后清零 */
} key_run_t;

static key_run_t s_keys[5];

/* ============================================================================
 * 硬件读取
 * ============================================================================ */
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

static inline uint32_t now_ms(void) {
    return (uint32_t)tick_ms;
}

/* ============================================================================
 * API
 * ============================================================================ */
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

void KEY_Tick(void) {
    uint32_t t = now_ms();

    for (uint32_t i = 0; i < 5; i++) {
        key_run_t *k = &s_keys[i];
        bool pressed = (read_pin((uint8_t)(i + 1)) == 0U);  /* 低电平 = 按下 */

        switch (k->state) {
        case KS_IDLE:
            /* 松开, 等下一个下降沿 */
            if (pressed) {
                k->state   = KS_PRESSED;
                k->down_ms = t;
                k->pending = down;
            }
            break;

        case KS_PRESSED:
            /* 按下, down 已挂出. 看是否到长按阈值, 或提前松开 */
            if (!pressed) {
                /* 提前松开, 没攒到 long → 正常 down/up, 不补 long */
                k->state   = KS_RELEASED;
                k->pending = up;
            } else if ((t - k->down_ms) >= KEY_LONG_MS) {
                /* 攒到 long → 发 long, 进入 LONGED 等松开 */
                k->state   = KS_LONGED;
                k->pending = long_press;
            } else {
                /* 还在按, 没到 long, 啥也不做 */
            }
            break;

        case KS_LONGED:
            /* 按下且已发 long, 等松开 */
            if (!pressed) {
                k->state   = KS_RELEASED;
                k->pending = up;
            }
            break;

        case KS_RELEASED:
            /* up 已挂出, 等回到 IDLE. 物理上松开 → 进 IDLE; 还在按 → 等真松开 */
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

bool key_pressed(uint8_t id) {
    if (id < 1 || id > 5) {
        return false;
    }
    return read_pin(id) == 0U;
}