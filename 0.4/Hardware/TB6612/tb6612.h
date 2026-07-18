/* ============================================================================
 * @file    tb6612.h
 * @brief   TB6612 编码电机驱动 - 2 路有刷直流 + H 桥方向 + PWM 调速
 *
 *   电机 A: AIN1=PB6, AIN2=PB7, PWMA=PB13 (TIMG12_CCP0)
 *   电机 B: BIN1=PB23, BIN2=PB27, PWMB=PA25 (TIMG12_CCP1)
 *   编码器 A: PA28 / PA31 (Encoder.h)
 *   编码器 B: PB4  / PB5
 *
 *   应用层用法 (主循环):
 *     1. main() 里 TB6612_Init();                    ← 上电全刹车
 *     2. TB6612_Run(A, TB_DIR_FWD);                  ← 正转 (PWM = 当前档)
 *     3. TB6612_Stop(A);                             ← 刹车 (IN1=IN2=1, PWM=0)
 *     4. TB6612_NextLevel(A);                        ← PWM 档 20% → 40% → 60%
 *     5. TB6612_GetLevel(A) / GetPct(A);             ← 读档位 0/1/2 或 20/40/60
 *     6. TB6612_GetSelected() / ToggleSelected();    ← 长按 K4 切 selected
 *
 * ============================================================================
 */
#ifndef __TB6612_H__
#define __TB6612_H__

#include <stdint.h>
#include <stdbool.h>

/* 电机下标 (用户 02:55 约定: L=左, R=右)
 *   TB_MOTOR_L = 左电机 = BIN1=B23/BIN2=B27, 编码器 PB4/PB5 (bianma2)
 *   TB_MOTOR_R = 右电机 = AIN1=B06/AIN2=B07, 编码器 PA28/PA31 (bianma1) */
typedef enum {
    TB_MOTOR_L = 0,
    TB_MOTOR_R = 1,
} tb_motor_t;

/* 方向 */
typedef enum {
    TB_DIR_FWD = 0,   /* IN1=1, IN2=0 */
    TB_DIR_REV = 1,   /* IN1=0, IN2=1 */
} tb_dir_t;

/* PWM 档 (3 档 20/40/60 %) */
#define TB_PWM_LEVELS  3U

/* PWM 计算常量 (与 syscfg TB6612_PWM 周期匹配) */
#define TB_PWM_PERIOD   2000U
#define TB_PWM_DUTY_OFF 0U     /* 占空比 0%  = 停转 */

static inline uint16_t TB_PWM_DUTY(uint8_t pct) {
    /* pct ∈ [0, 100] -> compare 值 [0, 2000] */
    if (pct > 100U) pct = 100U;
    return (uint16_t)((uint32_t)pct * TB_PWM_PERIOD / 100U);
}

/* 上电: 把 AIN1/AIN2/BIN1/BIN2 全部拉到刹车态 (1,1),
 *      PWM 设为 0 (停止), 软件档位初始化为 [0, 0] = 20%, 选中 = A */
void TB6612_Init(void);

/* 立即刹车: IN1=IN2=1 (低阻短路) + PWM=0 */
void TB6612_Stop(tb_motor_t motor);

/* 运行 (按 dir 设 IN1/IN2; PWM 维持当前档对应的 compare 值)
 *   - 切方向前自动 Stop (刹车), 间隔 1ms 内多次切方向不会烧桥
 *   - PWM 由 s_duty_comp[motor] 决定, 用户用 TB6612_NextLevel 改档 */
void TB6612_Run(tb_motor_t motor, tb_dir_t dir);

/* 档位轮转: 20 -> 40 -> 60 -> 20, 并立即把新档应用到 PWM
 *   - 只动当前档, 不改方向 (Run 之前的方向保持不变) */
void TB6612_NextLevel(tb_motor_t motor);

/* 读当前档位 (0/1/2) */
uint8_t TB6612_GetLevel(tb_motor_t motor);

/* 读当前档对应百分比 (20/40/60) */
uint8_t TB6612_GetPct(tb_motor_t motor);

/* 当前选中电机 (用户长按 K4 切换)
 *   - K4 短按 NextLevel 只动 selected 那个
 *   - 应用层 K2 正转 / K3 反转时, 两路电机一起动 */
tb_motor_t TB6612_GetSelected(void);
void        TB6612_ToggleSelected(void);

#endif /* __TB6612_H__ */
