/* ============================================================================
 * @file    tb6612.c
 * @brief   TB6612 编码电机驱动 (2 路 PWM + 方向 GPIO + 软件档位状态)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     TB6612_Init();                     // 两路全刹车 + PWM=0 + 档=[20%, 20%] + selected=L
 *
 *   while(1) / 按键回调:
 *     TB6612_Run(TB_MOTOR_L, TB_DIR_FWD);    // 左电机正转 (PWM=当前档)
 *     TB6612_Run(TB_MOTOR_R, TB_DIR_REV);    // 右电机反转
 *     TB6612_Stop(TB_MOTOR_L);               // 立即刹车 (IN1=IN2=1, PWM=0)
 *     TB6612_Stop(TB_MOTOR_R);
 *
 *     TB6612_NextLevel(TB6612_GetSelected()); // 切当前选中电机的 PWM 档 20→40→60→20
 *     TB6612_ToggleSelected();               // 长按 K4 切换 selected L ↔ R
 *
 *   LCD 读:
 *     TB6612_GetLevel(motor)  → 0/1/2 (档位)
 *     TB6612_GetPct(motor)    → 20/40/60 (百分比)
 *     TB6612_GetSelected()    → TB_MOTOR_L / TB_MOTOR_R
 *
 * ============================================================================
 * 硬件对照表 (syscfg 生成的宏)
 *   左电机 L (TB_MOTOR_L, 编码器 PB4/PB5 = bianma2):
 *     BIN1 = TB6612_BIN1_B23_PIN = DL_GPIO_PIN_23  (GPIOB)
 *     BIN2 = TB6612_BIN2_B27_PIN = DL_GPIO_PIN_27  (GPIOB)
 *     PWML = TB6612_PWM_C0       = TIMG12_CCP0     (PB13)
 *   右电机 R (TB_MOTOR_R, 编码器 PA28/PA31 = bianma1):
 *     AIN1 = TB6612_AIN1_B06_PIN = DL_GPIO_PIN_6   (GPIOB)
 *     AIN2 = TB6612_AIN2_B07_PIN = DL_GPIO_PIN_7   (GPIOB)
 *     PWMR = TB6612_PWM_C1       = TIMG12_CCP1     (PA25)
 *
 * ============================================================================
 * 设计要点
 *   1. PWM 周期 2000 @ 40MHz = 20kHz, 用 DL_TimerG_setCaptureCompareValue 改占空比
 *   2. AIN1/AIN2 走标准 GPIO 输出 (syscfg 已 initDigitalOutput + enableOutput,
 *      这里 setPins/clearPins 直接输出)
 *   3. 软件档位 (s_level[2]) 维护 0/1/2 三档, 与 s_pwm_comp[2] 同步,
 *      应用层不直接改 compare, 必须经过 NextLevel
 *   4. 切方向前先 Stop (刹车) + 短暂 NOP, 防 H 桥短路
 *   5. MSPM0 TIMG12 默认 PWM 是 active-low, set_pwm_comp 内部已反相
 *      (调用方传 active-high 直觉 compare 值即可)
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/TB6612/tb6612.h"

/* ============================================================================
 * 软件状态 (左右电机各一份; 上电 Init 时全归 0)
 * ============================================================================ */
static uint8_t  s_level[2]        = { 0U, 0U };       /* 当前档 (0/1/2) */
static uint16_t s_pwm_comp[2]     = { 0U, 0U };       /* 已应用的 compare 值 */
static tb_dir_t s_dir[2]          = { TB_DIR_FWD, TB_DIR_FWD };
static tb_motor_t s_selected      = TB_MOTOR_L;

/* ============================================================================
 * PWM 档位 -> compare 值表 (初始化一次, 表驱动)
 *   lv0=20% -> 400,  lv1=40% -> 800,  lv2=60% -> 1200 (TB_PWM_PERIOD=2000)
 *   用户 04:35 反馈: "PWM=20 显示应该是 20% 实际速度, 反了" → 复原为直观的
 *   PWM 越大 → 速度越快 (TB6612 物理特性: 占空比 = 平均电压 = 转速)
 *   注意: static const 数组初始化必须是编译期常量, 不能再调 TB_PWM_DUTY()
 * ============================================================================ */
static const uint16_t kLevelCompare[TB_PWM_LEVELS] = {
    400U,   /* 20% = lv0, 最慢 */
    800U,   /* 40% = lv1 */
    1200U,  /* 60% = lv2, 最快 */
};

/* 档位 → 百分比 (同上约束, 必须常量; 跟 kLevelCompare 同步正向) */
static const uint8_t kLevelPct[TB_PWM_LEVELS] = {
    20U, 40U, 60U,
};

/* ============================================================================
 * 内部辅助: 把 compare 值推到对应 PWM 通道
 *   L (左电机)  → TIMG12 CCP0 (PB13)
 *   R (右电机)  → TIMG12 CCP1 (PA25)
 *
 *   用户 04:38 反馈实测: comp=400 → 电机快, comp=1200 → 电机慢, 跟 TB6612 直觉反
 *     → MSPM0 TIMG12 默认 PWM 输出是 active-low (output LOW when CNT < compare)
 *     → 期望 HIGH 占空比 X% 时, 实际送 compare = period * (1 - X%)
 *   例如: 期望 20% 占空比 → compare = 2000 * 0.8 = 1600  (这样实际 HIGH 占空比 = 20%)
 *        期望 60% 占空比 → compare = 2000 * 0.4 = 800
 * ============================================================================ */
static inline void set_pwm_comp(tb_motor_t motor, uint16_t comp) {
    /* 反相: comp 是"调用方期望的占空比对应的 compare 值" (active-high 直觉)
     * 送进硬件的是 (period - comp), 抵消 MSPM0 默认 active-low 行为
     */
    uint16_t actual = (uint16_t)(TB_PWM_PERIOD - comp);
    if (motor == TB_MOTOR_L) {
        DL_TimerG_setCaptureCompareValue(TB6612_PWM_INST, actual,
                                         DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(TB6612_PWM_INST, actual,
                                         DL_TIMER_CC_1_INDEX);
    }
}

/* 内部: 设 A 路 IN1/IN2, B 路走另一组宏 */
static inline void set_in(tb_motor_t motor, uint8_t in1, uint8_t in2) {
    if (motor == TB_MOTOR_R) {
        /* 右电机 = AIN1=PB6, AIN2=PB7 */
        if (in1) DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_B06_PIN);
        else     DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_B06_PIN);
        if (in2) DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_B07_PIN);
        else     DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_B07_PIN);
    } else {
        /* 左电机 = BIN1=PB23, BIN2=PB27 */
        if (in1) DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_B23_PIN);
        else     DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_B23_PIN);
        if (in2) DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_B27_PIN);
        else     DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_B27_PIN);
    }
}

/* ============================================================================
 * API
 * ============================================================================ */
void TB6612_Init(void) {
    /* 上电: 两路全部刹车 (IN1=IN2=1) + PWM=0 */
    s_level[0]    = 0U;
    s_level[1]    = 0U;
    s_dir[0]      = TB_DIR_FWD;
    s_dir[1]      = TB_DIR_FWD;
    s_pwm_comp[0] = TB_PWM_DUTY_OFF;
    s_pwm_comp[1] = TB_PWM_DUTY_OFF;
    s_selected    = TB_MOTOR_L;

    set_in(TB_MOTOR_L, 1U, 1U);
    set_in(TB_MOTOR_R, 1U, 1U);
    set_pwm_comp(TB_MOTOR_L, TB_PWM_DUTY_OFF);
    set_pwm_comp(TB_MOTOR_R, TB_PWM_DUTY_OFF);

    /* SysConfig 已在 TB6612_PWM_init() 里把 TIMG12 enable, 不需要再开 */
}

void TB6612_Stop(tb_motor_t motor) {
    /* 刹车 = IN1=IN2=1 (低阻短路, 快衰减), PWM=0 */
    set_in(motor, 1U, 1U);
    set_pwm_comp(motor, TB_PWM_DUTY_OFF);
    s_pwm_comp[(uint8_t)motor] = TB_PWM_DUTY_OFF;
}

void TB6612_Run(tb_motor_t motor, tb_dir_t dir) {
    /* 切方向前先停 (刹车), 让 H 桥短暂脱离再反向 —
     * 省去也能跑 (TB6612 真值表反转时不会短路), 但物理机械冲击小更稳 */
    if (s_dir[(uint8_t)motor] != dir) {
        set_pwm_comp(motor, TB_PWM_DUTY_OFF);
        s_pwm_comp[(uint8_t)motor] = TB_PWM_DUTY_OFF;
    }

    /* 方向映射 (用户 04:06 反馈: R 电机方向反)
     *   用户 02:55 反馈已对 L/R 都翻一次 (IN1=0,IN2=1=FWD), 但实测 R 电机方向仍反。
     *   物理接线决定 R 路 (AIN1=PB6/AIN2=PB7) 的方向语义跟 L 路 (BIN1=PB23/BIN2=PB27)
     *   相反, 因此 R 路需要再翻一次 IN1/IN2 才能跟 L 路一致:
     *     L: TB_DIR_FWD → IN1=0, IN2=1 (保持)
     *     R: TB_DIR_FWD → IN1=1, IN2=0 (再翻一次)
     *   这样 K2 down → 两电机物理上都按用户期望的"正"转
     */
    if (motor == TB_MOTOR_R) {
        if (dir == TB_DIR_FWD) {
            set_in(motor, 1U, 0U);   /* R 路: 物理正转 (再翻一次) */
        } else {
            set_in(motor, 0U, 1U);   /* R 路: 物理反转 */
        }
    } else {
        if (dir == TB_DIR_FWD) {
            set_in(motor, 0U, 1U);   /* L 路: 物理正转 */
        } else {
            set_in(motor, 1U, 0U);   /* L 路: 物理反转 */
        }
    }
    s_dir[(uint8_t)motor] = dir;

    /* 把当前档对应的 compare 重新推到 PWM 通道 */
    uint8_t lv = s_level[(uint8_t)motor];
    uint16_t comp = kLevelCompare[lv < TB_PWM_LEVELS ? lv : 0U];
    set_pwm_comp(motor, comp);
    s_pwm_comp[(uint8_t)motor] = comp;
}

void TB6612_NextLevel(tb_motor_t motor) {
    uint8_t lv = s_level[(uint8_t)motor];
    lv = (uint8_t)((lv + 1U) % TB_PWM_LEVELS);
    s_level[(uint8_t)motor] = lv;

    uint16_t comp = kLevelCompare[lv];
    set_pwm_comp(motor, comp);
    s_pwm_comp[(uint8_t)motor] = comp;

    /* 关键约束: 档位改但 IN1/IN2 不动 — 如果当前是刹车态 (IN1=IN2=1), 推完 compare 也不会转;
     * TB6612 的 IN=11 是刹车, PWM=任何值电机都不转. 用户需要按 K2/K3 才能让电机真正转. */
}

uint8_t TB6612_GetLevel(tb_motor_t motor) {
    return s_level[(uint8_t)motor];
}

uint8_t TB6612_GetPct(tb_motor_t motor) {
    uint8_t lv = s_level[(uint8_t)motor];
    return kLevelPct[lv < TB_PWM_LEVELS ? lv : 0U];
}

tb_motor_t TB6612_GetSelected(void) {
    return s_selected;
}

void TB6612_ToggleSelected(void) {
    s_selected = (s_selected == TB_MOTOR_L) ? TB_MOTOR_R : TB_MOTOR_L;
}
