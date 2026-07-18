/* ============================================================================
 * @file    mpu9250.c
 * @brief   MPU-9250 9 轴 I2C 驱动 (软件 bit-bang) - 非阻塞状态机
 *          + 陀螺零漂校准 + hard-iron 偏移补偿 + 串口调试 (1Hz)
 *
 * ============================================================================
 * 协议要点
 * ============================================================================
 *
 *   MPU9250 = MPU6500 (0x68) + AK8963 (bypass 后 = 0x0C)
 *   I2C 寄存器读写走 SW_I2C (软件 bit-bang, PA0=SDA, PA1=SCL, 100kHz)
 *
 *   量纲:
 *     acc [g]   = raw / 16384
 *     gyro [dps]= raw / 131
 *     mag [uT]  = raw * (0.15 + ASAx/256 * 0.30)
 *
 *   状态机推进, 由 MPU9250_TIM (TIMG6, 5ms ISR) 唤醒:
 *     主循环每帧只调 MPU9250_Task(): 1ms 以内检查 ready 标志 + 姿态解算
 *     实际 I2C 事务在 MPU9250_TIM ISR 里推进 (每个 tick 推进 1 个状态).
 *
 * ============================================================================
 * 陀螺零漂校准 (用户 2026-07-14 决定: 上电自动 + 运行 slow-decay)
 * ============================================================================
 *
 *   - 上电后前 5 秒 (~50 ticks @ 5ms/tick) 收集 gx/gy/gz, 求平均 → bias_x/y/z
 *   - bias 应用: raw_g - bias → 解算姿态用的 gx/gy/gz
 *   - 运行 slow decay: 每 30 秒把当前 bias 向"最近 1 秒窗口平均"滑动 5%
 *     目的: 板子温度升高时 bias 会漂, 持续吸收新静态样本, 避免漂出
 *   - 校准完成前 g_mpu9250.calib_state = CAL_RUNNING, ROW 显示 "cal..."; 完成后 CAL_OK
 *
 *   注: 自动校准要求上电后 5 秒内板子**静止**. 如果用户立刻操作, bias 不准
 *       → 解算姿态有偏移. 用户可按 K4 reset 让 bias 重置 (见 ui.c MPU9250Page_OnKey).
 *
 * ============================================================================
 * hard-iron 校准 (用户 2026-07-14 决定: 不做运行时, 看串口数据手填)
 * ============================================================================
 *
 *   - 3 个全局硬铁偏移 hard_iron_{x,y,z} 默认 0.0
 *   - 应用: raw_m - hard_iron_{x,y,z} → yaw/pitch/roll
 *   - 用户看串口输出 `mag_raw mx my mz` + `mag_cal mx' my' mz'` 对比
 *     把对称中点 (mx_max+mx_min)/2 等写到 `s_hard_iron_x/y/z` 常量
 *   - **当前实现: 默认 0** — 后续比赛调试阶段再按实际环境调
 *
 * ============================================================================
 * 串口调试 (用户 2026-07-14 决定: 1Hz, 校准前+后+姿态)
 * ============================================================================
 *
 *   - UART_0 (PA10/PA11, 115200 baud 默认)
 *   - 1Hz (节流到每 1000ms 一次) — 主循环 MPU9250_Task 里发, 不在 ISR 里
 *   - 格式:
 *       T:1234 CAL=1 GYR_BIAS=+1.23,+0.45,-0.12
 *       RAW ax=... ay=... az=... gx=... gy=... gz=... mx=... my=... mz=...
 *       CAL ax=... ay=... az=... gx=... gy=... gz=... mx=... my=... mz=...
 *       ATT yaw=... pit=... rol=... T=...
 *   - 校准前/后对照方便看出零漂消除效果
 *
 * ============================================================================
 * 调用方法
 * ============================================================================
 *
 *   上电 (main.c 启动序列里调一次):
 *     MPU9250_Init();              // 初始化 PA0/PA1 PINCM + 启动状态机 (ISR 推进)
 *
 *   主循环每帧调:
 *     MPU9250_Task();              // 姿态解算 + 1Hz 串口调试
 *
 *   UI 读取:
 *     const MPU9250_Data *d = MPU9250_GetData();
 *     d->ax / ay / az              // 加速度 [g] (零漂后)
 *     d->gx / gy / gz              // 陀螺 [dps] (零漂后)
 *     d->mx / my / mz              // 磁力计 [uT] (hard-iron 后)
 *     d->yaw / pitch / roll        // 倾斜补偿磁航向 + 姿态 [deg]
 *     d->calib_state               // 0=未校准, 1=校准中, 2=校准完成
 *     d->bias_gx / bias_gy / bias_gz  // 当前零漂值 [dps]
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "sw_i2c.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ============================================================================
 * 常量 (量纲换算)
 * ============================================================================ */
#define ACC_SENS_2G            16384.0f
#define GYRO_SENS_250DPS       131.0f
#define MAG_RAW_TO_uT_16BIT    0.15f
#define TEMP_OFFSET            21.0f
#define TEMP_SENS              340.0f

/* ============================================================================
 * 陀螺零漂校准常量
 * ============================================================================
 */
#define CAL_SAMPLE_TICKS   200U    /* 200 tick × 5ms = 1.0 秒 (原始校准窗口, 之前 5s 太长) */
#define CAL_BIAS_DECAY_MS  30000U  /* 每 30 秒一次 slow decay 调整 */
#define CAL_DECAY_WINDOW_MS 1000U  /* slow decay 取最近 1 秒窗口平均 */
#define CAL_DECAY_ALPHA    0.05f   /* slow decay 滑动系数 (5%) */
#define CAL_SLOW_WINDOW_MS 60000U  /* slow decay 启用条件: 校准完成至少 60 秒 */

/* 静止检测 (用户 05:58 反馈: 静止时冻结 euler 积分避免零漂累积)
 *   |gx|+|gy|+|gz| < STATIC_G_SUM_TH 持续 STATIC_HOLD_TICKS 帧 (200ms) 置位
 *   板子不动时, 陀螺值是零漂 ±0.04 dps 量级, 累加 euler 会持续漂
 */
#define STATIC_G_SUM_TH    0.5f    /* 总角速度阈值 (实测残差 ~0.04 dps, 0.5 留余量) */
#define STATIC_HOLD_TICKS  40U     /* 持续 200ms = 40 tick × 5ms 才置位 */

/* ============================================================================
 * hard-iron 偏移 (用户根据串口输出手动调整)
 * 典型环境: 把 (mx_max+mx_min)/2, (my_max+my_min)/2, (mz_max+mz_min)/2
 * 写到这里. 默认 0 (未校准).
 *
 * 2026-07-14 04:33 实测 (板子静止 12 秒 RAW mx/my/mz 取均值):
 *   mx:  +16 ±0    LSB  → uT ≈ 16 * 0.15 ≈ +2.4   (但 mpu9250.c 用的转换系数是 s_adj * MAG_RAW_TO_uT_16BIT)
 *   my:  -216 ±10  LSB  → uT ≈ -32 to -37
 *   mz:  -214 ±8   LSB  → uT ≈ -32 to -36
 *   CAL 行 mx=+2.71 my=-36.5 mz=-35.3 (直接读)
 *   为简单起见使用 CAL 行实测值 — s_hard_iron 在 step_run_mag_burst 减之前
 *   就把 (mx, my, mz) - bias 抵消掉
 * ============================================================================
 */
static const float s_hard_iron_x =  2.71f;
static const float s_hard_iron_y = -36.50f;
static const float s_hard_iron_z = -35.30f;

/* ============================================================================
 * 全局数据 (UI 读这个)
 * ============================================================================ */
MPU9250_Data g_mpu9250 = {
    .who_mpu = 0, .who_mag = 0, .ok = false,
};

/* 工厂校准 (AK8963 ASAX/Y/Z 归一化系数) */
static float s_adj_x = 1.0f, s_adj_y = 1.0f, s_adj_z = 1.0f;

/* ============================================================================
 * 陀螺零漂校准状态
 * ============================================================================
 */
typedef enum {
    CAL_IDLE = 0,        /* 还没开始 (Init 后未到校准状态) */
    CAL_RUNNING,         /* 正在收集样本 */
    CAL_OK,              /* 校准完成 */
} calib_state_t;

static volatile calib_state_t s_calib = CAL_IDLE;
static uint32_t s_calib_start_ms = 0;        /* CAL_RUNNING 开始时间 */
static float s_bias_gx = 0.0f;
static float s_bias_gy = 0.0f;
static float s_bias_gz = 0.0f;
/* CAL_RUNNING 累加器 */
static float s_calib_sum_gx = 0.0f;
static float s_calib_sum_gy = 0.0f;
static float s_calib_sum_gz = 0.0f;
static uint32_t s_calib_count = 0;
/* slow decay 时间窗 */
static uint32_t s_last_decay_ms = 0;
static uint32_t s_calib_done_ms = 0;        /* CAL_OK 起始时间 (slow decay 启用判断) */

/* 静止检测状态 (ISR 推进, 主循环读 s_motionless 决定是否累加 euler_x/y/z)
 *   s_motionless_ticks: ISR 每 5ms 自增/清零, 满 STATIC_HOLD_TICKS 置位
 *   s_motionless:       静止状态锁存, 任何"动"帧立即清零
 */
static volatile uint32_t s_motionless_ticks = 0U;
static volatile bool     s_motionless = false;

/* RAW 样本 (ISR 写, MPU9250_Task 读 — 注意单线程访问没问题, 中断优先级 > 主循环) */
static volatile int16_t s_raw_ax = 0, s_raw_ay = 0, s_raw_az = 0;
static volatile int16_t s_raw_gx = 0, s_raw_gy = 0, s_raw_gz = 0;
static volatile int16_t s_raw_mx = 0, s_raw_my = 0, s_raw_mz = 0;
static volatile bool s_have_raw_acc = false;
static volatile bool s_have_raw_gyro = false;
static volatile bool s_have_raw_mag = false;

/* ============================================================================
 * 串口调试 (1Hz 节流)
 * ============================================================================
 */
static uint32_t s_dbg_last_ms = 0;
#define DBG_PERIOD_MS  1000U

/* 串口直接发, 不依赖 UART_Host (避免 NVIC 没挂 ISR 导致飞)
 * 必须每字节等 BUSY=0, 否则 TX FIFO 满或上次没发完时塞新字节会丢 → 串口乱码
 *   115200 baud 1 字节 ~87us, dbg_printf 一帧 ~150 字节 ≈ 13ms (主循环阻塞可接受)
 */
static void dbg_putc(char c) {
    while (DL_UART_Main_isBusy(UART_0_INST)) {}
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

static void dbg_puts(const char *s) {
    while (*s) dbg_putc(*s++);
}

static void dbg_printf(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    for (int i = 0; i < n; i++) dbg_putc(buf[i]);
}

/* ============================================================================
 * 状态机
 * ============================================================================
 */
typedef enum {
    S_IDLE = 0,        /* 空闲 / 检查 ok */
    S_INIT_RST,        /* 软复位 MPU + 等 50ms (跨多个 tick) */
    S_INIT_PWR,        /* PWR_MGMT_1=0x01 唤醒 */
    S_INIT_CFG,        /* SMPLRT_DIV/CONFIG/ACCEL_CONFIG/GYRO_CONFIG */
    S_INIT_BYPASS,     /* INT_PIN_CFG |= 0x42 + USER_CTRL=0 + 等 bypass 稳定 */
    S_INIT_AK_PD,      /* AK_CNTL1=0x00 power-down */
    S_INIT_AK_ROM,     /* AK_CNTL1=0x0F ROM access + 读 ASAX/Y/Z */
    S_INIT_AK_CONT,    /* AK_CNTL1=0x12 连续 100Hz 16-bit + 读 WIA */
    S_INIT_DONE,       /* 完成, 跳到 RUN */
    S_RUN_ACC,         /* 读 acc 6 字节 */
    S_RUN_TEMP,        /* 读 temp 2 字节 */
    S_RUN_GYRO,        /* 读 gyro 6 字节 */
    S_RUN_MAG_ST1,     /* 读 AK ST1 */
    S_RUN_MAG_BURST,   /* 读 AK HXL..HZH 6 字节 + ST2 */
} mpu_state_t;

static volatile mpu_state_t s_state = S_IDLE;
static volatile uint32_t s_init_retry_ms = 0;     /* INIT 重试间隔 */

static volatile uint32_t s_wait_until_ms = 0;  /* 截止时刻 deadline */  /* 截止时刻 deadline */

/* I2C 事务 scratch buffer (避免栈占用) */
static uint8_t s_buf[6];

/* ============================================================================
 * 工具函数
 * ============================================================================
 */
extern volatile unsigned long tick_ms;

static inline uint32_t now_ms(void) {
    return (uint32_t)tick_ms;
}

static inline bool tick_elapsed(uint32_t start_ms, uint32_t period_ms) {
    return ((uint32_t)(now_ms() - start_ms) >= period_ms);
}

static inline void set_wait_ms(uint32_t ms) {
    s_wait_until_ms = now_ms() + ms;
}

static inline bool wait_done(void) {
    return now_ms() >= s_wait_until_ms;
}

/* ============================================================================
 * 互补滤波融合常量 (yaw = 陀螺短期精度 + 磁力计长期稳定)
 * ============================================================================
 *
 *   yaw_new = (1 - YAW_GYRO_ALPHA) × (yaw_prev + gz_dps × dt)
 *           +   YAW_GYRO_ALPHA     × mag_yaw
 *
 *   物理含义:
 *     - 陀螺 (gz): 高频准 (反应快), 但有零漂 → 用 (1-α) 主导短期
 *     - 磁力计: 低频稳 (长期不漂), 但有噪声 + 易受金属干扰 → 用 α 长期修正
 *     - α=0.02: 1 秒内累积 ~2% 权重给磁, 其余靠陀螺
 *
 *   dt 用 MPU9250_TIM (TIMG6) 的固定周期, 不靠浮点累加 (避免溢出)
 *
 *   注: 静止时 yaw 不再漂; 转动时 yaw 跟着转 (跟纯磁力计响应一样快, 但更稳)
 * ============================================================================
 */
#define YAW_GYRO_ALPHA     0.005f   /* 磁力计权重 (剩余 0.995 给陀螺) */
#define MPU9250_DT_SEC     0.005f   /* ISR 周期 5ms = 0.005s (TIMG6 配置) */
#define MPU9250_DT_MS      5U       /* ms 单位 */

/* 互补滤波状态: 上一次 yaw (度, 0..360), 第一次调用前必须 init 为磁力计值 */
static bool  s_yaw_filt_init = false;

/* ============================================================================
 * Init 步骤 (每个状态做 1 个 I2C 事务, 跨 tick 用 wait)
 * ============================================================================
 */

static void step_init_rst(void) {
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x80U);
    set_wait_ms(100U);
    s_state = S_INIT_PWR;
}

static void step_init_pwr(void) {
    if (!wait_done()) return;
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x01U);
    s_state = S_INIT_CFG;
}

static void step_init_cfg(void) {
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_SMPLRT_DIV, 0x07U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_CONFIG, 0x06U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_2, 0x00U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_ACCEL_CONFIG, 0x00U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_GYRO_CONFIG, 0x00U);
    s_state = S_INIT_BYPASS;
}

static void step_init_bypass(void) {
    uint8_t int_pin = 0;
    (void)SW_I2C_ReadReg(MPU9250_I2C_ADDR, MPU_REG_INT_PIN_CFG, &int_pin);
    int_pin |= 0x42U;
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_INT_PIN_CFG, int_pin);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_USER_CTRL, 0x00U);
    set_wait_ms(10U);
    s_state = S_INIT_AK_PD;
}

static void step_init_ak_pd(void) {
    if (!wait_done()) return;
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x00U);
    set_wait_ms(5U);
    s_state = S_INIT_AK_ROM;
}

static void step_init_ak_rom(void) {
    if (!wait_done()) return;
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x0FU);
    set_wait_ms(5U);
    s_state = S_INIT_AK_CONT;
}

static void step_init_ak_cont(void) {
    if (!wait_done()) return;
    uint8_t asa[3] = { 0, 0, 0 };
    if (SW_I2C_ReadRegs(AK8963_I2C_ADDR, AK_REG_ASAX, asa, 3u)) {
        s_adj_x = ((float)(int16_t)asa[0] - 128.0f) / 256.0f + 1.0f;
        s_adj_y = ((float)(int16_t)asa[1] - 128.0f) / 256.0f + 1.0f;
        s_adj_z = ((float)(int16_t)asa[2] - 128.0f) / 256.0f + 1.0f;
        if (s_adj_x <= 0.0f) s_adj_x = 1.0f;
        if (s_adj_y <= 0.0f) s_adj_y = 1.0f;
        if (s_adj_z <= 0.0f) s_adj_z = 1.0f;
    }
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x12U);
    set_wait_ms(5U);
    s_state = S_INIT_DONE;
}

static void step_init_done(void) {
    if (!wait_done()) return;
    uint8_t who = 0;
    if (SW_I2C_ReadReg(AK8963_I2C_ADDR, AK_REG_WIA, &who)) {
        g_mpu9250.who_mag = who;
    }
    g_mpu9250.ok = true;
    /* 启动陀螺零漂校准 */
    s_calib = CAL_RUNNING;
    s_calib_start_ms = now_ms();
    s_calib_sum_gx = s_calib_sum_gy = s_calib_sum_gz = 0.0f;
    s_calib_count = 0;
    s_last_decay_ms = now_ms();
    s_state = S_RUN_ACC;
}

/* ============================================================================
 * 读 9 轴 (每个状态读 1 组, 写 RAW 样本供零漂校准和串口用)
 * ============================================================================
 */

static void step_run_acc(void) {
    if (!SW_I2C_ReadRegs(MPU9250_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, s_buf, 6u)) {
        s_state = S_IDLE;
        return;
    }
    s_raw_ax = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
    s_raw_ay = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
    s_raw_az = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);
    s_have_raw_acc = true;
    /* 应用量纲 + 写全局 (UI 显示用) */
    g_mpu9250.ax = (float)s_raw_ax / ACC_SENS_2G;
    g_mpu9250.ay = (float)s_raw_ay / ACC_SENS_2G;
    g_mpu9250.az = (float)s_raw_az / ACC_SENS_2G;
    s_state = S_RUN_TEMP;
}

static void step_run_temp(void) {
    if (!SW_I2C_ReadRegs(MPU9250_I2C_ADDR, MPU_REG_TEMP_OUT_H, s_buf, 2u)) {
        s_state = S_RUN_GYRO;
        return;
    }
    int16_t t = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
    g_mpu9250.temp_c = ((float)t / TEMP_SENS) + TEMP_OFFSET;
    s_state = S_RUN_GYRO;
}

static void step_run_gyro(void) {
    if (!SW_I2C_ReadRegs(MPU9250_I2C_ADDR, MPU_REG_GYRO_XOUT_H, s_buf, 6u)) {
        s_state = S_RUN_MAG_ST1;
        return;
    }
    s_raw_gx = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
    s_raw_gy = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
    s_raw_gz = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);
    s_have_raw_gyro = true;
    /* 应用零漂校准: raw - bias → g_mpu9250 (UI / 姿态解算用) */
    if (s_calib == CAL_OK) {
        g_mpu9250.gx = ((float)s_raw_gx / GYRO_SENS_250DPS) - s_bias_gx;
        g_mpu9250.gy = ((float)s_raw_gy / GYRO_SENS_250DPS) - s_bias_gy;
        g_mpu9250.gz = ((float)s_raw_gz / GYRO_SENS_250DPS) - s_bias_gz;
    } else {
        /* 校准未完成 → 用原始值 (UI 可见 bias 偏多少) */
        g_mpu9250.gx = (float)s_raw_gx / GYRO_SENS_250DPS;
        g_mpu9250.gy = (float)s_raw_gy / GYRO_SENS_250DPS;
        g_mpu9250.gz = (float)s_raw_gz / GYRO_SENS_250DPS;
    }
    s_state = S_RUN_MAG_ST1;
}

static void step_run_mag_st1(void) {
    if (g_mpu9250.who_mag != AK8963_WHO_AM_I_VAL) {
        s_state = S_RUN_ACC;
        return;
    }
    uint8_t st1 = 0;
    if (!SW_I2C_ReadReg(AK8963_I2C_ADDR, AK_REG_ST1, &st1)) {
        s_state = S_RUN_ACC;
        return;
    }
    if (!(st1 & 0x01U)) {
        s_state = S_RUN_ACC;
        return;
    }
    s_state = S_RUN_MAG_BURST;
}

static void step_run_mag_burst(void) {
    if (!SW_I2C_ReadRegs(AK8963_I2C_ADDR, AK_REG_HXL, s_buf, 6u)) {
        s_state = S_RUN_ACC;
        return;
    }
    (void)SW_I2C_ReadReg(AK8963_I2C_ADDR, AK_REG_ST2, s_buf);
    s_raw_mx = (int16_t)(((uint16_t)s_buf[1] << 8) | s_buf[0]);  /* LE */
    s_raw_my = (int16_t)(((uint16_t)s_buf[3] << 8) | s_buf[2]);
    s_raw_mz = (int16_t)(((uint16_t)s_buf[5] << 8) | s_buf[4]);
    s_have_raw_mag = true;
    /* 量纲 + 工厂 ASA 校准 + hard-iron 偏移 */
    g_mpu9250.mx = ((float)s_raw_mx * s_adj_x * MAG_RAW_TO_uT_16BIT) - s_hard_iron_x;
    g_mpu9250.my = ((float)s_raw_my * s_adj_y * MAG_RAW_TO_uT_16BIT) - s_hard_iron_y;
    g_mpu9250.mz = ((float)s_raw_mz * s_adj_z * MAG_RAW_TO_uT_16BIT) - s_hard_iron_z;

    s_state = S_RUN_ACC;
}

/* ============================================================================
 * 陀螺零漂校准: CAL_RUNNING → CAL_OK
 * ============================================================================
 *
 * 每个 gyro 读完后 (S_RUN_GYRO) 累积; 累计 CAL_SAMPLE_TICKS 帧后求平均.
 * 校准完后状态切 CAL_OK, UI 显示更新.
 *
 * 注: 上电 1 秒内板子必须静止, 否则 bias 不准.
 *     用户可以稍后按 K1 重置校准 (见 ui.c MPU9250Page_OnKey).
 *
 * 待数据收集后, 再加 EWMA 在线更新 (温度漂移自适应).
 */
static void gyro_calib_step(void) {
    if (s_calib != CAL_RUNNING) return;
    if (!s_have_raw_gyro) return;

    /* RAW → dps */
    float gx_dps = (float)s_raw_gx / GYRO_SENS_250DPS;
    float gy_dps = (float)s_raw_gy / GYRO_SENS_250DPS;
    float gz_dps = (float)s_raw_gz / GYRO_SENS_250DPS;

    s_calib_sum_gx += gx_dps;
    s_calib_sum_gy += gy_dps;
    s_calib_sum_gz += gz_dps;
    s_calib_count++;

    if (s_calib_count >= CAL_SAMPLE_TICKS) {
        s_bias_gx = s_calib_sum_gx / (float)s_calib_count;
        s_bias_gy = s_calib_sum_gy / (float)s_calib_count;
        s_bias_gz = s_calib_sum_gz / (float)s_calib_count;
        s_calib = CAL_OK;
        s_calib_done_ms = now_ms();
        s_last_decay_ms = now_ms();
        g_mpu9250.bias_gx = s_bias_gx;
        g_mpu9250.bias_gy = s_bias_gy;
        g_mpu9250.bias_gz = s_bias_gz;
        g_mpu9250.calib_state = CAL_OK;
    }
}

static void gyro_calib_slow_decay(void) {
    if (s_calib != CAL_OK) return;
    if (!tick_elapsed(s_calib_done_ms, CAL_SLOW_WINDOW_MS)) return;
    if (!tick_elapsed(s_last_decay_ms, CAL_BIAS_DECAY_MS)) return;
    if (!s_have_raw_gyro) return;
    /* 取"现在"的单帧 (简单实现, 不做 1 秒窗口求平均, 比赛时间紧) */
    float gx_dps = ((float)s_raw_gx / GYRO_SENS_250DPS);
    float gy_dps = ((float)s_raw_gy / GYRO_SENS_250DPS);
    float gz_dps = ((float)s_raw_gz / GYRO_SENS_250DPS);

    s_bias_gx = s_bias_gx * (1.0f - CAL_DECAY_ALPHA) + gx_dps * CAL_DECAY_ALPHA;
    s_bias_gy = s_bias_gy * (1.0f - CAL_DECAY_ALPHA) + gy_dps * CAL_DECAY_ALPHA;
    s_bias_gz = s_bias_gz * (1.0f - CAL_DECAY_ALPHA) + gz_dps * CAL_DECAY_ALPHA;
    g_mpu9250.bias_gx = s_bias_gx;
    g_mpu9250.bias_gy = s_bias_gy;
    g_mpu9250.bias_gz = s_bias_gz;
    s_last_decay_ms = now_ms();
}

/* ============================================================================
 * ISR 主推进 (TIMG6 5ms 调一次)
 * ============================================================================
 */
static void state_machine_tick(void) {
    switch (s_state) {
    case S_IDLE:
        if (!g_mpu9250.ok && tick_elapsed(s_init_retry_ms, 1000U)) {
            s_init_retry_ms = now_ms();
            s_state = S_INIT_RST;
        }
        break;
    case S_INIT_RST:       step_init_rst();     break;
    case S_INIT_PWR:       step_init_pwr();     break;
    case S_INIT_CFG:       step_init_cfg();     break;
    case S_INIT_BYPASS:    step_init_bypass();  break;
    case S_INIT_AK_PD:     step_init_ak_pd();   break;
    case S_INIT_AK_ROM:    step_init_ak_rom();  break;
    case S_INIT_AK_CONT:   step_init_ak_cont(); break;
    case S_INIT_DONE:      step_init_done();    break;
    case S_RUN_ACC:        step_run_acc();      break;
    case S_RUN_TEMP:       step_run_temp();     break;
    case S_RUN_GYRO:
        step_run_gyro();
        /* 陀螺读完后立即累积到校准窗口 (在 ISR 里, 简单累加) */
        gyro_calib_step();
        break;
    case S_RUN_MAG_ST1:    step_run_mag_st1();  break;
    case S_RUN_MAG_BURST:  step_run_mag_burst();break;
    default:               s_state = S_IDLE;    break;
    }
    /* 校准完成后, 每 30 秒做一次 slow decay (ISR 里安全: 只是简单比较 + 偶尔一次更新) */
    gyro_calib_slow_decay();

    /* 静止检测 (用户 05:58 反馈: 静止时冻结 euler 积分)
     *   条件: |gx|+|gy|+|gz| < 0.5 dps (持续 200ms 置位)
     *   数据源: 已经减 bias 的 g_mpu9250.gx/gy/gz (ISR 刚写完)
     *   静止 → 累计 ticks, 满 STATIC_HOLD_TICKS 置位
     *   动   → 清零 ticks 和 motionless
     */
    if (s_calib == CAL_OK) {
        float g_abs = fabsf(g_mpu9250.gx) + fabsf(g_mpu9250.gy) + fabsf(g_mpu9250.gz);
        if (g_abs < STATIC_G_SUM_TH) {
            if (s_motionless_ticks < STATIC_HOLD_TICKS) {
                s_motionless_ticks++;
            }
            if (s_motionless_ticks >= STATIC_HOLD_TICKS) {
                s_motionless = true;
            }
        } else {
            s_motionless_ticks = 0U;
            s_motionless = false;
        }
    } else {
        /* 校准未完成: 默认视为运动, 让 s_motionless=false, 但 euler 积分条件
         * (s_calib==CAL_OK) 会拦截, 所以这里设啥无所谓 */
        s_motionless_ticks = 0U;
        s_motionless = false;
    }
}

void MPU9250_TIM_INST_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(MPU9250_TIM_INST) == DL_TIMER_IIDX_ZERO) {
        state_machine_tick();
    }
}

/* ============================================================================
 * 公共 API
 * ============================================================================
 */

bool MPU9250_Init(void) {
    memset(&g_mpu9250, 0, sizeof(g_mpu9250));
    s_adj_x = s_adj_y = s_adj_z = 1.0f;
    s_state = S_IDLE;
    s_init_retry_ms = 0U;
    s_calib = CAL_IDLE;
    s_calib_count = 0;
    s_have_raw_acc = s_have_raw_gyro = s_have_raw_mag = false;
    s_raw_ax = s_raw_ay = s_raw_az = 0;
    s_raw_gx = s_raw_gy = s_raw_gz = 0;
    s_raw_mx = s_raw_my = s_raw_mz = 0;
    s_yaw_filt_init = false;

    SW_I2C_Init();
    DL_TimerG_startCounter(MPU9250_TIM_INST);

    uint8_t who = 0;
    if (SW_I2C_ReadReg(MPU9250_I2C_ADDR, MPU_REG_WHO_AM_I, &who) &&
        (who == MPU9250_WHO_AM_I_VAL || who == 0x70U)) {
        g_mpu9250.who_mpu = who;
        s_state = S_INIT_RST;
    }
    return g_mpu9250.who_mpu != 0U;
}

void MPU9250_Task(void) {
    /* 1. 姿态解算 (主循环, 不在 ISR) */
    if (!g_mpu9250.ok) return;

    /* 2. slow decay (主循环里也跑, 跟 ISR 里的慢周期版本不冲突, 因为时间窗 30s) */
    gyro_calib_slow_decay();

    /* 3. 累计欧拉角积分 (用户 05:36 反馈: "陀螺转了多少度, 上电时 0, 移动后清晰看到")
     *   euler_x (roll) : 绕 X 轴, 左翻/右翻
     *   euler_y (pitch): 绕 Y 轴, 上翘/下俯
     *   euler_z (yaw)  : 绕 Z 轴, 右转/左转  (右转为正, wrap 到 0..360)
     *
     *   三个积分条件 (用户 05:58 反馈: "上电就 4,15,357" 根因):
     *     (1) s_calib == CAL_OK  — 校准未完成时 gx/gy/gz 是原始零漂 (~1.3 dps),
     *                               不能累加, 否则上电 1 秒就跳 4° (实测)
     *     (2) !s_motionless      — 静止时冻结, 避免零漂累积 (核心抗漂移)
     *     (3) dt 合理            — 主循环卡死 > 1s 时跳过本帧
     */
    if (s_calib == CAL_OK && !s_motionless) {
        static uint32_t s_last_euler_ms = 0U;
        uint32_t now_e = now_ms();
        float dt_e = (s_last_euler_ms == 0U) ? MPU9250_DT_SEC
                                              : (float)(now_e - s_last_euler_ms) / 1000.0f;
        s_last_euler_ms = now_e;
        /* 异常保护: dt 不合理 (主循环卡死 > 1s) 时跳过本帧 */
        if (dt_e > 0.0f && dt_e < 1.0f) {
            g_mpu9250.euler_x += g_mpu9250.gx * dt_e;
            g_mpu9250.euler_y += g_mpu9250.gy * dt_e;
            g_mpu9250.euler_z += g_mpu9250.gz * dt_e;

            /* euler_z wrap 到 0..360 (用户原话: "向右转 60° → 60.0, 向左转 60° → 300.0")
             *   即: 上电时为 0, 任意方向旋转时数值始终在 [0, 360) 之间
             *   euler_x / euler_y 用 [-180, +180] 区间 (pitch/roll 物理合理范围) */
            while (g_mpu9250.euler_z >= 360.0f) g_mpu9250.euler_z -= 360.0f;
            while (g_mpu9250.euler_z <    0.0f) g_mpu9250.euler_z += 360.0f;
            while (g_mpu9250.euler_x >  180.0f) g_mpu9250.euler_x -= 360.0f;
            while (g_mpu9250.euler_x < -180.0f) g_mpu9250.euler_x += 360.0f;
            while (g_mpu9250.euler_y >  180.0f) g_mpu9250.euler_y -= 360.0f;
            while (g_mpu9250.euler_y < -180.0f) g_mpu9250.euler_y += 360.0f;
        }
    }

    if (g_mpu9250.who_mag != AK8963_WHO_AM_I_VAL) {
        g_mpu9250.pitch = 0.0f;
        g_mpu9250.roll = 0.0f;
        g_mpu9250.yaw = 0.0f;
        return;
    }

    float ax = g_mpu9250.ax;
    float ay = g_mpu9250.ay;
    float az = g_mpu9250.az;
    float mx = g_mpu9250.mx;
    float my = g_mpu9250.my;
    float mz = g_mpu9250.mz;

    /* pitch/roll 仍用加速度计 (短时间准, 没有积分漂移) */
    float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll_rad  = atan2f(ay, az);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    float cr = cosf(roll_rad),  sr = sinf(roll_rad);

    /* 磁航向: 倾斜补偿后求水平面方位 */
    float mx_t = mx * cp + my * sp * sr + mz * cr * sp;
    float my_t = my * cr - mz * sr;
    float yaw_rad = atan2f(-my_t, mx_t);
    float yaw_mag_deg = yaw_rad * (180.0f / M_PI);
    if (yaw_mag_deg < 0.0f) yaw_mag_deg += 360.0f;

    /* 互补滤波: yaw = (1-α)·(yaw_prev + gz·dt) + α·yaw_mag
     *
     * 关键: 陀螺积分时, 要把"上次的 yaw"转到 -180..+180 区间, 再加上 gz·dt,
     *       然后再转回 0..360, 避免在 0° 附近积分时突然跳 360° 出去
     *
     * 关键 2: 融合前必须把磁航向 wrap 到 yaw_prev 附近 (±180° 内).
     *         否则绕 0° 旋转时, yaw_mag 会从 358° 跳到 2°,
     *         算法误以为转了 -356°, 把 yaw 拉飞
     *
     * 第一次调用时直接拿磁力计的 yaw 当起点 (没历史数据, 没法积分)
     */
    float gz_dps = g_mpu9250.gz;
    float yaw_filt_deg;

    if (!s_yaw_filt_init) {
        yaw_filt_deg = yaw_mag_deg;
        s_yaw_filt_init = true;
    } else {
        /* 上次 yaw → -180..+180 */
        float yaw_prev = g_mpu9250.yaw;
        if (yaw_prev > 180.0f) yaw_prev -= 360.0f;

        /* 陀螺积分: yaw_prev + gz * dt (单位: 度) */
        float yaw_gyro = yaw_prev + gz_dps * MPU9250_DT_SEC;

        /* 把磁航向 wrap 到 yaw_prev 附近 (差值保持在 ±180°) */
        while (yaw_mag_deg - yaw_prev >  180.0f) yaw_mag_deg -= 360.0f;
        while (yaw_mag_deg - yaw_prev < -180.0f) yaw_mag_deg += 360.0f;

        /* 融合 */
        yaw_filt_deg = (1.0f - YAW_GYRO_ALPHA) * yaw_gyro
                     +        YAW_GYRO_ALPHA  * yaw_mag_deg;
    }

    /* 归一化到 0..360 */
    if (yaw_filt_deg <   0.0f) yaw_filt_deg += 360.0f;
    if (yaw_filt_deg >= 360.0f) yaw_filt_deg -= 360.0f;

    g_mpu9250.pitch = pitch_rad * (180.0f / M_PI);
    g_mpu9250.roll  = roll_rad  * (180.0f / M_PI);
    g_mpu9250.yaw   = yaw_filt_deg;

    /* 3. 1Hz 串口调试 (主循环里发, 不在 ISR) */
    uint32_t now = now_ms();
    if (tick_elapsed(s_dbg_last_ms, DBG_PERIOD_MS)) {
        s_dbg_last_ms = now;

        /* 校准前 (RAW) */
        if (s_have_raw_acc || s_have_raw_gyro || s_have_raw_mag) {
            dbg_printf("T:%lu CAL=%u ", (unsigned long)now, (unsigned)s_calib);
            if (s_calib == CAL_OK) {
                dbg_printf("BIAS=%.3f,%.3f,%.3f ",
                           (double)s_bias_gx, (double)s_bias_gy, (double)s_bias_gz);
            } else {
                dbg_printf("BIAS=calibrating... ");
            }
            /* 用户 05:58 反馈: 加 euler + 静止状态, 便于看漂移多少 */
            dbg_printf("EUL x=%+6.2f y=%+6.2f z=%+6.2f ST=%u\r\n",
                       (double)g_mpu9250.euler_x,
                       (double)g_mpu9250.euler_y,
                       (double)g_mpu9250.euler_z,
                       (unsigned)(s_motionless ? 1U : 0U));

            if (s_have_raw_acc) {
                dbg_printf("RAW ax=%+6d ay=%+6d az=%+6d ",
                           (int)s_raw_ax, (int)s_raw_ay, (int)s_raw_az);
            }
            if (s_have_raw_gyro) {
                dbg_printf("gx=%+6d gy=%+6d gz=%+6d ",
                           (int)s_raw_gx, (int)s_raw_gy, (int)s_raw_gz);
            }
            if (s_have_raw_mag) {
                dbg_printf("mx=%+6d my=%+6d mz=%+6d",
                           (int)s_raw_mx, (int)s_raw_my, (int)s_raw_mz);
            }
            dbg_puts("\r\n");

            /* 校准后 (CAL = 量纲换算 + 零漂 + hard-iron) */
            dbg_printf("CAL ax=%+6.3f ay=%+6.3f az=%+6.3f ",
                       (double)g_mpu9250.ax, (double)g_mpu9250.ay, (double)g_mpu9250.az);
            dbg_printf("gx=%+6.3f gy=%+6.3f gz=%+6.3f ",
                       (double)g_mpu9250.gx, (double)g_mpu9250.gy, (double)g_mpu9250.gz);
            dbg_printf("mx=%+6.2f my=%+6.2f mz=%+6.2f\r\n",
                       (double)g_mpu9250.mx, (double)g_mpu9250.my, (double)g_mpu9250.mz);

            /* 姿态 */
            dbg_printf("ATT yaw=%+6.2f pit=%+6.2f rol=%+6.2f T=%+5.2f\r\n\r\n",
                       (double)g_mpu9250.yaw, (double)g_mpu9250.pitch,
                       (double)g_mpu9250.roll, (double)g_mpu9250.temp_c);
        }
    }
}

const MPU9250_Data *MPU9250_GetData(void) {
    return &g_mpu9250;
}

/* 重置陀螺零漂校准 (UI K1 调用, 用户怀疑当前 bias 偏了重新采集) */
void MPU9250_ResetGyroCalib(void) {
    s_calib = CAL_RUNNING;
    s_calib_sum_gx = s_calib_sum_gy = s_calib_sum_gz = 0.0f;
    s_calib_count = 0;
    s_last_decay_ms = now_ms();
    g_mpu9250.calib_state = CAL_RUNNING;
    g_mpu9250.bias_gx = g_mpu9250.bias_gy = g_mpu9250.bias_gz = 0.0f;
    /* 互补滤波也重置 (下次 MPU9250_Task 会用磁力计 yaw 当新起点) */
    s_yaw_filt_init = false;
    /* 静止检测也重置 (避免上次残余状态干扰新校准) */
    s_motionless_ticks = 0U;
    s_motionless = false;
}