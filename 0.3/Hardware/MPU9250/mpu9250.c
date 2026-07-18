/* ============================================================================
 * @file    mpu9250.c
 * @brief   MPU-9250 9 轴 I2C 驱动 (软件 bit-bang) - 非阻塞状态机 (2026-07-14 重构)
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
 * ============================================================================
 * 非阻塞架构 (用户 2026-07-14 02:55 反馈 "按键反应慢了")
 * ============================================================================
 *
 *   旧版 (v015): MPU9250_Task() 在主循环同步跑, 一次卡 3~5ms (读 9 轴),
 *                Init 失败重试时一次卡 500ms (5 次 retry × delay 5/10/30ms + read).
 *                UI/按键全卡.
 *
 *   新版: 状态机推进, 由 MPU9250_TIM (TIMG6, 5ms ISR) 唤醒:
 *
 *     主循环每帧只调 MPU9250_Task(): 1ms 以内检查 ready 标志, 立即返回.
 *     实际 I2C 事务在 MPU9250_TIM ISR 里推进 (每个 tick 推进 1 个状态).
 *     一个 I2C 事务 ~500us, 一次采集 9 轴要 4 个 tick = 20ms.
 *     UI / 按键 / 编码器 50us ISR 完全不被 MPU 阻塞.
 *
 *   状态机:
 *     S_IDLE          → 检查 g_ok, 未 ok 跑 init 序列
 *     S_INIT_PWR      → 写 PWR_MGMT_1=0x01 (1 个 tick)
 *     S_INIT_BYPASS   → 写 INT_PIN_CFG |= 0x42 + USER_CTRL=0 (1 个 tick)
 *     S_INIT_AK_PD    → 写 AK_CNTL1=0x00 (1 个 tick)
 *     S_INIT_AK_ROM   → 写 AK_CNTL1=0x0F + read ASAX/Y/Z (1 个 tick)
 *     S_INIT_AK_CONT  → 写 AK_CNTL1=0x12 + read WIA (1 个 tick)
 *     S_RUN_ACC       → 读 0x3B (6 字节加速度) + 解析
 *     S_RUN_TEMP      → 读 0x41 (2 字节温度)
 *     S_RUN_GYRO      → 读 0x43 (6 字节陀螺) + 解析
 *     S_RUN_MAG       → 读 AK ST1 + 0x03 6 字节 + ST2 + 解析
 *
 * ============================================================================
 * 调用方法
 * ============================================================================
 *
 *   上电 (main.c 启动序列里调一次):
 *     MPU9250_Init();              // 初始化 PA0/PA1 PINCM + 启动状态机 (ISR 推进)
 *
 *   主循环每帧调 (极轻量, 仅检查 ready 标志):
 *     MPU9250_Task();              // no-op, 状态机由 TIMG6 ISR 推进
 *
 *   UI 读取 (按需):
 *     const MPU9250_Data *d = MPU9250_GetData();
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "sw_i2c.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
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
 * 全局数据 (UI 读这个)
 * ============================================================================ */
MPU9250_Data g_mpu9250 = {
    .who_mpu = 0, .who_mag = 0, .ok = false,
};

/* 工厂校准 (AK8963 ASAX/Y/Z 归一化系数) */
static float s_adj_x = 1.0f, s_adj_y = 1.0f, s_adj_z = 1.0f;

/* ============================================================================
 * 状态机
 * ============================================================================ */
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

/* 多步骤需要跨 tick 等待 (MPU reset, bypass 稳定), 用此全局 */
static volatile uint32_t s_wait_until_ms = 0;

/* I2C 事务 scratch buffer (避免栈占用) */
static uint8_t s_buf[6];

/* ============================================================================
 * 工具函数
 * ============================================================================ */

/* SysTick 维护的 1ms 时基 (clock.c 暴露) */
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
    return ((int32_t)(now_ms() - s_wait_until_ms) >= 0);
}

/* ============================================================================
 * Init 步骤 (每个状态做 1 个 I2C 事务, 跨 tick 用 wait)
 * ============================================================================ */

static void step_init_rst(void) {
    /* 软复位: 写 PWR_MGMT_1=0x80 (DEVICE_RESET), 等 100ms, 然后 PWR_MGMT_1=0x00 唤醒 */
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x80U);
    set_wait_ms(100U);
    s_state = S_INIT_PWR;
}

static void step_init_pwr(void) {
    if (!wait_done()) return;  /* 等 100ms */
    /* reset 后默认 SLEEP=1, 必须先写 0x00 唤醒 */
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x01U);
    s_state = S_INIT_CFG;
}

static void step_init_cfg(void) {
    /* 基础配置 (任意失败都继续, 设备寄存器访问就 OK) */
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_SMPLRT_DIV, 0x07U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_CONFIG, 0x06U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_PWR_MGMT_2, 0x00U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_ACCEL_CONFIG, 0x00U);
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_GYRO_CONFIG, 0x00U);
    s_state = S_INIT_BYPASS;
}

static void step_init_bypass(void) {
    /* INT_PIN_CFG |= 0x42 (BYPASS_EN + INT_LEVEL): bypass 让 AK8963 直通 */
    uint8_t int_pin = 0;
    (void)SW_I2C_ReadReg(MPU9250_I2C_ADDR, MPU_REG_INT_PIN_CFG, &int_pin);
    int_pin |= 0x42U;
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_INT_PIN_CFG, int_pin);
    /* USER_CTRL=0: 关 I2C_MST, 把 aux bus 交给 bypass */
    (void)SW_I2C_WriteReg(MPU9250_I2C_ADDR, MPU_REG_USER_CTRL, 0x00U);
    /* bypass 稳定时间 (datasheet: bypass 开启后需 ~5ms) */
    set_wait_ms(10U);
    s_state = S_INIT_AK_PD;
}

static void step_init_ak_pd(void) {
    if (!wait_done()) return;  /* 等 10ms bypass 稳定 */
    /* AK8963 power-down (默认就是 power-down, 但先确保) */
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x00U);
    set_wait_ms(5U);
    s_state = S_INIT_AK_ROM;
}

static void step_init_ak_rom(void) {
    if (!wait_done()) return;  /* 等 5ms */
    /* FUSE ROM access: 读 ASAX/Y/Z (工厂校准值) */
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x0FU);
    set_wait_ms(5U);
    s_state = S_INIT_AK_CONT;
}

static void step_init_ak_cont(void) {
    if (!wait_done()) return;  /* 等 5ms */
    /* 读 ASAX/Y/Z */
    uint8_t asa[3] = { 0, 0, 0 };
    if (SW_I2C_ReadRegs(AK8963_I2C_ADDR, AK_REG_ASAX, asa, 3u)) {
        s_adj_x = ((float)(int16_t)asa[0] - 128.0f) / 256.0f + 1.0f;
        s_adj_y = ((float)(int16_t)asa[1] - 128.0f) / 256.0f + 1.0f;
        s_adj_z = ((float)(int16_t)asa[2] - 128.0f) / 256.0f + 1.0f;
        if (s_adj_x <= 0.0f) s_adj_x = 1.0f;
        if (s_adj_y <= 0.0f) s_adj_y = 1.0f;
        if (s_adj_z <= 0.0f) s_adj_z = 1.0f;
    }
    /* 切连续测量模式 100Hz 16-bit */
    (void)SW_I2C_WriteReg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x12U);
    set_wait_ms(5U);
    s_state = S_INIT_DONE;
}

static void step_init_done(void) {
    if (!wait_done()) return;  /* 等 5ms */
    /* 读 WIA 确认 AK8963 真在 */
    uint8_t who = 0;
    if (SW_I2C_ReadReg(AK8963_I2C_ADDR, AK_REG_WIA, &who)) {
        g_mpu9250.who_mag = who;
    }
    g_mpu9250.ok = true;
    s_state = S_RUN_ACC;
}

/* ============================================================================
 * 读 9 轴 (每个状态读 1 组)
 * ============================================================================ */

static void step_run_acc(void) {
    if (!SW_I2C_ReadRegs(MPU9250_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, s_buf, 6u)) {
        s_state = S_IDLE;
        return;
    }
    int16_t ax = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
    int16_t ay = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
    int16_t az = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);
    g_mpu9250.ax = (float)ax / ACC_SENS_2G;
    g_mpu9250.ay = (float)ay / ACC_SENS_2G;
    g_mpu9250.az = (float)az / ACC_SENS_2G;
    s_state = S_RUN_TEMP;
}

static void step_run_temp(void) {
    if (!SW_I2C_ReadRegs(MPU9250_I2C_ADDR, MPU_REG_TEMP_OUT_H, s_buf, 2u)) {
        s_state = S_RUN_GYRO;  /* temp 失败不影响后续 */
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
    int16_t gx = (int16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]);
    int16_t gy = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
    int16_t gz = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);
    g_mpu9250.gx = (float)gx / GYRO_SENS_250DPS;
    g_mpu9250.gy = (float)gy / GYRO_SENS_250DPS;
    g_mpu9250.gz = (float)gz / GYRO_SENS_250DPS;
    s_state = S_RUN_MAG_ST1;
}

static void step_run_mag_st1(void) {
    /* 磁力计不可用 → 直接回 S_RUN_ACC 循环 */
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
        /* DRDY=0: 本次跳过, 回 S_RUN_ACC 等下一帧 */
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
    /* 必须读 ST2 才能解锁下一帧 (datasheet) */
    (void)SW_I2C_ReadReg(AK8963_I2C_ADDR, AK_REG_ST2, s_buf);
    int16_t mx_raw = (int16_t)(((uint16_t)s_buf[1] << 8) | s_buf[0]);  /* LE */
    int16_t my_raw = (int16_t)(((uint16_t)s_buf[3] << 8) | s_buf[2]);
    int16_t mz_raw = (int16_t)(((uint16_t)s_buf[5] << 8) | s_buf[4]);
    /* 原始 uT (no 上面的浮点数学, 留给 MPU9250_Task 算 yaw/pitch/roll) */
    g_mpu9250.mx = (float)mx_raw * s_adj_x * MAG_RAW_TO_uT_16BIT;
    g_mpu9250.my = (float)my_raw * s_adj_y * MAG_RAW_TO_uT_16BIT;
    g_mpu9250.mz = (float)mz_raw * s_adj_z * MAG_RAW_TO_uT_16BIT;

    s_state = S_RUN_ACC;  /* 循环 */
}

/* ============================================================================
 * ISR 主推进 (TIMG6 5ms 调一次)
 * ============================================================================ */
static void state_machine_tick(void) {
    switch (s_state) {
    case S_IDLE:
        /* 未 ok + 距上次重试 ≥ 1000ms → 重新跑 init 序列 */
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
    case S_RUN_GYRO:       step_run_gyro();     break;
    case S_RUN_MAG_ST1:    step_run_mag_st1();  break;
    case S_RUN_MAG_BURST:  step_run_mag_burst();break;
    default:               s_state = S_IDLE;    break;
    }
}

/* ============================================================================
 * ISR 入口 (TIMG6 5ms)
 * ============================================================================ */
void MPU9250_TIM_INST_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(MPU9250_TIM_INST) == DL_TIMER_IIDX_ZERO) {
        state_machine_tick();
    }
}

/* ============================================================================
 * 公共 API
 * ============================================================================ */

bool MPU9250_Init(void) {
    /* 清状态 + 启动 MPU9250_TIM */
    memset(&g_mpu9250, 0, sizeof(g_mpu9250));
    s_adj_x = s_adj_y = s_adj_z = 1.0f;
    s_state = S_IDLE;
    s_init_retry_ms = 0U;

    SW_I2C_Init();

    /* SysConfig 默认 startTimer=false (DL_TIMER_STOP), 状态机 ISR 永远不触发,
     * 主循环 MPU9250_Task() 是 no-op, 整个 I2C 就废了.
     * 这里手动 start counter (用户 2026-07-14 03:18 反馈 "9250 又没了"). */
    DL_TimerG_startCounter(MPU9250_TIM_INST);

    /* 上电后先读一次 WHO_AM_I (主循环里 MPU9250_Init 之前没跑过状态机,
     * 这里直接读; 后续状态机不会重读 WHO) */
    uint8_t who = 0;
    if (SW_I2C_ReadReg(MPU9250_I2C_ADDR, MPU_REG_WHO_AM_I, &who) &&
        (who == MPU9250_WHO_AM_I_VAL || who == 0x70U)) {
        g_mpu9250.who_mpu = who;
        /* 找到 MPU 后立即推进 reset 状态, 不再等 ISR tick */
        s_state = S_INIT_RST;
    }
    return g_mpu9250.who_mpu != 0U;
}

void MPU9250_Task(void) {
    /* 非阻塞: 状态机由 TIMG6 ISR 推进. 这里做 UI 用到的姿态解算 (atan2f/cosf/sinf),
     * 不能放 ISR 里 (MSPM0 无 FPU, 软实现不 ISR-safe + 堆栈吃紧). */
    if (!g_mpu9250.ok) return;
    if (g_mpu9250.who_mag != AK8963_WHO_AM_I_VAL) return;  /* 无 mag 算不出 yaw */

    float ax = g_mpu9250.ax;
    float ay = g_mpu9250.ay;
    float az = g_mpu9250.az;
    float mx = g_mpu9250.mx;
    float my = g_mpu9250.my;
    float mz = g_mpu9250.mz;

    float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll_rad  = atan2f(ay, az);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    float cr = cosf(roll_rad),  sr = sinf(roll_rad);
    float mx_t = mx * cp + my * sp * sr + mz * cr * sp;
    float my_t = my * cr - mz * sr;
    float yaw_rad = atan2f(-my_t, mx_t);
    float yaw_deg = yaw_rad * (180.0f / M_PI);
    if (yaw_deg < 0.0f)    yaw_deg += 360.0f;
    if (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
    g_mpu9250.pitch = pitch_rad * (180.0f / M_PI);
    g_mpu9250.roll  = roll_rad  * (180.0f / M_PI);
    g_mpu9250.yaw   = yaw_deg;
}

const MPU9250_Data *MPU9250_GetData(void) {
    return &g_mpu9250;
}
