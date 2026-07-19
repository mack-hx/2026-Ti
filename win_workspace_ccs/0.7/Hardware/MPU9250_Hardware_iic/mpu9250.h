/* ============================================================================
 * @file    mpu9250.h
 * @brief   MPU-9250 9 轴 IMU 硬件 I2C 驱动, 非阻塞状态机
 *
 *   MPU-9250 = MPU-6500 (加速度 + 陀螺, I2C 地址 0x68) +
 *              AK8963 (磁力计, bypass 后 = 0x0C)
 *
 *   走硬件 I2C0 (PA0=SDA, PA1=SCL, 100kHz),
 *   由 MPU9250_TIM (TIMG6, 5ms) ISR 推进状态机. 主循环 MPU9250_Task()
 *   极轻量, 不阻塞 UI / 按键.
 *
 * ============================================================================
 * 调用方法
 * ============================================================================
 *
 *   上电 (main.c 启动序列里调一次):
 *     MPU9250_Init();              // 启动状态机 + TIMG6 5ms ISR
 *
 *   主循环每帧调 (无操作):
 *     MPU9250_Task();
 *
 *   UI 读取:
 *     const MPU9250_Data *d = MPU9250_GetData();
 *     d->ax / ay / az      // 加速度, g
 *     d->gx / gy / gz      // 陀螺, dps
 *     d->mx / my / mz      // 磁力计, uT
 *     d->yaw / pitch / roll // 倾斜补偿磁航向 + 姿态, deg
 *     d->temp_c            // 内部温度, °C
 *     d->who_mpu / who_mag // WHO_AM_I (0=失败)
 *
 * ============================================================================
 * 失败处理 (用户 2026-07-13 决定: 自适应等待)
 * ============================================================================
 *
 *   - Init 失败 → g_mpu9250.ok=false, 状态机每 1s 自动重试
 *   - AK8963 失败 → who_mag=0, UI 显示 'mag:------'
 *   - 单次 read 失败 → 该帧保持上次值, 下一帧继续
 *
 * ============================================================================
 */
#ifndef __MPU9250_H__
#define __MPU9250_H__

#include <stdbool.h>
#include <stdint.h>

/* I2C 地址 (7-bit) */
#define MPU9250_I2C_ADDR        0x68U
#define AK8963_I2C_ADDR         0x0CU

/* 主要寄存器 */
#define MPU_REG_WHO_AM_I        0x75U
#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_PWR_MGMT_2      0x6CU
#define MPU_REG_INT_PIN_CFG     0x37U
#define MPU_REG_USER_CTRL       0x6AU
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_ACCEL_CONFIG    0x1CU
#define MPU_REG_GYRO_CONFIG     0x23U
#define MPU_REG_ACCEL_XOUT_H    0x3BU
#define MPU_REG_GYRO_XOUT_H     0x43U
#define MPU_REG_TEMP_OUT_H      0x41U

#define AK_REG_WIA              0x00U
#define AK_REG_ST1              0x02U
#define AK_REG_HXL              0x03U
#define AK_REG_ST2              0x09U
#define AK_REG_CNTL1            0x0AU
#define AK_REG_ASAX             0x10U

/* 期望的 WHO_AM_I 答复 */
#define MPU9250_WHO_AM_I_VAL    0x71U
#define AK8963_WHO_AM_I_VAL     0x48U

/* 9 轴 + 倾角 + 温度 + 校准状态 */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float yaw, pitch, roll;
    float temp_c;
    /* 陀螺零漂校准 (单位 dps, 应用到 gx/gy/gz) */
    float bias_gx, bias_gy, bias_gz;
    /* 校准状态: 0=未校准/校准中 (UI 显示 "cal..."), 1=校准完成 */
    uint8_t calib_state;
    uint8_t who_mpu;
    uint8_t who_mag;
    bool ok;
    /* 累计欧拉角 (用户 05:36 反馈: "陀螺转了多少度, 上电时 0, 移动后能清晰看到动了多少")
     *   euler_x (roll)  : 绕 X 轴累计转角 (左右翻)   [-180..+180] 重置 0
     *   euler_y (pitch) : 绕 Y 轴累计转角 (上下翘)   [-180..+180]
     *   euler_z (yaw)   : 绕 Z 轴累计转角 (左右转)   [   0..+360] 右转为正
     *   注意: 这是纯陀螺积分, **不抗漂**, 长时间会偏, 但用户要求"清晰看到移动量"
     *   与上方 yaw/pitch/roll (绝对姿态, 已磁力校准) 不同: 这是"移动量" */
    float euler_x, euler_y, euler_z;
} MPU9250_Data;

extern MPU9250_Data g_mpu9250;

/* ============================================================================
 * API
 * ============================================================================ */

bool MPU9250_Init(void);
void MPU9250_Task(void);
const MPU9250_Data *MPU9250_GetData(void);

/* 重置陀螺零漂校准: 重新进入 CAL_RUNNING, 累积新 bias
 *   用户在 K4 短按触发 (UI 层). 用于: 板子没静止 / 温度漂移大 / 用户怀疑当前 bias 偏了 */
void MPU9250_ResetGyroCalib(void);

#endif /* __MPU9250_H__ */
