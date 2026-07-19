/* ============================================================================
 * @file    wt9011g4k.h
 * @brief   WT9011G4K (JY-901) 九轴 IMU I2C 驱动 — 硬件 I2C
 *
 *   WT9011G4K 含三轴加速度计、三轴陀螺仪、三轴磁力计, 内置传感器融合 (Kalman),
 *   I2C 读取即用角度.
 *   走硬件 I2C0 (PA0=SDA, PA1=SCL, 400kHz Fast Mode).
 *   SysConfig 配置 I2C0, 轮询方式, 主循环调 WT9011G4K_Task() 即可.
 *
 *   I2C 协议要点:
 *     - 7-bit 从机地址默认 0x50
 *     - 每个寄存器 16-bit, **低字节在前**, 高字节在后
 *     - 支持连续读: START + SLA+W + RegAddr + RESTART + SLA+R + N bytes
 *     - 寄存器地址自增
 *
 * ============================================================================
 * 调用方法
 * ============================================================================
 *
 *   上电:
 *     WT9011G4K_Init();
 *
 *   主循环:
 *     WT9011G4K_Task();
 *
 *   读取:
 *     const WT9011G4K_Data *d = WT9011G4K_GetData();
 *     d->ax / ay / az        加速度 [g]
 *     d->gx / gy / gz        角速度 [°/s]
 *     d->roll / pitch / yaw  姿态角 [°]
 *     d->temp_c              温度 [°C]
 *     d->ok                  通信是否正常
 * ============================================================================
 */
#ifndef __WT9011G4K_H__
#define __WT9011G4K_H__

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * I2C 从机地址 (7-bit)
 * ============================================================================ */
#define WT9011G4K_I2C_ADDR          0x50U

/* ============================================================================
 * 寄存器地址
 * ============================================================================ */

/* 系统控制 */
#define WT_REG_SAVE             0x00U   /* 保存/重启/恢复出厂 */
#define WT_REG_CALSW            0x01U   /* 校准模式 */
#define WT_REG_RSW              0x02U   /* 输出内容 (串口用) */
#define WT_REG_RRATE            0x03U   /* 输出速率 (串口用) */
#define WT_REG_BAUD             0x04U   /* 串口波特率 */

/* 零偏校准 */
#define WT_REG_AXOFFSET         0x05U
#define WT_REG_AYOFFSET         0x06U
#define WT_REG_AZOFFSET         0x07U
#define WT_REG_GXOFFSET         0x08U
#define WT_REG_GYOFFSET         0x09U
#define WT_REG_GZOFFSET         0x0AU
#define WT_REG_HXOFFSET         0x0BU
#define WT_REG_HYOFFSET         0x0CU
#define WT_REG_HZOFFSET         0x0DU

/* 端口模式 */
#define WT_REG_D0MODE           0x0EU
#define WT_REG_D1MODE           0x0FU
#define WT_REG_D2MODE           0x10U
#define WT_REG_D3MODE           0x11U

/* 设备配置 */
#define WT_REG_IICADDR          0x1AU   /* I2C 地址 */
#define WT_REG_LEDOFF           0x1BU   /* 关闭 LED */
#define WT_REG_BANDWIDTH        0x1FU   /* 带宽 */
#define WT_REG_GYRORANGE        0x20U   /* 陀螺仪量程 */
#define WT_REG_ACCRANGE         0x21U   /* 加速度量程 */
#define WT_REG_SLEEP            0x22U   /* 休眠 */
#define WT_REG_ORIENT           0x23U   /* 安装方向 */
#define WT_REG_AXIS6            0x24U   /* 6/9 轴算法选择 */
#define WT_REG_FILTK            0x25U   /* K 值滤波 */
#define WT_REG_READADDR         0x27U   /* 串口读取起始地址 */
#define WT_REG_ACCFILT          0x2AU   /* 加速度滤波 */

/* 版本 */
#define WT_REG_VERSION          0x2EU   /* 版本号 (只读) */

/* 时间 */
#define WT_REG_YYMM             0x30U
#define WT_REG_DDHH             0x31U
#define WT_REG_MMSS             0x32U
#define WT_REG_MS               0x33U

/* ======== 传感器数据 (只读, 连续读取最高效) ======== */
#define WT_REG_AX               0x34U   /* 加速度 X 起始 (2 bytes) */
#define WT_REG_AY               0x35U
#define WT_REG_AZ               0x36U
#define WT_REG_GX               0x37U   /* 角速度 X 起始 (2 bytes) */
#define WT_REG_GY               0x38U
#define WT_REG_GZ               0x39U
#define WT_REG_HX               0x3AH   /* 磁场 X 起始 (2 bytes) */
#define WT_REG_HY               0x3BH
#define WT_REG_HZ               0x3CH
#define WT_REG_ROLL             0x3DH   /* 滚转角 起始 (2 bytes) */
#define WT_REG_PITCH            0x3EH
#define WT_REG_YAW              0x3FH
#define WT_REG_TEMP             0x40U   /* 温度 (2 bytes) */

/* 四元数 */
#define WT_REG_Q0               0x51U
#define WT_REG_Q1               0x52U
#define WT_REG_Q2               0x53U
#define WT_REG_Q3               0x54U

/* 写保护 */
#define WT_REG_KEY              0x69U   /* 解锁寄存器 */
#define WT_KEY_UNLOCK            0xB588U /* 解锁值 */

/* 陀螺仪校准 */
#define WT_REG_GYROCALITHR      0x61U
#define WT_REG_GYROCALTIME      0x63U

/* 设备编号 */
#define WT_REG_NUMBERID1        0x7FU

/* ============================================================================
 * SAVE 寄存器值
 * ============================================================================ */
#define WT_SAVE_SAVE            0x0000U
#define WT_SAVE_RESET           0x00FFU
#define WT_SAVE_RESTORE         0x0001U

/* ============================================================================
 * CALSW 校准模式
 * ============================================================================ */
#define WT_CAL_NORMAL           0x0000U
#define WT_CAL_ACCEL            0x0001U   /* 自动加计校准 */
#define WT_CAL_HEIGHT_ZERO      0x0003U   /* 高度清零 */
#define WT_CAL_YAW_ZERO         0x0004U   /* 航向角置零 */
#define WT_CAL_MAG_SPHERE       0x0007U   /* 磁场校准 (球型) */
#define WT_CAL_SET_REF          0x0008U   /* 设置角度参考 */
#define WT_CAL_MAG_DUAL         0x0009U   /* 磁场校准 (双平面) */

/* ============================================================================
 * RRATE 输出速率
 * ============================================================================ */
#define WT_RRATE_0_2HZ          0x0001U
#define WT_RRATE_0_5HZ          0x0002U
#define WT_RRATE_1HZ            0x0003U
#define WT_RRATE_2HZ            0x0004U
#define WT_RRATE_5HZ            0x0005U
#define WT_RRATE_10HZ           0x0006U
#define WT_RRATE_20HZ           0x0007U
#define WT_RRATE_50HZ           0x0008U
#define WT_RRATE_100HZ          0x0009U
#define WT_RRATE_200HZ          0x000BU

/* ============================================================================
 * 量纲换算常量
 * ============================================================================ */
#define WT_ACC_SCALE            (16.0f / 32768.0f)     /* ±16g */
#define WT_GYRO_SCALE           (2000.0f / 32768.0f)   /* ±2000°/s */
#define WT_ANGLE_SCALE          (180.0f / 32768.0f)    /* ±180° */
#define WT_TEMP_SCALE           (1.0f / 100.0f)        /* °C */

/* ============================================================================
 * 数据结构
 * ============================================================================ */
typedef struct {
    /* 加速度 [g] */
    float ax, ay, az;
    /* 角速度 [°/s] */
    float gx, gy, gz;
    /* 磁场 [LSB] */
    int16_t mx, my, mz;
    /* 姿态角 [°] (模块内部融合) */
    float roll, pitch, yaw;
    /* 温度 [°C] */
    float temp_c;
    /* 版本号 */
    uint16_t version;
    /* 通信状态 */
    bool ok;
} WT9011G4K_Data;

extern WT9011G4K_Data g_wt9011g4k;

/* ============================================================================
 * API
 * ============================================================================ */

/* 初始化: 探测 I2C 地址, 读版本号 */
bool WT9011G4K_Init(void);

/* 主循环调用: 读取全部传感器数据并转换 */
void WT9011G4K_Task(void);

/* 获取数据指针 */
const WT9011G4K_Data *WT9011G4K_GetData(void);

/* 写寄存器 (需先解锁): WT9011G4K_WriteReg(addr, value) */
bool WT9011G4K_WriteReg(uint8_t reg, uint16_t val);

/* 读单个 16-bit 寄存器: 低字节在前 */
bool WT9011G4K_ReadReg16(uint8_t reg, uint16_t *val);

/* 保存配置到 Flash */
bool WT9011G4K_Save(void);

/* 软复位 */
bool WT9011G4K_Reset(void);

/* 恢复出厂设置 */
bool WT9011G4K_Restore(void);

#endif /* __WT9011G4K_H__ */
