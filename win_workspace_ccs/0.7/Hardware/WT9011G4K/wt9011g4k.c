/* ============================================================================
 * @file    wt9011g4k.c
 * @brief   WT9011G4K (JY-901) 九轴 I2C 驱动 — 硬件 I2C0 (PA0/PA1), 轮询
 *
 * ============================================================================
 * I2C 协议
 * ============================================================================
 *
 *   写入: START → SLA+W → RegAddr → DataL → DataH → STOP
 *   读取: START → SLA+W → RegAddr → RESTART → SLA+R → DataL DataH ... → STOP
 *
 *   - 每个寄存器 16-bit, 低字节在前 (DataL, DataH)
 *   - 连续读时寄存器地址自增
 *   - 传感器数据从 0x34 开始连续排列, 一次 burst 读 26 字节
 *
 *   量纲:
 *     AX [g]   = AX_raw / 32768 × 16
 *     GX [°/s] = GX_raw / 32768 × 2000
 *     Roll [°] = Roll_raw / 32768 × 180
 *     TEMP [°C]= TEMP_raw / 100
 *
 * ============================================================================
 * 注意事项 (来自手册)
 * ============================================================================
 *
 *   1. 嵌入电路板, I2C 短距离使用 (建议 < 10cm)
 *   2. 一问一答方式, 不主动输出
 *   3. SDA/SCL 必须上拉 4.7k~10k 到 VCC
 *   4. 支持多设备: 不同 IICADDR
 *   5. 写配置前需先解锁 (KEY = 0xB588)
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/System/clock.h"
#include "Hardware/WT9011G4K/wt9011g4k.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ============================================================================
 * 全局数据
 * ============================================================================ */
WT9011G4K_Data g_wt9011g4k = { .ok = false };

/* 连续读取缓冲区: 0x34(AX) ~ 0x40(TEMP) = 26 bytes */
#define SENSOR_BURST_LEN    26U
static uint8_t s_burst_buf[SENSOR_BURST_LEN];

#define I2C_TIMEOUT_MS      10U

/* ============================================================================
 * 硬件 I2C 底层 (使用 SysConfig 生成的 I2C_WT9011G4K 实例)
 * ============================================================================
 *
 * SysConfig 配置 I2C0 在 PA0(SDA)/PA1(SCL), 400kHz Fast Mode.
 * 生成的宏:
 *   I2C_Gyro_INST              — I2C 外设实例
 *   SYSCFG_DL_I2C_Gyro_init()  — SysConfig 初始化函数
 *
 * 时序: 写 reg 地址后加 delay 等寄存器指针 latch (与 MPU9250 同理).
 * ============================================================================ */

/* 等待 I2C 总线空闲 */
static bool i2c_wait_idle(void) {
    uint32_t t0 = (uint32_t)tick_ms;
    while (DL_I2C_getControllerStatus(I2C_Gyro_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) return false;
    }
    return true;
}

/* 写 n 字节: START + SLA+W + RegAddr + n bytes + STOP */
static bool i2c_write_burst(uint8_t slave_7bit, uint8_t reg,
                            const uint8_t *data, uint8_t len) {
    if (!i2c_wait_idle()) return false;

    /* 先发 reg 地址 */
    DL_I2C_transmitControllerData(I2C_Gyro_INST, reg);
    DL_I2C_clearInterruptStatus(I2C_Gyro_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    while (!(DL_I2C_getControllerStatus(I2C_Gyro_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* 启动传输: SLA+W + (len) bytes data */
    DL_I2C_startControllerTransfer(I2C_Gyro_INST, slave_7bit,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, len);

    uint32_t t0 = (uint32_t)tick_ms;
    uint8_t cnt = len;
    const uint8_t *ptr = data;

    while (cnt > 0) {
        uint8_t filled = DL_I2C_fillControllerTXFIFO(
            I2C_Gyro_INST, ptr, cnt);
        cnt -= filled;
        ptr += filled;

        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) return false;

        if (DL_I2C_getRawInterruptStatus(I2C_Gyro_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
            break;
        }
    }

    return true;
}

/* 读 n 字节: START + SLA+W + RegAddr + RESTART + SLA+R + n bytes + STOP */
static bool i2c_read_burst(uint8_t slave_7bit, uint8_t reg,
                           uint8_t *data, uint8_t len) {
    if (!i2c_wait_idle()) return false;

    /* 先发 reg 地址 */
    DL_I2C_transmitControllerData(I2C_Gyro_INST, reg);
    I2C_Gyro_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(I2C_Gyro_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    while (!(DL_I2C_getControllerStatus(I2C_Gyro_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* 启动接收 */
    DL_I2C_startControllerTransfer(I2C_Gyro_INST, slave_7bit,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, len);

    uint32_t t0 = (uint32_t)tick_ms;
    uint8_t idx = 0;

    while (!DL_I2C_getRawInterruptStatus(I2C_Gyro_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE)) {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_Gyro_INST)) {
            if (idx < len) {
                data[idx++] = DL_I2C_receiveControllerData(I2C_Gyro_INST);
            }
        }
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) return false;
    }

    /* 读取 FIFO 中剩余数据 */
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_Gyro_INST)) {
        if (idx < len) {
            data[idx++] = DL_I2C_receiveControllerData(I2C_Gyro_INST);
        }
    }

    I2C_Gyro_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_Gyro_INST);

    return (idx == len);
}

/* ============================================================================
 * 16-bit 寄存器读写 (低字节在前)
 * ============================================================================ */

bool WT9011G4K_ReadReg16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];
    if (!i2c_read_burst(WT9011G4K_I2C_ADDR, reg, buf, 2)) return false;
    *val = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return true;
}

static bool wt_write_reg_raw(uint8_t reg, uint16_t val) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)(val >> 8);
    return i2c_write_burst(WT9011G4K_I2C_ADDR, reg, buf, 2);
}

static bool wt_unlock(void) {
    return wt_write_reg_raw(WT_REG_KEY, WT_KEY_UNLOCK);
}

bool WT9011G4K_WriteReg(uint8_t reg, uint16_t val) {
    if (!wt_unlock()) return false;
    return wt_write_reg_raw(reg, val);
}

/* ============================================================================
 * 高层 API
 * ============================================================================ */

bool WT9011G4K_Init(void) {
    uint16_t ver = 0;
    if (!WT9011G4K_ReadReg16(WT_REG_VERSION, &ver)) {
        g_wt9011g4k.ok = false;
        return false;
    }
    g_wt9011g4k.version = ver;
    g_wt9011g4k.ok = true;
    return true;
}

void WT9011G4K_Task(void) {
    if (!g_wt9011g4k.ok) {
        WT9011G4K_Init();
        return;
    }

    /* 连续读 0x34(AX) ~ 0x40(TEMP) = 26 字节 */
    if (!i2c_read_burst(WT9011G4K_I2C_ADDR, WT_REG_AX,
                        s_burst_buf, SENSOR_BURST_LEN)) {
        g_wt9011g4k.ok = false;
        return;
    }

    /* --- 加速度 [g] --- */
    int16_t raw_ax = (int16_t)((uint16_t)s_burst_buf[ 1] << 8 | s_burst_buf[ 0]);
    int16_t raw_ay = (int16_t)((uint16_t)s_burst_buf[ 3] << 8 | s_burst_buf[ 2]);
    int16_t raw_az = (int16_t)((uint16_t)s_burst_buf[ 5] << 8 | s_burst_buf[ 4]);
    g_wt9011g4k.ax = (float)raw_ax * WT_ACC_SCALE;
    g_wt9011g4k.ay = (float)raw_ay * WT_ACC_SCALE;
    g_wt9011g4k.az = (float)raw_az * WT_ACC_SCALE;

    /* --- 角速度 [°/s] --- */
    int16_t raw_gx = (int16_t)((uint16_t)s_burst_buf[ 7] << 8 | s_burst_buf[ 6]);
    int16_t raw_gy = (int16_t)((uint16_t)s_burst_buf[ 9] << 8 | s_burst_buf[ 8]);
    int16_t raw_gz = (int16_t)((uint16_t)s_burst_buf[11] << 8 | s_burst_buf[10]);
    g_wt9011g4k.gx = (float)raw_gx * WT_GYRO_SCALE;
    g_wt9011g4k.gy = (float)raw_gy * WT_GYRO_SCALE;
    g_wt9011g4k.gz = (float)raw_gz * WT_GYRO_SCALE;

    /* --- 磁场 [LSB] --- */
    g_wt9011g4k.mx = (int16_t)((uint16_t)s_burst_buf[13] << 8 | s_burst_buf[12]);
    g_wt9011g4k.my = (int16_t)((uint16_t)s_burst_buf[15] << 8 | s_burst_buf[14]);
    g_wt9011g4k.mz = (int16_t)((uint16_t)s_burst_buf[17] << 8 | s_burst_buf[16]);

    /* --- 姿态角 [°] (模块内部 Kalman 融合) --- */
    int16_t raw_roll  = (int16_t)((uint16_t)s_burst_buf[19] << 8 | s_burst_buf[18]);
    int16_t raw_pitch = (int16_t)((uint16_t)s_burst_buf[21] << 8 | s_burst_buf[20]);
    int16_t raw_yaw   = (int16_t)((uint16_t)s_burst_buf[23] << 8 | s_burst_buf[22]);
    g_wt9011g4k.roll  = (float)raw_roll  * WT_ANGLE_SCALE;
    g_wt9011g4k.pitch = (float)raw_pitch * WT_ANGLE_SCALE;
    g_wt9011g4k.yaw   = (float)raw_yaw   * WT_ANGLE_SCALE;

    /* --- 温度 [°C] --- */
    int16_t raw_temp = (int16_t)((uint16_t)s_burst_buf[25] << 8 | s_burst_buf[24]);
    g_wt9011g4k.temp_c = (float)raw_temp * WT_TEMP_SCALE;
}

const WT9011G4K_Data *WT9011G4K_GetData(void) {
    return &g_wt9011g4k;
}

bool WT9011G4K_Save(void) {
    return WT9011G4K_WriteReg(WT_REG_SAVE, WT_SAVE_SAVE);
}

bool WT9011G4K_Reset(void) {
    return WT9011G4K_WriteReg(WT_REG_SAVE, WT_SAVE_RESET);
}

bool WT9011G4K_Restore(void) {
    return WT9011G4K_WriteReg(WT_REG_SAVE, WT_SAVE_RESTORE);
}