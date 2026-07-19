/* ============================================================================
 * @file    hwi2c.c
 * @brief   硬件 I2C 驱动 — MPU9250 专用, 使用 I2C0 (I2C_Gyro_INST)
 *
 *   内部调用 MSPM0 DL_I2C HAL 操作硬件 I2C0.
 *   I2C0 由 SysConfig 配置在 PA0(SDA)/PA1(SCL).
 *
 *   参考 mspm0_i2c.c (MPU6050 硬件 I2C 驱动) 的传输模式:
 *   - 写: reg 先入 FIFO → startControllerTransfer(TX, len+1) → 填 data
 *   - 读: reg 先入 FIFO + RD_ON_TXEMPTY → startControllerTransfer(RX, len)
 *
 * ============================================================================
 */
#include "hwi2c.h"
#include "system/clock.h"
#include <stdint.h>

#define I2C_TIMEOUT_MS  10U

/* ============================================================================
 * 等待 I2C 总线空闲
 * ============================================================================ */
static bool i2c_wait_idle(void) {
    uint32_t t0 = (uint32_t)tick_ms;
    while (DL_I2C_getControllerStatus(I2C_Gyro_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) return false;
    }
    return true;
}

/* ============================================================================
 * 清理 I2C 状态 (每次事务结束后调用)
 * ============================================================================ */
static void i2c_cleanup(void) {
    I2C_Gyro_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_Gyro_INST);
}

/* ============================================================================
 * 写 n 字节: START + SLA+W + reg + n bytes + STOP
 *
 * 参考 mspm0_i2c_write: reg 先入 TX FIFO, 等 IDLE, 然后 startControllerTransfer
 * 传输总长度 = 1(reg) + n(data) = len + 1
 * ============================================================================ */
static bool hw_i2c_write_burst(uint8_t slave_7bit, uint8_t reg,
                               const uint8_t *data, uint8_t len) {
    if (!i2c_wait_idle()) { i2c_cleanup(); return false; }

    /* 把 reg 地址放入 TX FIFO */
    DL_I2C_transmitControllerData(I2C_Gyro_INST, reg);
    DL_I2C_clearInterruptStatus(I2C_Gyro_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    while (!(DL_I2C_getControllerStatus(I2C_Gyro_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* 启动传输: 总长度 = 1(reg) + len(data) */
    DL_I2C_startControllerTransfer(I2C_Gyro_INST, slave_7bit,
                                    DL_I2C_CONTROLLER_DIRECTION_TX,
                                    (uint16_t)(len + 1));

    uint32_t t0 = (uint32_t)tick_ms;
    uint8_t cnt = len;
    const uint8_t *ptr = data;

    while (cnt > 0) {
        uint8_t filled = DL_I2C_fillControllerTXFIFO(
            I2C_Gyro_INST, ptr, cnt);
        cnt -= filled;
        ptr += filled;

        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) {
            i2c_cleanup();
            return false;
        }

        if (DL_I2C_getRawInterruptStatus(I2C_Gyro_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
            break;
        }
    }

    /* 等传输完成 */
    t0 = (uint32_t)tick_ms;
    while (!DL_I2C_getRawInterruptStatus(I2C_Gyro_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) break;
    }

    i2c_cleanup();
    return true;
}

/* ============================================================================
 * 读 n 字节: START + SLA+W + reg + RESTART + SLA+R + n bytes + STOP
 *
 * 参考 mspm0_i2c_read: reg 先入 TX FIFO + RD_ON_TXEMPTY_ENABLE,
 * startControllerTransfer(RX), 硬件自动先发 reg 再切 RX.
 * ============================================================================ */
static bool hw_i2c_read_burst(uint8_t slave_7bit, uint8_t reg,
                              uint8_t *data, uint8_t len) {
    if (!i2c_wait_idle()) { i2c_cleanup(); return false; }

    /* 把 reg 地址放入 TX FIFO */
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
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) {
            i2c_cleanup();
            return false;
        }
    }

    /* 读取 FIFO 中剩余数据 */
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_Gyro_INST)) {
        if (idx < len) {
            data[idx++] = DL_I2C_receiveControllerData(I2C_Gyro_INST);
        }
    }

    i2c_cleanup();
    return (idx == len);
}

/* ============================================================================
 * Init — 硬件 I2C0 由 SysConfig 完成配置, 这里不需要额外操作
 * ============================================================================ */
void SW_I2C_Init(void) {
    /* SysConfig 的 SYSCFG_DL_init() 已经配置好 I2C0 引脚和时钟 */
}

/* ============================================================================
 * 高层 API
 * ============================================================================ */

/* ProbeAddr: START + SLA+W + STOP. 看 ADDR_ACK 标志. */
bool SW_I2C_ProbeAddr(uint8_t addr_7bit) {
    if (!i2c_wait_idle()) { i2c_cleanup(); return false; }

    DL_I2C_startControllerTransfer(I2C_Gyro_INST, addr_7bit,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 0);

    uint32_t t0 = (uint32_t)tick_ms;
    while (!DL_I2C_getRawInterruptStatus(I2C_Gyro_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if ((uint32_t)((uint32_t)tick_ms - t0) > I2C_TIMEOUT_MS) {
            DL_I2C_resetControllerTransfer(I2C_Gyro_INST);
            i2c_cleanup();
            return false;
        }
    }

    bool ack = (DL_I2C_getControllerStatus(I2C_Gyro_INST)
                & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0;
    i2c_cleanup();
    return ack;
}

/* WriteReg: START + SLA+W + reg + val + STOP */
bool SW_I2C_WriteReg(uint8_t addr_7bit, uint8_t reg, uint8_t val) {
    return hw_i2c_write_burst(addr_7bit, reg, &val, 1u);
}

/* ReadRegs: START + SLA+W + reg + RESTART + SLA+R + N bytes + STOP */
bool SW_I2C_ReadRegs(uint8_t addr_7bit, uint8_t start_reg,
                     uint8_t *buf, uint8_t len) {
    return hw_i2c_read_burst(addr_7bit, start_reg, buf, len);
}

/* ReadReg: 单寄存器读便捷封装 */
bool SW_I2C_ReadReg(uint8_t addr_7bit, uint8_t reg, uint8_t *val) {
    return SW_I2C_ReadRegs(addr_7bit, reg, val, 1);
}
