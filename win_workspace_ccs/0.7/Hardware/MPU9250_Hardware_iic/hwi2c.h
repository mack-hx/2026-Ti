/* ============================================================================
 * @file    hwi2c.h
 * @brief   硬件 I2C 驱动 — MPU9250 专用, 使用 I2C0 (PA0=SDA, PA1=SCL)
 *
 *   SysConfig 配置 I2C0 在 PA0(SDA)/PA1(SCL), 由 I2C_Gyro_INST 宏引用.
 *
 * ============================================================================
 */
#ifndef __HWI2C_H__
#define __HWI2C_H__

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ============================================================================
 * API (名称保持不变, 供 mpu9250.c 直接调用)
 * ============================================================================ */

/* 初始化硬件 I2C0 (SysConfig 已配置, 这里只需 enable power + reset) */
void SW_I2C_Init(void);

/* 发 START + addr (R/W) + STOP, 仅看 ACK (用于地址扫描) */
bool SW_I2C_ProbeAddr(uint8_t addr_7bit);

/* 写 1 个寄存器: START + SLA+W + reg + val + STOP */
bool SW_I2C_WriteReg(uint8_t addr_7bit, uint8_t reg, uint8_t val);

/* 读 N 个寄存器 (从 start_reg 起): START + SLA+W + reg + RESTART +
 *   SLA+R + read N bytes + STOP */
bool SW_I2C_ReadRegs(uint8_t addr_7bit, uint8_t start_reg,
                     uint8_t *buf, uint8_t len);

/* 单寄存器读便捷封装 */
bool SW_I2C_ReadReg(uint8_t addr_7bit, uint8_t reg, uint8_t *val);

#endif /* __HWI2C_H__ */
