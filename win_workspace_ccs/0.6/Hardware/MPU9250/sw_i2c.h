/* ============================================================================
 * @file    sw_i2c.h
 * @brief   Software I2C bit-bang on PA0(SDA)/PA1(SCL) — 极简版
 *
 * 设计原则 (2026-07-14 重构, 从 SDK-style wrapper 改成 STM32 风格直接 API):
 *   - 不再模仿硬件 I2C controller 的 fillTXFIFO + startTransfer 抽象.
 *   - 直接暴露 3 个 API: ProbeAddr / ReadReg / WriteReg, 每个 API 自带完整
 *     START + ADDR + DATA + STOP 时序.
 *   - 照搬 STM32 GY-9250 例程 (lunzhou I2C_Start/SendByte/WaitAck/ReadByte)
 *     结构, 时序参考已经验证能跑 9 轴.
 *
 * 关键时序修复 (2026-07-14 用户反馈 "调试半天没结果" 后):
 *   - 阶段 1 写 reg 地址 和 阶段 2 SLA+R 之间 加 100us delay,
 *     让 MPU9250 内部寄存器指针 latch (SLA+R NACK 根因)
 *   - 写完一个 register 给 MPU9250 5ms (post-write delay) 让内部状态机更新
 *   - 100kHz SCL = 10us/cycle (高 5us + 低 5us)
 *
 * SysConfig 配合:
 *   - MPU9250 group (GPIO10): PA0=INPUT hi-Z ENABLE, PA1=INPUT hi-Z ENABLE
 *   - 硬件 I2C_1 controller 占 PA30/PA17, 跟 MPU9250 物理无关
 *   - 外部上拉由用户 4.7k 接到 3.3V
 *
 * ============================================================================
 */
#ifndef __SW_I2C_H__
#define __SW_I2C_H__

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 引脚 */
#define SW_I2C_SDA_PORT        GPIOA
#define SW_I2C_SDA_PIN         DL_GPIO_PIN_0
#define SW_I2C_SCL_PORT        GPIOA
#define SW_I2C_SCL_PIN         DL_GPIO_PIN_1
#define SW_I2C_SDA_PINCM       IOMUX_PINCM1
#define SW_I2C_SCL_PINCM       IOMUX_PINCM2

/* 时序 (80MHz CPU, 1 cycle = 12.5ns)
 *   100kHz I2C: 周期 10us = 800 cycles. 高低各 5us = 400 cycles.
 *   用 500 cycles = 6.25us 留 buffer, SCL 实际 ~80kHz (略慢但稳). */
#define SW_I2C_HALF_CYCLES     500U

/* 阶段间 delay (写 reg 后等 MPU9250 寄存器指针 latch)
 * 用户 2026-07-14 发现 SLA+R 阶段 ADRACK=1: 阶段 1 写 0x75 后紧接 SLA+R,
 * 寄存器指针还没 set 完. MPU9250 数据手册写后需至少 1us, 保险用 100us. */
#define SW_I2C_INTER_PHASE_US  100U  /* 阶段 1 → 阶段 2 之间 */

/* ============================================================================
 * API
 * ============================================================================ */

/* 上电: 把 PA0/PA1 配成 OUTPUT + Hi-Z + 初始 HIGH (模拟 open-drain)
 * SysConfig 已经把 PA0/PA1 配成 INPUT + hi-Z=ENABLE (无内部上拉),
 * 运行时切到 OUTPUT + Hi-Z 让 master 能 drive low. */
void SW_I2C_Init(void);

/* 发 START + addr (R/W) + STOP, 仅看 ACK (用于地址扫描)
 *   返回 true = slave ACK, false = NACK (bus 上没设备) */
bool SW_I2C_ProbeAddr(uint8_t addr_7bit);

/* 写 1 个寄存器: START + SLA+W + reg + val + STOP
 *   返回 true = 全部 ACK */
bool SW_I2C_WriteReg(uint8_t addr_7bit, uint8_t reg, uint8_t val);

/* 读 N 个寄存器 (从 start_reg 起): START + SLA+W + reg + RESTART +
 *   SLA+R + read N bytes + STOP. 最后 1 byte NACK, 其余 ACK.
 *   返回 true = 全部 ACK, false = 任一阶段 NACK
 *   buf 至少 len 字节. */
bool SW_I2C_ReadRegs(uint8_t addr_7bit, uint8_t start_reg,
                     uint8_t *buf, uint8_t len);

/* 单寄存器读便捷封装 */
bool SW_I2C_ReadReg(uint8_t addr_7bit, uint8_t reg, uint8_t *val);

#endif /* __SW_I2C_H__ */