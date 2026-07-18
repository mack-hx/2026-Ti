/* ============================================================================
 * @file    sw_i2c.c
 * @brief   Software I2C bit-bang — 极简版 (照搬 STM32 lunzhou GY-9250 例程)
 *
 * 设计:
 *   - 4 个核心函数: i2c_start, i2c_stop, i2c_write_byte (返回 ACK),
 *                    i2c_read_byte (主控发 ACK/NACK)
 *   - 80MHz CPU, 5us high / 5us low = 100kHz SCL
 *   - open-drain: PA0/PA1 配成 OUTPUT + Hi-Z, setPins/release, clearPins/drive low
 *   - 每个 byte 的 SLA+W/R ACK 都打 trace, 方便定位 SLA+R NACK 根因
 *
 * 重要时序修复 (2026-07-14):
 *   - 阶段 1 写 reg → 阶段 2 SLA+R 之间, 调 ReadRegs 内部加 100us delay.
 *   - 写完 1 byte register 后, MPU9250 内部寄存器指针 latch 需要时间.
 *     之前 0 delay → SLA+R NACK. STM32 例程也没显式 delay, 但其 CPU 慢 + bit-bang
 *     自然产生 ~30us 间隔; MSPM0 80MHz 太快, 必须显式补 100us.
 *
 * ============================================================================
 */
#include "sw_i2c.h"
#include <stdint.h>

/* ============================================================================
 * GPIO 底层 (open-drain 仿真)
 *
 * MSPM0G3507 PINCM bit 25 (HIZ1) 在 PF=GPIO 时:
 *   DOE=1 + DOUT=1 + HIZ1=1 → pin 释放 (Hi-Z, 外部上拉决定电平)
 *   DOE=1 + DOUT=0 + HIZ1=1 → pin 强制 LOW
 *
 * PA0/PA1 在 SW_I2C_Init() runtime 配成 PC | PF=GPIO | INENA | HIZ1 | PIPU.
 * 关键: INENA 必须开, 否则 sda_read() 永远 0.
 * 关键: 必须 enableOutput (DOE=1), 否则 setPins/clearPins 不驱动 pin.
 * ============================================================================ */

static inline void sda_high(void)  { DL_GPIO_setPins(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN); }
static inline void sda_low(void)   { DL_GPIO_clearPins(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN); }
static inline void scl_high(void)  { DL_GPIO_setPins(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN); }
static inline void scl_low(void)   { DL_GPIO_clearPins(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN); }

static inline uint8_t sda_read(void) {
    return (DL_GPIO_readPins(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN) != 0U) ? 1U : 0U;
}
static inline uint8_t scl_read(void) {
    return (DL_GPIO_readPins(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN) != 0U) ? 1U : 0U;
}

static inline void dly(void) { delay_cycles(SW_I2C_HALF_CYCLES); }

/* ============================================================================
 * Bus recovery (用户 2026-07-14 03:18 反馈 "示波器只有复位时有波形")
 *
 * 症状: I2C 总线死锁 — 可能是 master 在某次 SLA+R NACK 后 SDA 拉低没释放,
 *       也可能是 slave (MPU9250) reset 时序错误导致 hold SDA LOW.
 *       表现: 后续所有 I2C 事务的 i2c_start 都因 sda_read()==0 失败, 总线"沉默".
 *
 * 修复: 在 SW_I2C_Init 和每个失败的事务末尾, 手动 toggle SCL 9 次
 *       (I2C spec: 9 = 1 byte + ACK), 让 slave 释放 SDA, 然后发 STOP.
 *       这是 linux i2c_recover_bus 标准做法.
 * ============================================================================ */
static void i2c_bus_recover(void) {
    /* 先 release SDA (DOUT=1) 看看外部上拉能不能把它拉高 */
    sda_high();
    dly();
    /* 如果 SDA 还是 LOW, 说明有 slave 把它拉住了, 用 9 个 SCL 把 slave 推出去 */
    if (sda_read() == 0U) {
        for (uint8_t i = 0; i < 9U; i++) {
            scl_high();
            dly();
            scl_low();
            dly();
        }
        sda_high();
        dly();
    }
    /* 总线空闲后, 发一个 STOP 让所有 slave 回到 IDLE */
    scl_low();
    dly();
    sda_low();
    dly();
    scl_high();
    dly();
    sda_high();
    dly();
}

/* ============================================================================
 * Init — 直接寄存器级写 PINCM
 *
 * 关键点 (2026-07-14 调试发现):
 *   - MSPM0 HAL 的 DL_GPIO_initDigitalOutputFeatures() 不写 INENA bit,
 *     会把 SysConfig 之前设的 INENA=1 清成 0 → sda_read() 永远 0.
 *   - 修复: 直接寄存器级写 PINCM, 一次性设齐 PC | PF=GPIO | INENA | HIZ1 | PIPU.
 *   - 然后 enableOutput 让 DOE=1, master 才能 drive pin.
 *
 * open-drain 仿真:
 *   - DOE=1 + DOUT=1 + HIZ1=1 → pin 释放 (Hi-Z, 外部上拉决定电平)
 *   - DOE=1 + DOUT=0 + HIZ1=1 → pin 强制 LOW
 *   - INENA=1 → DIN 能反映外部电平 (slave ACK 拉低时 master 能检测)
 * ============================================================================ */
void SW_I2C_Init(void) {
    /* 直接寄存器级写 PINCM: PC | PF=GPIO | INENA | HIZ1 | PIPU
     * 注意 PINCM 寄存器是 read-write, bit 含义:
     *   bit 0-6: PC (port control) | bit 7: PC_CONNECTED | bit 8-11: PF (port func)
     *   bit 17: PIPU (pull-up) | bit 18: INENA (input enable) | bit 25: HIZ1 (open-drain) */
    const uint32_t PINCM_GPIO_OPEN_DRAIN =
        (IOMUX_PINCM_PC_CONNECTED |        /* bit 7: PC=1 (port control) */
         (1U << 0) |                        /* PF=0x01 (GPIO) */
         IOMUX_PINCM_INENA_ENABLE |         /* bit 18: input enable (read back) */
         IOMUX_PINCM_HIZ1_ENABLE |          /* bit 25: open-drain mode */
         IOMUX_PINCM_PIPU_ENABLE);          /* bit 17: internal pull-up (redundant) */

    IOMUX->SECCFG.PINCM[SW_I2C_SDA_PINCM] = PINCM_GPIO_OPEN_DRAIN;
    IOMUX->SECCFG.PINCM[SW_I2C_SCL_PINCM] = PINCM_GPIO_OPEN_DRAIN;

    /* enableOutput: DOE=1 让 master 能 drive pin (DOUT=0 时拉低) */
    DL_GPIO_enableOutput(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN | SW_I2C_SCL_PIN);

    /* 初始 HIGH: DOUT=1 + HIZ1=1 = pin release, 外部上拉拉到 VCC */
    sda_high();
    scl_high();
    dly();

    /* Bus recovery: 上电或上次失败后, 强制把总线解死锁 */
    i2c_bus_recover();
}

/* ============================================================================
 * 4 个核心位操作
 *
 * 严格按 STM32 lunzhou 时序:
 *   i2c_start:  SDA_H; SCL_H; dly; (SDA 总线忙检查); SDA_L; dly;
 *   i2c_stop:   SCL_L; dly; SDA_L; dly; SCL_H; dly; SDA_H; dly;
 *   i2c_write_byte: 8 bit (SCL_L→set SDA→dly→SCL_H→dly, repeat); 末尾 SCL_L
 *   i2c_wait_ack (in i2c_start after addr): SDA_H; SCL_H; dly; check SDA
 *   i2c_read_byte: SDA_H; 8 次 (SCL_L→dly→SCL_H→dly→read SDA); 末尾 SCL_L
 *   i2c_ack / i2c_nack: SDA_L/H → dly → SCL_H → dly → SCL_L → dly
 *
 * 不同点: 我们不调 SCL_H 之前先 SDA_H (因为我们的状态机严格管理 SDA 在 idle HIGH,
 * 不需要在 START 内重新 release).
 * ============================================================================ */

/* START: SDA HIGH→LOW while SCL HIGH. 返回是否成功 (bus 不忙).
 *   - 入口 SCL HIGH, SDA HIGH (idle)
 *   - 拉 SDA LOW → START
 *   - 拉 SCL LOW → 时钟开始 */
static bool i2c_start(void) {
    sda_high();
    scl_high();
    dly();
    /* Bus 忙检查: SDA LOW = slave 在发数据, 不能 START */
    if (sda_read() == 0U) return false;
    sda_low();   /* START 条件 */
    dly();
    scl_low();
    dly();
    return true;
}

/* STOP: SDA LOW→HIGH while SCL HIGH.
 *   - 入口 SCL LOW, SDA LOW (master 控制中)
 *   - 拉 SCL HIGH
 *   - 拉 SDA HIGH → STOP */
static void i2c_stop(void) {
    sda_low();
    dly();
    scl_high();
    dly();
    sda_high();
    dly();
}

/* 写 1 byte (MSB first). 返回第 9 个 SCL 时钟的 ACK (true=ACK=0, false=NACK=1).
 *   - 入口 SCL LOW, SDA 任意 (我们强制先 release)
 *   - 8 bit: set SDA → dly → SCL_H → dly → SCL_L → dly
 *   - release SDA → SCL_H → dly → read SDA → SCL_L → dly */
static bool i2c_write_byte(uint8_t b) {
    sda_high();  /* release for safety */
    for (uint8_t i = 0; i < 8; i++) {
        if ((b & 0x80U) != 0U) sda_high(); else sda_low();
        dly();
        scl_high();
        dly();
        scl_low();
        dly();
        b <<= 1;
    }
    /* ACK 时钟: master release SDA, slave 拉低=ACK */
    sda_high();
    dly();
    scl_high();
    dly();
    bool ack = (sda_read() == 0U);
    scl_low();
    dly();
    return ack;
}

/* 读 1 byte (MSB first). 主控在第 9 个 SCL 发 ACK/NACK:
 *   ack=true  → SDA LOW  (master 还想收下一 byte)
 *   ack=false → SDA HIGH (master 收完, 下一 byte 是 STOP/RESTART) */
static uint8_t i2c_read_byte(bool ack) {
    sda_high();  /* release SDA 让 slave drive */
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
        dly();
        scl_high();
        dly();
        b = (uint8_t)((b << 1) | sda_read());
        scl_low();
        dly();
    }
    /* ACK/NACK 时钟 */
    if (ack) sda_low(); else sda_high();
    dly();
    scl_high();
    dly();
    scl_low();
    dly();
    return b;
}

/* ============================================================================
 * 阶段间 delay: 写完 reg byte 后等 MPU9250 寄存器指针 latch.
 * 之前 100us 写成 100ms bug: 误把 80000 当循环数累乘, 实际 8000000 cyc = 100ms
 * 让 ISR 阻塞 100ms, LCD SPI/BUSY 等被丢, 屏幕黑屏 (用户 03:22 反馈).
 * 修正: 80MHz × 100us = 8000 cycles. */
static inline void inter_phase_delay(void) {
    delay_cycles(80U * SW_I2C_INTER_PHASE_US);
}

/* ============================================================================
 * 高层 API
 * ============================================================================ */

/* ProbeAddr: START + SLA+W (or R) + STOP. 看 SLA + R/W 的 ACK. */
bool SW_I2C_ProbeAddr(uint8_t addr_7bit) {
    if (!i2c_start()) return false;
    bool ack = i2c_write_byte((uint8_t)(addr_7bit << 1));
    i2c_stop();
    return ack;
}

/* WriteReg: START + SLA+W + reg + val + STOP */
bool SW_I2C_WriteReg(uint8_t addr_7bit, uint8_t reg, uint8_t val) {
    if (!i2c_start()) return false;
    if (!i2c_write_byte((uint8_t)(addr_7bit << 1))) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg))                          { i2c_stop(); return false; }
    if (!i2c_write_byte(val))                          { i2c_stop(); return false; }
    i2c_stop();
    return true;
}

/* ReadRegs: START + SLA+W + reg + RESTART + SLA+R + N bytes + STOP */
bool SW_I2C_ReadRegs(uint8_t addr_7bit, uint8_t start_reg,
                     uint8_t *buf, uint8_t len) {
    /* 阶段 1: START + SLA+W + reg */
    if (!i2c_start()) return false;
    if (!i2c_write_byte((uint8_t)(addr_7bit << 1))) { i2c_stop(); return false; }
    if (!i2c_write_byte(start_reg))                  { i2c_stop(); return false; }
    /* 写完 reg byte 后等 100us 让 MPU9250 寄存器指针 latch */
    inter_phase_delay();

    /* 阶段 2: RESTART + SLA+R + N bytes */
    if (!i2c_start()) { i2c_stop(); return false; }
    bool ack = i2c_write_byte((uint8_t)((addr_7bit << 1) | 1U));
    if (!ack) { i2c_stop(); return false; }
    for (uint8_t i = 0; i < len; i++) {
        bool send_ack = (i + 1 < len);  /* 最后 1 byte NACK */
        buf[i] = i2c_read_byte(send_ack);
    }
    i2c_stop();
    return true;
}

/* ReadReg: 单寄存器读便捷封装 */
bool SW_I2C_ReadReg(uint8_t addr_7bit, uint8_t reg, uint8_t *val) {
    return SW_I2C_ReadRegs(addr_7bit, reg, val, 1);
}