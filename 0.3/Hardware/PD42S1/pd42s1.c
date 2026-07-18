/* ============================================================================
 * @file    pd42s1.c
 * @brief   PD42S1 闭环步进电机驱动器 - 协议底层 (SMD 自定义协议, 8 位校验和)
 *
 * ============================================================================
 * 调用方法 (应用层通常通过 stepmotor.h 间接调用, 不直接接触本文件)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     PD42S1_Init(PD42S1_BAUD_RATE);     // 默认 115200, 清接收缓冲
 *
 *   中断入口 (main.c 里挂):
 *     void x_bujin_INST_IRQHandler(void) {
 *         if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
 *             PD42S1_UART_Callback(DL_UART_receiveData(x_bujin_INST));
 *         }
 *     }
 *
 *   应用层不直接调这些函数 (stepmotor.h 已封装高级 API):
 *     PD42S1_SendCommand(addr, func, data, len)   协议帧发送 (阻塞)
 *     PD42S1_UART_Callback(rx_data)               UART 中断回调
 *     PD42S1_TakeFrame(frame)                     原子取走最新应答帧
 *     PD42S1_GetFrame()                           只读 peek 最新帧
 *
 * ============================================================================
 * 协议帧格式 (正点原子自定义协议):
 *   下行帧: [C5][ADDR][FUNC][DATA...][CHECKSUM][5C]
 *   上行帧: [C5][ADDR][FUNC][ERR][DATA...][CHECKSUM][5C]
 *   CHECKSUM = sum([C5][ADDR][FUNC][DATA...]) & 0xFF  (包含帧头, 1 字节)
 *
 * 数据字段字节序:
 *   int16_t  → [MSB][LSB] (大端)
 *   int32_t  → [B3][B2][B1][B0] (大端)
 *   float    → IEEE754 4 字节 (大端)
 *
 * ============================================================================
 */
#include "pd42s1.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdbool.h>
#include <cmsis_compiler.h>

/* ============================================================================
 * 内部变量
 * ============================================================================ */
static pd42_frame_t g_rx_frame;           /* 接收帧缓冲区 */
static volatile bool g_frame_ready = false;

/* 供外部使用的发送缓冲区 */
volatile uint8_t g_tx_buffer[64];
volatile uint8_t g_tx_buffer_len = 0;
volatile bool g_tx_ready = false;
volatile bool g_tx_fired = false;      /* 刚发送过，main.c 抓TX用 */

/* === 诊断计数器 (供 LCD 显示) === */
volatile uint32_t g_rx_byte_cnt = 0;        /* UART 中断收到的字节数 */
volatile uint32_t g_rx_frame_done_cnt = 0; /* 收到完整帧的次数 */
volatile uint32_t g_rx_chk_ok_cnt = 0;     /* 校验通过次数 */
volatile uint32_t g_rx_chk_fail_cnt = 0;   /* 校验失败次数 */
volatile uint8_t g_last_frame_len = 0;     /* 最近一次帧的字节数 */
volatile uint8_t g_last_frame_data_len = 0; /* 最近一次帧的 DATA 长度 */
volatile uint8_t g_last_frame_raw[16];     /* 最近一次帧的原始字节 (最多16字节) */
volatile bool g_captured = false;          /* 已抓帧，停止更新 */

/* ============================================================================
 * 8位校验和 (正点原子自定义协议)
 * CHECKSUM = sum(data[0..len-1]) & 0xFF
 * 包含帧头 0xC5 在校验范围内
 * ============================================================================ */
static uint8_t checksum8(const uint8_t *data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* ============================================================================
 * 接收数据回调 - 由 UART 中断调用
 * 协议帧: [C5][ADDR][FUNC][ERR][DATA...][CHECKSUM][5C]
 * 最小帧长 = 7 字节 ([HEAD][ADDR][FUNC][ERR][CHK][TAIL])
 * ============================================================================ */
/**
 * @brief   把状态机复位到"等待帧头"
 * @note    必须在收到合法帧尾、校验失败、长度异常等所有"退出"路径上调用，
 *          否则下一次新帧的 HEAD (0xC5) 会被当成数据字节写到 rx_buffer[1],
 *          后续收到的字节都会偏移一格，CHKSUM/TAIL 全部错位，
 *          LCD 上 RX 的 FUNC 永远对不上 0x2A → K3 读位置看起来"有时灵有时不灵"。
 *          原来只在 rx_index>=64 时清零，丢了绝大多数同步机会。
 */
static inline void sm_rx_reset(uint8_t *idx) {
    *idx = 0;
}

void PD42S1_UART_Callback(uint8_t rx_data) {
    static uint8_t rx_buffer[64];
    static uint8_t rx_index = 0;

    g_rx_byte_cnt++;

    /* 阶段 0: 等帧头 (0xC5) — 任何非 0xC5 字节直接丢弃 (包括驱动器空闲时
     * 上位机 TX 回环、噪声字节等)。这样状态机不会"假启动"。 */
    if (rx_index == 0) {
        if (rx_data == PD42S1_FRAME_HEAD) {
            rx_buffer[rx_index++] = rx_data;
        }
        return;
    }

    /* 阶段 1: 已经收到 HEAD, 开始累计数据 */
    if (rx_index >= sizeof(rx_buffer)) {
        /* 极端情况: 没收到 TAIL 但 buffer 已满。直接复位重新等 HEAD。 */
        sm_rx_reset(&rx_index);
        return;
    }
    rx_buffer[rx_index++] = rx_data;

    /* 中途又收到 HEAD: 如果 buffer 里前面那些字节根本不像一个完整帧
     * (长度 < 6 一定不是合法帧), 直接丢弃前面所有，从这个 HEAD 重新开始。
     * 这一招能"抢回"大多数因上一帧错位导致的失同步状态。 */
    if (rx_data == PD42S1_FRAME_HEAD && rx_index > 1) {
        /* 仅当到目前为止 buffer 还没凑出一个合法帧 (长度 < 6) 时,
         * 才允许把这一字节当作新帧头。否则它是 TAIL 之前的数据，不动。 */
        if (rx_index - 1 < 6) {
            rx_buffer[0] = PD42S1_FRAME_HEAD;
            rx_index = 1;
        }
        return;  /* 等下一字节 (可能是 ADDR) */
    }

    /* 没收到 TAIL 就先继续累计 */
    if (rx_data != PD42S1_FRAME_TAIL) return;

    /* 收到 TAIL: 帧最短也要 [HEAD][ADDR][FUNC][CHK][TAIL] = 5B
     * 但应用层协议最小帧 = 6B (含 ERR 字节), 这里放宽到 6B。*/
    if (rx_index < 6) {
        /* 不合法长度 → 复位 */
        sm_rx_reset(&rx_index);
        return;
    }

    uint8_t frame_len = rx_index;

    /* 校验和: HEAD..CHK前一字节 求和 & 0xFF
     * CHECKSUM 是 TAIL 之前那字节 */
    uint8_t chk_data_len = frame_len - 2;
    uint8_t recv_chk = rx_buffer[frame_len - 2];
    uint8_t calc_chk = checksum8(&rx_buffer[0], chk_data_len);

    if (recv_chk != calc_chk) {
        g_rx_chk_fail_cnt++;
        sm_rx_reset(&rx_index);
        return;
    }

    /* === 校验通过, 解析帧 === */
    g_rx_frame.slave_addr    = rx_buffer[1];
    g_rx_frame.function_code = rx_buffer[2];

    /* 帧结构: [HEAD][ADDR][FUNC][ERR][DATA...][CHK][TAIL]
     * 非数据字节 = [HEAD][ADDR][FUNC] + [CHK][TAIL] = 5 字节
     * data[0] = ERR, data[1..] = DATA  */
    g_rx_frame.data_len = (frame_len > 5) ? (frame_len - 5) : 0;
    g_rx_frame.error_code = (g_rx_frame.data_len >= 1) ? rx_buffer[3] : 0;

    if (g_rx_frame.data_len > 0) {
        memcpy(g_rx_frame.data, &rx_buffer[3], g_rx_frame.data_len);
    }
    g_rx_frame.checksum = recv_chk;

    g_frame_ready = true;
    g_rx_chk_ok_cnt++;

    g_last_frame_len = frame_len;
    g_last_frame_data_len = g_rx_frame.data_len;

    /* 只抓第一帧 (之后停止更新 last_frame_raw, 给 LCD 显示完整快照) */
    if (!g_captured) {
        uint8_t copy_len = (frame_len < 16) ? frame_len : 16;
        for (uint8_t i = 0; i < copy_len; i++) {
            g_last_frame_raw[i] = rx_buffer[i];
        }
        g_captured = true;
    }
    g_rx_frame_done_cnt++;

    /* ★★★ 关键: 解析完一个完整帧后必须复位, 否则下一次新帧的 HEAD
     * 会被当成上一帧的延续, 状态机从此永久错位。*/
    sm_rx_reset(&rx_index);
}

/* ============================================================================
 * 基础通信
 * ============================================================================ */
void PD42S1_Init(uint32_t baud_rate) {
    g_frame_ready = false;
    memset(&g_rx_frame, 0, sizeof(g_rx_frame));
    (void)baud_rate;
}

/**
 * @brief   组帧并发送
 * @param   addr        从机地址
 * @param   func_code   功能码
 * @param   data        数据载荷 (可为 NULL)
 * @param   len         数据载荷字节数
 *
 * 帧格式: [C5][ADDR][FUNC][DATA...][CHECKSUM][5C]
 * CHECKSUM = sum([C5][ADDR][FUNC][DATA...]) & 0xFF
 */
void PD42S1_SendCommand(uint8_t addr, uint8_t func_code, uint8_t *data, uint8_t len) {
    uint8_t tx_buf[64];
    uint8_t tx_index = 0;

    tx_buf[tx_index++] = PD42S1_FRAME_HEAD; /* 0xC5 */
    tx_buf[tx_index++] = addr;
    tx_buf[tx_index++] = func_code;

    if (len > 0 && data != NULL) {
        memcpy(&tx_buf[tx_index], data, len);
        tx_index += len;
    }

    /* 8位校验和，包含帧头 */
    uint8_t chk = checksum8(tx_buf, tx_index);
    tx_buf[tx_index++] = chk;

    tx_buf[tx_index++] = PD42S1_FRAME_TAIL;    /* 0x5C */

    /* 阻塞发送每个字节 */
    for (uint8_t i = 0; i < tx_index; i++) {
        DL_UART_transmitDataBlocking(x_bujin_INST, tx_buf[i]);
    }

    /* 供上层观察 */
    g_tx_buffer_len = tx_index;
    memcpy((void *)g_tx_buffer, tx_buf, tx_index);
    g_tx_ready = true;
    g_tx_fired = true;
}

bool PD42S1_WaitResponse(uint32_t timeout_ms) {
    volatile uint32_t cnt = 0;
    while (!g_frame_ready && cnt < timeout_ms * 1000) {
        for (volatile uint32_t i = 0; i < 1000; i++) __NOP();
        cnt++;
    }
    return g_frame_ready;
}

bool PD42S1_TakeFrame(pd42_frame_t *frame) {
    if (frame == NULL || !g_frame_ready) return false;

    __disable_irq();
    if (!g_frame_ready) {
        __enable_irq();
        return false;
    }
    memcpy(frame, &g_rx_frame, sizeof(*frame));
    g_frame_ready = false;
    g_captured = false;
    __enable_irq();
    return true;
}

pd42_frame_t* PD42S1_GetFrame(void) {
    return &g_rx_frame;
}

/* ============================================================================
 * 校验和计算 (供外部引用)
 * ============================================================================ */
uint16_t PD42S1_CalcCRC16(const uint8_t *data, uint8_t len) {
    return checksum8(data, len);
}

void PD42S1_ResetCapture(void) {
    g_captured = false;
    g_last_frame_len = 0;
    for (uint8_t i = 0; i < 16; i++) {
        g_last_frame_raw[i] = 0;
    }
}

/* ============================================================================
 * 系统指令
 * ============================================================================ */
void PD42S1_CalEncoder(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_CAL_ENCODER, NULL, 0);
}
void PD42S1_Restart(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_RESTART, NULL, 0);
}
void PD42S1_ResetFactory(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_RESET_FACTORY, NULL, 0);
}
void PD42S1_SaveParams(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_PARAM_SAVE, NULL, 0);
}

/* ============================================================================
 * 读取参数
 * ============================================================================ */
void PD42S1_ReadVersion(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_VER, NULL, 0);
}
void PD42S1_ReadVoltage(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_VOLTAGE, NULL, 0);
}
void PD42S1_ReadSpeed(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_SPEED, NULL, 0);
}
void PD42S1_ReadPosition(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_POSITION, NULL, 0);
}
void PD42S1_ReadStatus(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_STATUS, NULL, 0);
}
void PD42S1_ReadArrived(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_READ_ARRIVED, NULL, 0);
}

/* ============================================================================
 * 运动控制 - 速度模式 (大端 float)
 * @note  协议规定: [DIR][ACCEL][SPEED_4BYTE_FLOAT]
 *        速度字节序: [b3][b2][b1][b0] (big-endian，与手册示例一致)
 *        例: 60.0f → 0x42700000 → 原始字节 [42][70][00][00]
 * ============================================================================ */
void PD42S1_SpeedMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, float speed_rpm) {
    uint8_t data[6];
    data[0] = dir;
    data[1] = accel;
    /* float → 4 字节 (小端 / little-endian，与 STM32 memcpy 行为一致) */
    uint32_t speed_bits;
    memcpy(&speed_bits, &speed_rpm, sizeof(float));
    data[2] = (uint8_t)(speed_bits & 0xFF);
    data[3] = (uint8_t)(speed_bits >> 8);
    data[4] = (uint8_t)(speed_bits >> 16);
    data[5] = (uint8_t)(speed_bits >> 24);
    PD42S1_SendCommand(addr, PD42_FCT_SPEED_MODE, data, 6);
}

/**
 * @brief   直接发送速度模式原始数据（保证大端 float 字节序）
 * @note    速度字节序: [b3][b2][b1][b0] (big-endian)
 *          例: 60.0f → 0x42700000 → 原始字节 [42][70][00][00]
 *          正转: [C5][01][F1][00][64][42][70][00][00][D9][5C]
 *          反转: [C5][01][F1][01][64][42][70][00][00][DA][5C]
 */
void PD42S1_SendSpeedRaw(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                          uint8_t speed_b3, uint8_t speed_b2,
                          uint8_t speed_b1, uint8_t speed_b0) {
    uint8_t data[6];
    data[0] = dir;
    data[1] = accel;
    data[2] = speed_b3;
    data[3] = speed_b2;
    data[4] = speed_b1;
    data[5] = speed_b0;
    PD42S1_SendCommand(addr, PD42_FCT_SPEED_MODE, data, 6);
}

/* ============================================================================
 * 运动控制 - 位置模式
 * ============================================================================ */
/**
 * @brief   绝对位置模式
 * @param   addr        从机地址
 * @param   dir         方向: 0=顺时针(CW), 1=逆时针(CCW)
 * @param   accel      加速度 (0~200, 0=直接启动)
 * @param   speed      速度 (RPM)
 * @param   pulses     目标脉冲数 (51200=一圈), int32_t 大端
 */
void PD42S1_AbsPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                        uint16_t speed, int32_t pulses) {
    uint8_t data[8];
    data[0] = dir;
    data[1] = accel;
    data[2] = (uint8_t)(speed >> 8);
    data[3] = (uint8_t)(speed & 0xFF);
    data[4] = (uint8_t)(pulses >> 24);
    data[5] = (uint8_t)(pulses >> 16);
    data[6] = (uint8_t)(pulses >> 8);
    data[7] = (uint8_t)(pulses & 0xFF);
    PD42S1_SendCommand(addr, PD42_FCT_ABS_POS_MODE, data, 8);
}

/**
 * @brief   相对位置模式
 * @param   addr        从机地址
 * @param   dir         方向: 0=顺时针(CW), 1=逆时针(CCW)
 * @param   accel      加速度 (0~200, 0=直接启动)
 * @param   speed      速度 (RPM)
 * @param   pulses     相对位移脉冲数 (51200=一圈), int32_t 大端
 */
void PD42S1_RelPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                       uint16_t speed, int32_t pulses) {
    uint8_t data[8];
    data[0] = dir;
    data[1] = accel;
    data[2] = (uint8_t)(speed >> 8);
    data[3] = (uint8_t)(speed & 0xFF);
    data[4] = (uint8_t)(pulses >> 24);
    data[5] = (uint8_t)(pulses >> 16);
    data[6] = (uint8_t)(pulses >> 8);
    data[7] = (uint8_t)(pulses & 0xFF);
    PD42S1_SendCommand(addr, PD42_FCT_REL_POS_MODE, data, 8);
}

/* ============================================================================
 * 辅助功能
 * ============================================================================ */
/**
 * @brief   电机使能/失能
 * @note    协议规定: DATA=0 → 使能电机; DATA=1 → 失能电机
 */
void PD42S1_MotorEnable(uint8_t addr, bool enable) {
    uint8_t data[1];
    data[0] = enable ? 0 : 1;
    PD42S1_SendCommand(addr, PD42_FCT_MOTOR_ENABLE, data, 1);
}

void PD42S1_StopImmediate(uint8_t addr) {
    /* 手册 4.4.13：立即停止指令无数据字段 */
    PD42S1_SendCommand(addr, PD42_FCT_STOP_IMMEDIATE, NULL, 0);
}

void PD42S1_ClearStatus(uint8_t addr) {
    /* 手册 4.4.12：清除状态指令无数据字段 */
    PD42S1_SendCommand(addr, PD42_FCT_CLEAR_STATUS, NULL, 0);
}

void PD42S1_ClearStall(uint8_t addr) {
    uint8_t data[1] = {1};
    PD42S1_SendCommand(addr, PD42_FCT_CLEAR_STALL, data, 1);
}

void PD42S1_ZeroPosition(uint8_t addr) {
    uint8_t data[1] = {1};
    PD42S1_SendCommand(addr, PD42_FCT_ZERO_ANGLE, data, 1);
}

/* ============================================================================
 * 参数设置
 * ============================================================================ */
void PD42S1_SetMicrostep(uint8_t addr, uint16_t step) {
    uint8_t data[2];
    data[0] = (uint8_t)(step >> 8);
    data[1] = (uint8_t)(step & 0xFF);
    PD42S1_SendCommand(addr, PD42_FCT_SET_MICROSTEP, data, 2);
}

void PD42S1_SetCurrent(uint8_t addr, int16_t current_ma) {
    uint8_t data[2];
    data[0] = (uint8_t)(current_ma >> 8);
    data[1] = (uint8_t)(current_ma & 0xFF);
    PD42S1_SendCommand(addr, PD42_FCT_SET_CURRENT, data, 2);
}

/**
 * @brief   设置工作模式
 * @param   addr    从机地址
 * @param   mode    模式:
 *                  0x00 = 通信位置模式
 *                  0x01 = 通信速度模式  ← 速度命令需要这个
 *                  0x02 = 通信力矩模式
 *                  0x08 = 开环速度模式
 *                  0x09 = 开环位置模式
 * @note    设置后需调用 PD42S1_SaveParams 保存到 EEPROM
 */
void PD42S1_SetWorkMode(uint8_t addr, uint8_t mode) {
    uint8_t data[1];
    data[0] = mode;
    PD42S1_SendCommand(addr, PD42_FCT_SET_MODE, data, 1);
}

/* ============================================================================
 * 回零类指令 (手册 4.5)
 * ============================================================================ */

/**
 * @brief   设置左限位原点位置 (手册 4.5.1, 0x90)
 * @param   pulses  原点位置 (int32 big-endian, 51200 = 一圈)
 * @note    设置后需调 PD42S1_SaveParams 落盘, 否则掉电丢失 */
void PD42S1_SetLeftLimitOrigin(uint8_t addr, int32_t pulses) {
    uint8_t data[4];
    data[0] = (uint8_t)(pulses >> 24);
    data[1] = (uint8_t)(pulses >> 16);
    data[2] = (uint8_t)(pulses >>  8);
    data[3] = (uint8_t)(pulses);
    PD42S1_SendCommand(addr, PD42_FCT_SET_LEFT_LIMIT, data, 4);
}

/**
 * @brief   设置右限位原点位置 (手册 4.5.9, 0x98)
 * @note    设置后需调 PD42S1_SaveParams 落盘 */
void PD42S1_SetRightLimitOrigin(uint8_t addr, int32_t pulses) {
    uint8_t data[4];
    data[0] = (uint8_t)(pulses >> 24);
    data[1] = (uint8_t)(pulses >> 16);
    data[2] = (uint8_t)(pulses >>  8);
    data[3] = (uint8_t)(pulses);
    PD42S1_SendCommand(addr, PD42_FCT_SET_RIGHT_LIMIT, data, 4);
}

/**
 * @brief   设置有无限位回零参数 (手册 4.5.2, 0x91)
 * @param   mode        模式: 0=左无限位, 1=右无限位, 2=左有限位, 3=右有限位
 * @param   dir         回零方向: 0=正转, 1=反转
 * @param   speed_rpm   回零速度 (0~6000 RPM), float IEEE754 大端
 * @param   limit_ma    限位电流 (0~3000 mA), 仅无限位回零生效
 * @note    这是驱动器侧的回零路径参数, 设置后再调 PD42S1_TriggerHome 才有效。
 *          无限位回零不依赖外部限位开关, 靠电机堵转检测到位。
 */
void PD42S1_SetLimitHome(uint8_t addr, uint8_t mode, uint8_t dir,
                         uint16_t speed_rpm, uint16_t limit_ma) {
    union { float f; uint32_t u; } u;
    u.f = (float)speed_rpm;
    uint8_t data[8];
    data[0] = mode;
    data[1] = dir;
    /* float IEEE754, big-endian (手册示例 2000 RPM = 0x00 0x00 0x07 0xD0) */
    data[2] = (uint8_t)(u.u >> 24);
    data[3] = (uint8_t)(u.u >> 16);
    data[4] = (uint8_t)(u.u >>  8);
    data[5] = (uint8_t)(u.u);
    /* 限位电流 uint16, big-endian */
    data[6] = (uint8_t)(limit_ma >> 8);
    data[7] = (uint8_t)(limit_ma & 0xFF);
    PD42S1_SendCommand(addr, PD42_FCT_SET_LIMIT_HOME, data, 8);
}

/**
 * @brief   修改原点回零超时时间 (手册 4.5.6, 0x95)
 * @param   timeout_ms   超时 (uint32 big-endian, 单位 ms)
 * @note    设置后需调 PD42S1_SaveParams 落盘 */
void PD42S1_SetZeroTimeout(uint8_t addr, uint32_t timeout_ms) {
    uint8_t data[4];
    data[0] = (uint8_t)(timeout_ms >> 24);
    data[1] = (uint8_t)(timeout_ms >> 16);
    data[2] = (uint8_t)(timeout_ms >>  8);
    data[3] = (uint8_t)(timeout_ms);
    PD42S1_SendCommand(addr, PD42_FCT_SET_ZERO_TIMEOUT, data, 4);
}

/**
 * @brief   设置上电自动回零 (手册 4.5.8, 0x97)
 * @param   enable   true = 上电自动回零, false = 不自动
 * @note    自动回零用的零点对应**左限位原点** (手册 4.5.8)。
 *          设置后需调 PD42S1_SaveParams 落盘; 标志仅在下次驱动器上电时生效。 */
void PD42S1_SetAutoHome(uint8_t addr, bool enable) {
    uint8_t data[1] = { enable ? 1U : 0U };
    PD42S1_SendCommand(addr, PD42_FCT_SET_AUTO_HOME, data, 1);
}

/**
 * @brief   触发回零 (手册 4.5.3, 0x92)
 * @param   mode   PD42_HOME_SINGLE (单圈) / PD42_HOME_NEAREST (就近) /
 *                 PD42_HOME_MULTI (多圈找绝对 0 点)
 * @note    回零过程中可调 PD42S1_ReadStatus / ReadArrived 等查询状态;
 *          想强制中断用手册 4.5.4 (0x93) - 本文件暂未封装。 */
void PD42S1_TriggerHome(uint8_t addr, uint8_t mode) {
    uint8_t data[1] = { mode };
    PD42S1_SendCommand(addr, PD42_FCT_TRIGGER_HOME, data, 1);
}

/**
 * @brief   设置左右限位开关状态 (手册 4.5.10, 0x99)
 * @param   enable  true = 开启左右限位 (电机运动范围受限于左右限位原点),
 *                  false = 关闭左右限位
 * @note    开启后 0x90 (左限位原点) 和 0x98 (右限位原点) 限定的范围框
 *          住电机行程, 撞到任一边都会自动停机 (不分撞左/撞右, 是固定两端限位)。
 *          与 0x91 SET_LIMIT_HOME 的 mode=2/3 "有限位回零模式" 是独立的:
 *          0x99 控制**正常运动**是否被限位框住,
 *          0x91 控制**回零动作**按哪种回零路径走。
 *          实际"有限位回零"动作靠 0x99 开 + 0x90/0x98 设坐标 + 0x92 NEAREST 触发。
 */
void PD42S1_SetLimitSwitch(uint8_t addr, bool enable) {
    uint8_t data[1] = { enable ? 1U : 0U };
    PD42S1_SendCommand(addr, PD42_FCT_SET_LIMIT_SWITCH, data, 1);
}
