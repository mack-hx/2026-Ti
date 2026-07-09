/**
 * @file    pd42s1.c
 * @brief   PD42S1 闭环步进电机驱动器控制接口实现
 * @note    基于正点原子SMD协议，适用于MSPM0G3507
 */
#include "pd42s1.h"
#include "ti/driverlib/DL_UART.h"
#include "ti/driverlib/DL_GPIO.h"
#include <string.h>

/* ============================================================================
 * 内部变量
 * ============================================================================ */
static pd42_frame_t g_rx_frame;           /* 接收帧缓冲区 */
static uint8_t g_rx_buffer[128];          /* UART接收缓冲区 */
static volatile bool g_frame_ready = false;

/* 接收数据回调 - 由UART中断调用 */
void PD42S1_UART_Callback(uint8_t rx_data) {
    static uint8_t rx_index = 0;
    static uint8_t frame_len = 0;
    
    /* 简单协议解析: 帧头 + 地址 + 功能码 + 数据 + 校验 + 帧尾 */
    if (rx_index == 0 && rx_data == PD42S1_FRAME_HEAD) {
        g_rx_buffer[rx_index++] = rx_data;
    } else if (rx_index > 0 && rx_index < sizeof(g_rx_buffer)) {
        g_rx_buffer[rx_index++] = rx_data;
        
        /* 简单帧解析: 长度至少为7字节 [HEAD][ADDR][FUNC][LEN][DATA...][CRC_H][CRC_L][TAIL] */
        if (rx_index >= 7 && rx_data == PD42S1_FRAME_TAIL) {
            /* 完整帧接收完成 */
            frame_len = rx_index;
            rx_index = 0;
            
            if (frame_len >= 7) {
                /* 解析帧 */
                g_rx_frame.slave_addr = g_rx_buffer[1];
                g_rx_frame.function_code = g_rx_buffer[2];
                g_rx_frame.data_len = g_rx_buffer[3];
                
                if (g_rx_frame.data_len < sizeof(g_rx_frame.data)) {
                    memcpy(g_rx_frame.data, &g_rx_buffer[4], g_rx_frame.data_len);
                }
                
                /* CRC校验 */
                uint16_t recv_crc = (g_rx_buffer[4 + g_rx_frame.data_len] << 8) | 
                                    g_rx_buffer[5 + g_rx_frame.data_len];
                uint16_t calc_crc = PD42S1_CalcCRC16(&g_rx_buffer[1], 4 + g_rx_frame.data_len);
                
                if (recv_crc == calc_crc) {
                    g_rx_frame.checksum = recv_crc;
                    g_frame_ready = true;
                }
            }
        }
    } else {
        rx_index = 0;
    }
}

/* ============================================================================
 * 基础通信
 * ============================================================================ */
void PD42S1_Init(uint32_t baud_rate) {
    g_frame_ready = false;
    memset(&g_rx_frame, 0, sizeof(g_rx_frame));
    (void)baud_rate; /* 实际波特率在SysConfig中配置 */
}

void PD42S1_SendCommand(uint8_t addr, uint8_t func_code, uint8_t *data, uint8_t len) {
    uint8_t tx_buffer[64];
    uint8_t tx_index = 0;
    uint8_t frame_len;
    
    /* 组帧: [HEAD][ADDR][FUNC][DATA_LEN][DATA...][CRC_H][CRC_L][TAIL] */
    tx_buffer[tx_index++] = PD42S1_FRAME_HEAD;
    tx_buffer[tx_index++] = addr;
    tx_buffer[tx_index++] = func_code;
    tx_buffer[tx_index++] = len;
    
    if (len > 0 && data != NULL) {
        memcpy(&tx_buffer[tx_index], data, len);
        tx_index += len;
    }
    
    /* 计算并添加CRC */
    uint16_t crc = PD42S1_CalcCRC16(&tx_buffer[1], tx_index - 1);
    tx_buffer[tx_index++] = (uint8_t)(crc >> 8);
    tx_buffer[tx_index++] = (uint8_t)(crc & 0xFF);
    
    tx_buffer[tx_index++] = PD42S1_FRAME_TAIL;
    
    /* 发送帧 */
    frame_len = tx_index;
    
    /* 使用DriverLib发送数据 - 实际串口发送需要根据硬件配置 */
    /* 这里使用占位符，实际使用时请在SysConfig中配置UART */
    extern DL_UARTController UART0;
    for (uint8_t i = 0; i < frame_len; i++) {
        while (!DL_UART_isTXReady(UART0));
        DL_UART_transmitData(UART0, tx_buffer[i]);
    }
}

bool PD42S1_WaitResponse(uint32_t timeout_ms) {
    volatile uint32_t cnt = 0;
    
    while (!g_frame_ready && cnt < timeout_ms * 1000) {
        /* 简单延时，实际应使用定时器 */
        for (volatile uint32_t i = 0; i < 1000; i++);
        cnt++;
    }
    
    return g_frame_ready;
}

pd42_frame_t* PD42S1_GetFrame(void) {
    g_frame_ready = false;
    return &g_rx_frame;
}

/* ============================================================================
 * CRC16校验 (Modbus)
 * ============================================================================ */
uint16_t PD42S1_CalcCRC16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
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
 * 速度模式控制
 * ============================================================================ */
/**
 * @brief   速度模式控制
 * @param   addr        从机地址
 * @param   dir         方向: 0=顺时针, 1=逆时针
 * @param   accel       加速度 (1-255)
 * @param   speed_rpm   目标速度 (RPM), 支持小数
 */
void PD42S1_SpeedMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, float speed_rpm) {
    uint8_t data[6];
    uint32_t speed_int = (uint32_t)(speed_rpm * 100); /* 扩大100倍传输 */
    
    data[0] = dir;
    data[1] = accel;
    data[2] = (uint8_t)(speed_int >> 24);
    data[3] = (uint8_t)(speed_int >> 16);
    data[4] = (uint8_t)(speed_int >> 8);
    data[5] = (uint8_t)(speed_int & 0xFF);
    
    PD42S1_SendCommand(addr, PD42_FCT_SPEED_MODE, data, 6);
}

/* ============================================================================
 * 位置模式控制
 * ============================================================================ */
/**
 * @brief   绝对位置模式控制
 * @param   addr        从机地址
 * @param   dir         方向: 0=顺时针, 1=逆时针
 * @param   accel      加速度 (1-255)
 * @param   speed       速度 (RPM)
 * @param   pulses     目标脉冲数 (绝对位置)
 */
void PD42S1_AbsPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses) {
    uint8_t data[9];
    
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
 * @brief   相对位置模式控制
 * @param   addr        从机地址
 * @param   dir         方向: 0=顺时针, 1=逆时针
 * @param   accel      加速度 (1-255)
 * @param   speed       速度 (RPM)
 * @param   pulses     目标脉冲数 (相对位移)
 */
void PD42S1_RelPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, uint16_t speed, uint32_t pulses) {
    uint8_t data[9];
    
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
void PD42S1_MotorEnable(uint8_t addr, bool enable) {
    uint8_t data[1];
    data[0] = enable ? 1 : 0;
    PD42S1_SendCommand(addr, PD42_FCT_MOTOR_ENABLE, data, 1);
}

void PD42S1_StopImmediate(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_STOP_IMMEDIATE, NULL, 0);
}

void PD42S1_ClearStall(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_CLEAR_STALL, NULL, 0);
}

void PD42S1_ZeroPosition(uint8_t addr) {
    PD42S1_SendCommand(addr, PD42_FCT_ZERO_ANGLE, NULL, 0);
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
