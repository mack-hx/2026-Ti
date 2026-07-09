/**
 * @file    pd42s1.h
 * @brief   PD42S1 闭环步进电机驱动器控制接口
 * @note    基于正点原子SMD协议，适用于MSPM0G3507
 */
#ifndef __PD42S1_H__
#define __PD42S1_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 通信配置
 * ============================================================================ */
#define PD42S1_UART_INST        UART0       /* 使用UART0连接驱动器 */
#define PD42S1_BAUD_RATE        115200      /* 驱动器默认波特率 */
#define PD42S1_DEFAULT_ADDR     0x01         /* 默认从机地址 */

/* 通信协议帧格式 */
#define PD42S1_FRAME_HEAD       0xC5        /* 帧头 */
#define PD42S1_FRAME_TAIL       0x5C        /* 帧尾 */

/* ============================================================================
 * 功能码定义
 * ============================================================================ */

/* 系统指令 (0x00~0x0F) */
typedef enum {
    PD42_FCT_IDLE            = 0x00,   /* 空闲功能码 */
    PD42_FCT_CAL_ENCODER     = 0x01,   /* 校准编码器 */
    PD42_FCT_RESTART         = 0x02,   /* 复位重启 */
    PD42_FCT_RESET_FACTORY   = 0x03,   /* 恢复出厂设置 */
    PD42_FCT_PARAM_SAVE      = 0x04,   /* 参数保存 */
} pd42_sys_cmd_t;

/* 读参数指令 (0x20~0x3F) */
typedef enum {
    PD42_FCT_READ_VER         = 0x20,   /* 读取软硬件版本 */
    PD42_FCT_READ_PSI         = 0x21,   /* 读取电机磁链 */
    PD42_FCT_READ_RES_IND     = 0x22,   /* 读取相电阻和电感 */
    PD42_FCT_READ_CURRENT     = 0x23,   /* 读取相电流 */
    PD42_FCT_READ_VOLTAGE    = 0x24,   /* 读取总线电压 */
    PD42_FCT_READ_MA_PID     = 0x25,   /* 读取电流环PID */
    PD42_FCT_READ_SPD_PID    = 0x26,   /* 读取速度环PID */
    PD42_FCT_READ_POS_PID    = 0x27,   /* 读取位置环PID */
    PD42_FCT_READ_PULSE_CNT  = 0x28,   /* 读取累计脉冲数 */
    PD42_FCT_READ_SPEED      = 0x29,   /* 读取实时转速 */
    PD42_FCT_READ_POSITION   = 0x2A,   /* 读取实时位置 */
    PD42_FCT_READ_POS_ERR    = 0x2B,   /* 读取位置误差 */
    PD42_FCT_READ_STATUS     = 0x2C,   /* 读取运行状态 */
    PD42_FCT_READ_STALL_FLAG = 0x2D,   /* 读取堵转标志 */
    PD42_FCT_READ_STALL_CURR = 0x2E,   /* 读取堵转电流 */
    PD42_FCT_READ_ENABLE     = 0x2F,   /* 读使能状态 */
    PD42_FCT_READ_ARRIVED    = 0x30,   /* 读取到位状态 */
} pd42_read_cmd_t;

/* 设置参数指令 (0x60~0x7F) */
typedef enum {
    PD42_FCT_SET_ADDR         = 0x60,   /* 设置从机地址 */
    PD42_FCT_SET_MODE         = 0x62,   /* 设置工作模式 */
    PD42_FCT_SET_POS_PID      = 0x63,   /* 设置位置环PID */
    PD42_FCT_SET_POS_TORQUE   = 0x64,   /* 设置位置环最大力矩 */
    PD42_FCT_SET_MICROSTEP    = 0x65,   /* 设置细分 */
    PD42_FCT_SET_CURRENT      = 0x66,   /* 设置目标电流 */
    PD42_FCT_SET_BAUD         = 0x67,   /* 设置串口波特率 */
} pd42_set_cmd_t;

/* 运动控制指令 (0xE0~0xFF) */
typedef enum {
    PD42_FCT_OL_SPEED_MODE    = 0xE0,   /* 开环速度模式 */
    PD42_FCT_OL_ABS_POS_MODE  = 0xE1,   /* 开环绝对位置模式 */
    PD42_FCT_OL_REL_POS_MODE  = 0xE2,   /* 开环相对位置模式 */
    PD42_FCT_TORQUE_MODE      = 0xF0,   /* 力矩模式 */
    PD42_FCT_SPEED_MODE       = 0xF1,   /* 速度模式 */
    PD42_FCT_ABS_POS_MODE     = 0xF2,   /* 绝对位置模式 */
    PD42_FCT_REL_POS_MODE     = 0xF3,   /* 相对位置模式 */
    PD42_FCT_PULSE_MODE       = 0xF4,   /* 脉冲模式 */
    PD42_FCT_ZERO_ANGLE       = 0xF8,   /* 位置清零 */
    PD42_FCT_CLEAR_STALL      = 0xF9,   /* 解除堵转 */
    PD42_FCT_MOTOR_ENABLE     = 0xFA,   /* 电机使能 */
    PD42_FCT_CLEAR_STATUS     = 0xFB,   /* 清除状态 */
    PD42_FCT_STOP_IMMEDIATE   = 0xFC,   /* 立即停止 */
} pd42_ctrl_cmd_t;

/* 方向定义 */
typedef enum {
    PD42_DIR_CW  = 0,    /* 顺时针 */
    PD42_DIR_CCW = 1,    /* 逆时针 */
} pd42_dir_t;

/* 电机状态 */
typedef enum {
    PD42_STATUS_IDLE      = 0,    /* 空闲 */
    PD42_STATUS_RUNNING   = 1,    /* 运行中 */
    PD42_STATUS_ARRIVED   = 2,    /* 已到位 */
    PD42_STATUS_STALLED   = 3,    /* 堵转 */
} pd42_status_t;

/* 应答状态 */
typedef enum {
    PD42_ACK_OK              = 0x01,  /* 应答成功 */
    PD42_ACK_FRAME_SHORT     = 0xE1,  /* 帧长度不足 */
    PD42_ACK_HEADER_ERR      = 0xE2,  /* 帧头错误 */
    PD42_ACK_FOOTER_ERR      = 0xE3,  /* 帧尾错误 */
    PD42_ACK_CRC_ERR         = 0xE4,  /* 校验错误 */
    PD42_ACK_UNSUPPORTED     = 0xE5,  /* 不支持的功能码 */
    PD42_ACK_ILLEGAL_VAL     = 0xE6,  /* 数据不合法 */
} pd42_ack_t;

/* 接收帧结构 */
typedef struct {
    uint8_t  slave_addr;      /* 从机地址 */
    uint8_t  function_code;   /* 功能码 */
    uint8_t  error_code;      /* 错误码 */
    uint8_t  data[64];       /* 数据缓冲区 */
    uint8_t  data_len;       /* 数据长度 */
    uint16_t checksum;       /* 校验和 */
} pd42_frame_t;

/* 驱动器信息结构 */
typedef struct {
    uint8_t  addr;           /* 从机地址 */
    uint16_t microstep;      /* 细分设置 */
    int16_t  current_ma;     /* 电流设置 (mA) */
    pd42_status_t status;    /* 当前状态 */
    int32_t  position;       /* 当前位置 */
    int32_t  speed_rpm;      /* 当前速度 */
} pd42_driver_t;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/* 初始化 */
void PD42S1_Init(uint32_t baud_rate);
void PD42S1_SendCommand(uint8_t addr, uint8_t func_code, uint8_t *data, uint8_t len);
bool PD42S1_WaitResponse(uint32_t timeout_ms);
pd42_frame_t* PD42S1_GetFrame(void);

/* 系统指令 */
void PD42S1_CalEncoder(uint8_t addr);
void PD42S1_Restart(uint8_t addr);
void PD42S1_ResetFactory(uint8_t addr);
void PD42S1_SaveParams(uint8_t addr);

/* 读取参数 */
void PD42S1_ReadVersion(uint8_t addr);
void PD42S1_ReadVoltage(uint8_t addr);
void PD42S1_ReadSpeed(uint8_t addr);
void PD42S1_ReadPosition(uint8_t addr);
void PD42S1_ReadStatus(uint8_t addr);
void PD42S1_ReadArrived(uint8_t addr);

/* 运动控制 - 速度模式 */
void PD42S1_SpeedMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, float speed_rpm);

/* 运动控制 - 位置模式 */
void PD42S1_AbsPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses);
void PD42S1_RelPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, uint16_t speed, uint32_t pulses);

/* 辅助功能 */
void PD42S1_MotorEnable(uint8_t addr, bool enable);
void PD42S1_StopImmediate(uint8_t addr);
void PD42S1_ClearStall(uint8_t addr);
void PD42S1_ZeroPosition(uint8_t addr);

/* 参数设置 */
void PD42S1_SetMicrostep(uint8_t addr, uint16_t step);
void PD42S1_SetCurrent(uint8_t addr, int16_t current_ma);

/* CRC校验 */
uint16_t PD42S1_CalcCRC16(const uint8_t *data, uint8_t len);

#endif /* __PD42S1_H__ */
