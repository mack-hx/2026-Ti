/* ============================================================================
 * @file    pd42s1.h
 * @brief   PD42S1 闭环步进电机驱动器 - 协议层 (SMD 自定义协议, 8 位校验和含帧头)
 *
 *   模块命名含义:
 *     - PD42 = PD42S1 驱动器型号
 *     - PD42S1 = 正点原子闭环步进电机驱动器
 *     - 所有 PD42S1_* 函数 = 协议层直接封装, 应用层通过 stepmotor.h 调用
 *
 *   命名约定:
 *     - PD42_FCT_* = 功能码 (Function code)
 *     - PD42_DIR_* = 方向 (CW=顺时针 / CCW=逆时针)
 *     - PD42_STATUS_* = 电机状态
 *     - PD42_ACK_* = 应答状态 (OK / ERR)
 *     - PD42_HOME_* = 回零模式
 *     - PD42_LIMIT_* = 限位模式
 *     - PD42_MODE_* = 工作模式
 *
 *   应用层用法:
 *     通常不直接调本文件 API, 而是通过 stepmotor.h 的高级封装:
 *       SM_Run / SM_Stop / SM_Move / SM_MoveTo / SM_ReadPosition
 *       SM_zeroset / SM_zero / SM_infzero / SM_limithome
 *       SM_Init / SM_Tick
 *
 *     底层接口 (调试 / 二次封装时用):
 *       PD42S1_Init / SendCommand / TakeFrame / GetFrame
 *       PD42S1_ReadPosition / SpeedMode / AbsPosMode / RelPosMode
 *       PD42S1_MotorEnable / ZeroPosition / ClearStatus / SaveParams
 *       PD42S1_SetLeftLimitOrigin / SetRightLimitOrigin / SetLimitHome
 *       PD42S1_TriggerHome / SetZeroTimeout / SetAutoHome / SetLimitSwitch
 *
 * ============================================================================
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
 * 功能码定义 (FCT = Function Code)
 * ============================================================================ */

/* 系统指令 (0x00~0x0F) */
typedef enum {
    PD42_FCT_IDLE            = 0x00,   /* IDLE: 空闲功能码 */
    PD42_FCT_CAL_ENCODER     = 0x01,   /* CAL_ENCODER: 校准编码器 */
    PD42_FCT_RESTART         = 0x02,   /* RESTART: 复位重启 */
    PD42_FCT_RESET_FACTORY   = 0x03,   /* RESET_FACTORY: 恢复出厂设置 */
    PD42_FCT_PARAM_SAVE      = 0x04,   /* PARAM_SAVE: 参数保存到 EEPROM */
} pd42_sys_cmd_t;

/* 读参数指令 (0x20~0x3F) */
typedef enum {
    PD42_FCT_READ_VER         = 0x20,   /* READ_VER: 读取软硬件版本 */
    PD42_FCT_READ_PSI         = 0x21,   /* READ_PSI: 读取电机磁链 */
    PD42_FCT_READ_RES_IND     = 0x22,   /* READ_RES_IND: 读取相电阻和电感 */
    PD42_FCT_READ_CURRENT     = 0x23,   /* READ_CURRENT: 读取相电流 */
    PD42_FCT_READ_VOLTAGE    = 0x24,   /* READ_VOLTAGE: 读取总线电压 */
    PD42_FCT_READ_MA_PID     = 0x25,   /* READ_MA_PID: 读取电流环 PID */
    PD42_FCT_READ_SPD_PID    = 0x26,   /* READ_SPD_PID: 读取速度环 PID */
    PD42_FCT_READ_POS_PID    = 0x27,   /* READ_POS_PID: 读取位置环 PID */
    PD42_FCT_READ_PULSE_CNT  = 0x28,   /* READ_PULSE_CNT: 读取累计脉冲数 */
    PD42_FCT_READ_SPEED      = 0x29,   /* READ_SPEED: 读取实时转速 */
    PD42_FCT_READ_POSITION   = 0x2A,   /* READ_POSITION: 读取实时位置 (最常用) */
    PD42_FCT_READ_POS_ERR    = 0x2B,   /* READ_POS_ERR: 读取位置误差 */
    PD42_FCT_READ_STATUS     = 0x2C,   /* READ_STATUS: 读取运行状态 */
    PD42_FCT_READ_STALL_FLAG = 0x2D,   /* READ_STALL_FLAG: 读取堵转标志 */
    PD42_FCT_READ_STALL_CURR = 0x2E,   /* READ_STALL_CURR: 读取堵转电流 */
    PD42_FCT_READ_ENABLE     = 0x2F,   /* READ_ENABLE: 读使能状态 */
    PD42_FCT_READ_ARRIVED    = 0x30,   /* READ_ARRIVED: 读取到位状态 */
} pd42_read_cmd_t;

/* 设置参数指令 (0x60~0x7F) */
typedef enum {
    PD42_FCT_SET_ADDR         = 0x60,   /* SET_ADDR: 设置从机地址 */
    PD42_FCT_SET_MODE         = 0x62,   /* SET_MODE: 设置工作模式 */
    PD42_FCT_SET_POS_PID      = 0x63,   /* SET_POS_PID: 设置位置环 PID */
    PD42_FCT_SET_POS_TORQUE   = 0x64,   /* SET_POS_TORQUE: 设置位置环最大力矩 */
    PD42_FCT_SET_MICROSTEP    = 0x65,   /* SET_MICROSTEP: 设置细分 */
    PD42_FCT_SET_CURRENT      = 0x66,   /* SET_CURRENT: 设置目标电流 */
    PD42_FCT_SET_BAUD         = 0x67,   /* SET_BAUD: 设置串口波特率 */
} pd42_set_cmd_t;

/* 运动控制指令 (0xE0~0xFF) */
typedef enum {
    PD42_FCT_OL_SPEED_MODE    = 0xE0,   /* OL_SPEED_MODE: 开环速度模式 */
    PD42_FCT_OL_ABS_POS_MODE  = 0xE1,   /* OL_ABS_POS_MODE: 开环绝对位置模式 */
    PD42_FCT_OL_REL_POS_MODE  = 0xE2,   /* OL_REL_POS_MODE: 开环相对位置模式 */
    PD42_FCT_TORQUE_MODE      = 0xF0,   /* TORQUE_MODE: 力矩模式 */
    PD42_FCT_SPEED_MODE       = 0xF1,   /* SPEED_MODE: 速度模式 (按住连续转) */
    PD42_FCT_ABS_POS_MODE     = 0xF2,   /* ABS_POS_MODE: 绝对位置模式 */
    PD42_FCT_REL_POS_MODE     = 0xF3,   /* REL_POS_MODE: 相对位置模式 (常用) */
    PD42_FCT_PULSE_MODE       = 0xF4,   /* PULSE_MODE: 脉冲模式 */
    PD42_FCT_ZERO_ANGLE       = 0xF8,   /* ZERO_ANGLE: 位置清零 */
    PD42_FCT_CLEAR_STALL      = 0xF9,   /* CLEAR_STALL: 解除堵转 */
    PD42_FCT_MOTOR_ENABLE     = 0xFA,   /* MOTOR_ENABLE: 电机使能/失能 */
    PD42_FCT_CLEAR_STATUS     = 0xFB,   /* CLEAR_STATUS: 清除状态 (STOP 后必调) */
    PD42_FCT_STOP_IMMEDIATE   = 0xFC,   /* STOP_IMMEDIATE: 立即停止 */
} pd42_ctrl_cmd_t;

/* 回零指令 (0x90~0x9F, 手册 4.5) */
typedef enum {
    PD42_FCT_SET_LEFT_LIMIT   = 0x90,   /* SET_LEFT_LIMIT: 设置左限位原点位置 */
    PD42_FCT_SET_LIMIT_HOME   = 0x91,   /* SET_LIMIT_HOME: 设置有无限位回零参数 */
    PD42_FCT_TRIGGER_HOME     = 0x92,   /* TRIGGER_HOME: 触发回零 */
    PD42_FCT_ABORT_HOME       = 0x93,   /* ABORT_HOME: 强制中断回零 */
    PD42_FCT_READ_HOME_PARAM  = 0x94,   /* READ_HOME_PARAM: 读取回零参数 */
    PD42_FCT_SET_ZERO_TIMEOUT = 0x95,   /* SET_ZERO_TIMEOUT: 修改原点回零超时时间 */
    PD42_FCT_READ_HOME_STATUS = 0x96,   /* READ_HOME_STATUS: 读取回零状态 */
    PD42_FCT_SET_AUTO_HOME    = 0x97,   /* SET_AUTO_HOME: 设置上电自动回零 */
    PD42_FCT_SET_RIGHT_LIMIT  = 0x98,   /* SET_RIGHT_LIMIT: 设置右限位原点位置 */
    PD42_FCT_SET_LIMIT_SWITCH = 0x99,   /* SET_LIMIT_SWITCH: 设置左右限位开关状态 */
} pd42_home_cmd_t;

/* 方向定义 (DIR = Direction) */
typedef enum {
    PD42_DIR_CW  = 0,    /* CW: 顺时针 (ClockWise) */
    PD42_DIR_CCW = 1,    /* CCW: 逆时针 (Counter-ClockWise) */
} pd42_dir_t;

/* 电机状态 (STATUS) */
typedef enum {
    PD42_STATUS_IDLE      = 0,    /* IDLE: 空闲 */
    PD42_STATUS_RUNNING   = 1,    /* RUNNING: 运行中 */
    PD42_STATUS_ARRIVED   = 2,    /* ARRIVED: 已到位 */
    PD42_STATUS_STALLED   = 3,    /* STALLED: 堵转 */
} pd42_status_t;

/* 应答状态 (ACK = Acknowledgment) */
typedef enum {
    PD42_ACK_OK              = 0x01,  /* OK: 应答成功 */
    PD42_ACK_FRAME_SHORT     = 0xE1,  /* FRAME_SHORT: 帧长度不足 */
    PD42_ACK_HEADER_ERR      = 0xE2,  /* HEADER_ERR: 帧头错误 */
    PD42_ACK_FOOTER_ERR      = 0xE3,  /* FOOTER_ERR: 帧尾错误 (常见丢步原因) */
    PD42_ACK_CRC_ERR         = 0xE4,  /* CRC_ERR: 校验错误 */
    PD42_ACK_UNSUPPORTED     = 0xE5,  /* UNSUPPORTED: 不支持的功能码 */
    PD42_ACK_ILLEGAL_VAL     = 0xE6,  /* ILLEGAL_VAL: 数据不合法 */
} pd42_ack_t;

/* 回零模式 (HOMING mode) */
#define PD42_HOME_SINGLE   0x00   /* 单圈回零: 按完整一圈找原点信号 */
#define PD42_HOME_NEAREST  0x01   /* 就近回零: 从当前位置朝最近原点位置移动 */
#define PD42_HOME_MULTI    0x02   /* 多圈回零: 找到绝对 0 点 */

/* 工作模式 (Work Mode) */
#define PD42_MODE_POS_LOOP      0x00   /* 通信位置模式 */
#define PD42_MODE_SPEED_LOOP     0x01   /* 通信速度模式 */
#define PD42_MODE_TORQUE_LOOP    0x02   /* 通信力矩模式 */
#define PD42_MODE_PULSE          0x03   /* 脉冲模式 */
#define PD42_MODE_PW_POS         0x04   /* 脉宽位置模式 */
#define PD42_MODE_PW_SPEED       0x05   /* 脉宽速度模式 */
#define PD42_MODE_PW_TORQUE      0x06   /* 脉宽力矩模式 */
#define PD42_MODE_HOME           0x07   /* 回零模式 */
#define PD42_MODE_OL_SPEED       0x08   /* 开环速度模式 */
#define PD42_MODE_OL_POS         0x09   /* 开环位置模式 */

/* 接收帧结构 (RX frame parsed from UART callback) */
typedef struct {
    uint8_t  slave_addr;      /* 从机地址 (0x01=X轴 / 0x02=Y轴) */
    uint8_t  function_code;   /* 功能码 */
    uint8_t  error_code;      /* 错误码 (0x01=OK, 其他=失败) */
    uint8_t  data[64];       /* 数据缓冲区 */
    uint8_t  data_len;       /* 数据长度 */
    uint16_t checksum;       /* 校验和 */
} pd42_frame_t;

/* 驱动器信息结构 (driver info) */
typedef struct {
    uint8_t  addr;           /* 从机地址 */
    uint16_t microstep;      /* 细分设置 */
    int16_t  current_ma;     /* 电流设置 (mA) */
    pd42_status_t status;    /* 当前状态 */
    int32_t  position;       /* 当前位置 */
    int32_t  speed_rpm;      /* 当前速度 */
} pd42_driver_t;

/**
 * @brief   PD42S1_SendCommand - 组帧并发送协议命令
 * @param   addr        从机地址 (0x01=X轴 / 0x02=Y轴)
 * @param   func_code   功能码 (例如 PD42_FCT_SPEED_MODE=0xF1)
 * @param   data        数据载荷 (可为 NULL)
 * @param   len         数据载荷字节数
 * @note    帧格式: [C5][ADDR][FUNC][DATA...][CHECKSUM][5C]
 *          CHECKSUM = sum([C5][ADDR][FUNC][DATA...]) & 0xFF
 * @usage   底层协议封装用, 应用层通过 stepmotor.h 调用
 */
void PD42S1_SendCommand(uint8_t addr, uint8_t func_code, uint8_t *data, uint8_t len);

/* PD42S1_WaitResponse: 等待应答 (阻塞, 不推荐使用) */
bool PD42S1_WaitResponse(uint32_t timeout_ms);

/**
 * @brief   PD42S1_TakeFrame - 原子取走一帧新应答
 * @param   frame  帧结构指针
 * @return  true=有帧并已取走, false=无新帧
 * @note    无新帧时返回 false, 原子操作防止 ISR 竞争
 * @usage   SM_Tick 中调用
 */
bool PD42S1_TakeFrame(pd42_frame_t *frame);

/**
 * @brief   PD42S1_TakeFrameFor - 按轴取走新应答
 * @param   addr   从机地址 (SM_X=0x01 / SM_Y=0x02)
 * @param   frame  帧结构指针
 * @return  true=有帧并已取走, false=无新帧
 * @usage   SM_Tick 中调用, 推荐使用
 */
bool PD42S1_TakeFrameFor(uint8_t addr, pd42_frame_t *frame);

/**
 * @brief   PD42S1_GetFrame - 只读查看最近一帧 (不消费)
 * @return  帧指针
 * @usage   LCD 显示用, 不影响 frame_ready 标志
 */
pd42_frame_t* PD42S1_GetFrame(void);

/**
 * @brief   PD42S1_GetFrameFor - 按轴只读查看最近一帧 (不消费)
 * @param   addr  从机地址
 * @return  帧指针
 */
pd42_frame_t* PD42S1_GetFrameFor(uint8_t addr);

/**
 * @brief   PD42S1_GetTxBuffer - 按轴读取最近一次 TX 快照
 * @param   addr   从机地址
 * @param   len    返回缓冲区长度 (可为 NULL)
 * @param   fired  返回是否已发出 (可为 NULL)
 * @return  TX 缓冲区指针
 * @usage   LCD 显示最近发送的帧内容
 */
const volatile uint8_t* PD42S1_GetTxBuffer(uint8_t addr, uint8_t *len, bool *fired);

/* ============================================================================
 * 系统指令
 * ============================================================================ */

/**
 * @brief   PD42S1_CalEncoder - 校准编码器
 * @param   addr  从机地址
 * @note    发送 0x01 指令
 */
void PD42S1_CalEncoder(uint8_t addr);

/**
 * @brief   PD42S1_Restart - 复位重启驱动器
 * @param   addr  从机地址
 * @note    发送 0x02 指令
 */
void PD42S1_Restart(uint8_t addr);

/**
 * @brief   PD42S1_ResetFactory - 恢复出厂设置
 * @param   addr  从机地址
 * @note    发送 0x03 指令, 会丢失所有配置
 */
void PD42S1_ResetFactory(uint8_t addr);

/**
 * @brief   PD42S1_SaveParams - 保存参数到 EEPROM
 * @param   addr  从机地址
 * @note    发送 0x04 指令, 掉电不丢失
 * @usage   修改配置后必须调用
 */
void PD42S1_SaveParams(uint8_t addr);

/* ============================================================================
 * 读取参数
 * ============================================================================ */

/**
 * @brief   PD42S1_ReadVersion - 读取软硬件版本
 */
void PD42S1_ReadVersion(uint8_t addr);

/**
 * @brief   PD42S1_ReadVoltage - 读取总线电压
 */
void PD42S1_ReadVoltage(uint8_t addr);

/**
 * @brief   PD42S1_ReadSpeed - 读取实时转速
 */
void PD42S1_ReadSpeed(uint8_t addr);

/**
 * @brief   PD42S1_ReadPosition - 读取实时位置 (最常用)
 * @param   addr  从机地址
 * @note    发送 0x2A, 驱动器返回位置数据
 * @usage   用于查询当前位置
 */
void PD42S1_ReadPosition(uint8_t addr);

/**
 * @brief   PD42S1_ReadStatus - 读取运行状态
 */
void PD42S1_ReadStatus(uint8_t addr);

/**
 * @brief   PD42S1_ReadArrived - 读取到位状态
 * @note    0x30, 用于查询电机是否到达目标位置
 */
void PD42S1_ReadArrived(uint8_t addr);

/* ============================================================================
 * 运动控制
 * ============================================================================ */

/**
 * @brief   PD42S1_SpeedMode - 速度模式
 * @param   addr        从机地址
 * @param   dir         方向: 0=CW(正转) / 1=CCW(反转)
 * @param   accel      加速度 (0~200, 0=直接启动)
 * @param   speed_rpm  速度 (RPM, float)
 * @note    按住连续转, 松开需调 StopImmediate
 * @usage   速度控制时用
 */
void PD42S1_SpeedMode(uint8_t addr, pd42_dir_t dir, uint8_t accel, float speed_rpm);

/**
 * @brief   PD42S1_SendSpeedRaw - 直接发送速度模式原始数据 (保证大端 float 字节序)
 * @param   addr        从机地址
 * @param   dir         方向: 0=CW / 1=CCW
 * @param   accel      加速度
 * @param   speed_b3~b0 速度字节 (IEEE754 float, 大端)
 * @note    用于确保和大端 float 字节序一致
 * @usage   stepmotor.c 内部调用
 */
void PD42S1_SendSpeedRaw(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                          uint8_t speed_b3, uint8_t speed_b2,
                          uint8_t speed_b1, uint8_t speed_b0);

/**
 * @brief   PD42S1_AbsPosMode - 绝对位置模式
 * @param   addr        从机地址
 * @param   dir         方向: 0=CW / 1=CCW
 * @param   accel      加速度 (0~200)
 * @param   speed      速度 (RPM)
 * @param   pulses     目标绝对位置 (脉冲数, int32, 大端, 51200=一圈)
 * @note    转到驱动器侧编码器的绝对位置
 * @usage   绝对定位时用
 */
void PD42S1_AbsPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                        uint16_t speed, int32_t pulses);

/**
 * @brief   PD42S1_RelPosMode - 相对位置模式
 * @param   addr        从机地址
 * @param   dir         方向: 0=CW / 1=CCW
 * @param   accel      加速度 (0~200)
 * @param   speed      速度 (RPM)
 * @param   pulses     相对位移脉冲数 (int32, 大端, 51200=一圈)
 * @note    从当前位置走一段相对位移
 * @usage   相对移动时用
 */
void PD42S1_RelPosMode(uint8_t addr, pd42_dir_t dir, uint8_t accel,
                       uint16_t speed, int32_t pulses);

/* ============================================================================
 * 辅助功能
 * ============================================================================ */

/**
 * @brief   PD42S1_MotorEnable - 电机使能/失能
 * @param   addr    从机地址
 * @param   enable  true=使能, false=失能
 * @note    DATA=0 使能, DATA=1 失能
 */
void PD42S1_MotorEnable(uint8_t addr, bool enable);

/**
 * @brief   PD42S1_StopImmediate - 立即停止
 * @param   addr  从机地址
 * @note    发送 0xFC, 无数据字段
 * @usage   松开按键时调用, 之后必须调 ClearStatus
 */
void PD42S1_StopImmediate(uint8_t addr);

/**
 * @brief   PD42S1_ClearStall - 解除堵转
 * @param   addr  从机地址
 */
void PD42S1_ClearStall(uint8_t addr);

/**
 * @brief   PD42S1_ClearStatus - 清除状态
 * @param   addr  从机地址
 * @note    发送 0xFB, 无数据字段
 * @usage   StopImmediate 后必须调, 否则电机发烫
 */
void PD42S1_ClearStatus(uint8_t addr);

/**
 * @brief   PD42S1_ZeroPosition - 位置清零
 * @param   addr  从机地址
 * @note    发送 0xF8, 当前位置变为 0
 */
void PD42S1_ZeroPosition(uint8_t addr);

/* ============================================================================
 * 参数设置
 * ============================================================================ */

/**
 * @brief   PD42S1_SetMicrostep - 设置细分
 * @param   addr  从机地址
 * @param   step  细分值
 */
void PD42S1_SetMicrostep(uint8_t addr, uint16_t step);

/**
 * @brief   PD42S1_SetCurrent - 设置目标电流
 * @param   addr       从机地址
 * @param   current_ma 电流 (mA)
 */
void PD42S1_SetCurrent(uint8_t addr, int16_t current_ma);

/**
 * @brief   PD42S1_SetWorkMode - 设置工作模式
 * @param   addr  从机地址
 * @param   mode  模式:
 *              0x00 = 通信位置模式 (最常用)
 *              0x01 = 通信速度模式
 *              0x02 = 通信力矩模式
 *              0x08 = 开环速度模式
 *              0x09 = 开环位置模式
 * @note    设置后需调用 SaveParams 保存
 */
void PD42S1_SetWorkMode(uint8_t addr, uint8_t mode);

/* ============================================================================
 * 回零类指令 (手册 4.5)
 * ============================================================================ */

/**
 * @brief   PD42S1_SetLeftLimitOrigin - 设置左限位原点位置
 * @param   addr    从机地址
 * @param   pulses  原点位置 (int32, 大端, 51200=一圈)
 * @note    发送 0x90, 设置后需调 SaveParams 落盘
 */
void PD42S1_SetLeftLimitOrigin(uint8_t addr, int32_t pulses);

/**
 * @brief   PD42S1_SetRightLimitOrigin - 设置右限位原点位置
 * @param   addr    从机地址
 * @param   pulses  原点位置 (int32, 大端, 51200=一圈)
 * @note    发送 0x98, 设置后需调 SaveParams 落盘
 */
void PD42S1_SetRightLimitOrigin(uint8_t addr, int32_t pulses);

/**
 * @brief   PD42S1_SetLimitHome - 设置有无限位回零参数
 * @param   addr       从机地址
 * @param   mode        模式: 0=左无限位, 1=右无限位, 2=左有限位, 3=右有限位
 * @param   dir         回零方向: 0=CW / 1=CCW
 * @param   speed_rpm  回零速度 (RPM, float 大端)
 * @param   limit_ma  限位电流 (mA, 仅无限位回零生效)
 * @note    发送 0x91
 */
void PD42S1_SetLimitHome(uint8_t addr, uint8_t mode, uint8_t dir,
                         uint16_t speed_rpm, uint16_t limit_ma);

/**
 * @brief   PD42S1_SetZeroTimeout - 修改原点回零超时时间
 * @param   addr        从机地址
 * @param   timeout_ms  超时时间 (ms, uint32 大端)
 * @note    发送 0x95, 设置后需调 SaveParams 落盘
 */
void PD42S1_SetZeroTimeout(uint8_t addr, uint32_t timeout_ms);

/**
 * @brief   PD42S1_SetAutoHome - 设置上电自动回零
 * @param   addr    从机地址
 * @param   enable  true=上电自动回零, false=不自动
 * @note    发送 0x97, 设置后需调 SaveParams 落盘
 *          标志仅在下次驱动器上电时生效
 */
void PD42S1_SetAutoHome(uint8_t addr, bool enable);

/**
 * @brief   PD42S1_TriggerHome - 触发回零
 * @param   addr  从机地址
 * @param   mode  模式: PD42_HOME_SINGLE(0)=单圈 / PD42_HOME_NEAREST(1)=就近 / PD42_HOME_MULTI(2)=多圈
 * @note    发送 0x92
 * @usage   设好回零参数后调用
 */
void PD42S1_TriggerHome(uint8_t addr, uint8_t mode);

/**
 * @brief   PD42S1_SetLimitSwitch - 设置左右限位开关状态
 * @param   addr    从机地址
 * @param   enable  true=开启左右限位, false=关闭
 * @note    发送 0x99, 开启后行程被框在 [左限位, 右限位]
 *          与 0x91 的区别: 0x99 控制正常运动是否限位
 */
void PD42S1_SetLimitSwitch(uint8_t addr, bool enable);

/* ============================================================================
 * 初始化和回调
 * ============================================================================ */

/**
 * @brief   PD42S1_Init - 初始化 PD42S1 协议层
 * @param   baud_rate  UART 波特率
 */
void PD42S1_Init(uint32_t baud_rate);

/**
 * @brief   PD42S1_UART_CallbackFor - UART 中断回调，转发给指定轴
 * @param   addr     从机地址 (SM_X=0x01 / SM_Y=0x02)
 * @param   rx_data  UART 收到的字节
 */
void PD42S1_UART_CallbackFor(uint8_t addr, uint8_t rx_data);

/* ============================================================================
 * 全局变量 (供外部读取显示)
 * ============================================================================ */

/* TX 发送缓冲区 (供 LCD 显示最近发送的帧) */
extern volatile uint8_t g_tx_buffer[64];
extern volatile uint8_t g_tx_buffer_len;
extern volatile bool g_tx_ready;
extern volatile bool g_tx_fired;

/* 诊断计数器 (供 LCD 显示 RX 链路状态) */
extern volatile uint32_t g_rx_byte_cnt;        /* UART 中断收到的字节数 */
extern volatile uint32_t g_rx_frame_done_cnt; /* 收到完整帧的次数 */
extern volatile uint32_t g_rx_chk_ok_cnt;    /* 校验通过次数 */
extern volatile uint32_t g_rx_chk_fail_cnt;  /* 校验失败次数 */
extern volatile uint8_t g_last_frame_len;     /* 最近一次帧的字节数 */
extern volatile uint8_t g_last_frame_data_len; /* 最近一次帧的 DATA 长度 */
extern volatile uint8_t g_last_frame_raw[16];  /* 最近一次帧的原始字节 (最多16字节) */
extern volatile bool g_captured;              /* 已抓帧，停止更新 */

/**
 * @brief   PD42S1_ResetCapture - 重置帧抓取状态
 * @usage   开始新一次帧抓取前调用
 */
void PD42S1_ResetCapture(void);

#endif /* __PD42S1_H__ */
