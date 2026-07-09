/*
 * @file    main.c
 * @brief   PD42S1 闭环步进电机控制 - MSPM0G3507
 * @note    基于正点原子SMD协议，适用于电赛备赛
 */
#include "ti_msp_dl_config.h"
#include "PD42S1/pd42s1.h"
#include <stdint.h>
#include <string.h>

/* ============================================================================
 * 硬件配置
 * ============================================================================ */
/*
 * PD42S1 驱动器连接:
 * - X轴步进: UART2 (PB15=TX, PB16=RX) -> x_bujin
 * - Y轴步进: UART3 (PB2=TX, PB3=RX)   -> y_bujin
 * - 波特率: 115200
 */
#define PD42S1_X_ADDR      0x01        /* X轴驱动器地址 */
#define PD42S1_Y_ADDR      0x02        /* Y轴驱动器地址 */

/* 按键定义 (GPIO) */
#define KEY1_PORT          GPIOB
#define KEY1_PIN           DL_GPIO_PIN_27   /* K1_B00 */
#define KEY2_PORT          GPIOB
#define KEY2_PIN           DL_GPIO_PIN_1    /* K2B_01 */

/* LED定义 */
#define LED_PORT           GPIOA
#define LED_PIN            DL_GPIO_PIN_14   /* B22 */

/* ============================================================================
 * 全局变量
 * ============================================================================ */
static volatile bool g_x_arrived = false;  /* X轴到位标志 */
static volatile bool g_y_arrived = false;  /* Y轴到位标志 */

/* ============================================================================
 * 延时函数
 * ============================================================================ */
static void delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 1000; j++) {
            __NOP();
        }
    }
}

/* ============================================================================
 * 按键扫描
 * ============================================================================ */
static uint8_t key_scan(void) {
    static uint8_t key_state = 0;
    
    if (DL_GPIO_readPins(KEY1_PORT, KEY1_PIN) == 0) {
        delay_ms(10);
        if (DL_GPIO_readPins(KEY1_PORT, KEY1_PIN) == 0) {
            return 1;  /* KEY1按下 */
        }
    }
    
    if (DL_GPIO_readPins(KEY2_PORT, KEY2_PIN) == 0) {
        delay_ms(10);
        if (DL_GPIO_readPins(KEY2_PORT, KEY2_PIN) == 0) {
            return 2;  /* KEY2按下 */
        }
    }
    
    return 0;
}

/* ============================================================================
 * UART发送/接收
 * ============================================================================ */
static void uart2_send(uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!DL_UART_isTXReady(x_bujin_INST));
        DL_UART_transmitData(x_bujin_INST, data[i]);
    }
}

static void uart3_send(uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!DL_UART_isTXReady(y_bujin_INST));
        DL_UART_transmitData(y_bujin_INST, data[i]);
    }
}

/* ============================================================================
 * PD42S1 驱动封装
 * ============================================================================ */
/**
 * @brief   发送命令到X轴驱动器
 */
static void pd42s1_x_send(uint8_t addr, uint8_t func, uint8_t *data, uint8_t len) {
    PD42S1_SendCommand(addr, func, data, len);
}

/**
 * @brief   发送命令到Y轴驱动器
 */
static void pd42s1_y_send(uint8_t addr, uint8_t func, uint8_t *data, uint8_t len) {
    PD42S1_SendCommand(addr, func, data, len);
}

/* ============================================================================
 * PD42S1 快捷控制函数
 * ============================================================================ */
/**
 * @brief   电机使能
 * @param   axis    0=X轴, 1=Y轴
 * @param   enable  true=使能, false=失能
 */
static void motor_enable(uint8_t axis, bool enable) {
    uint8_t data[1] = {enable ? 1 : 0};
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    
    if (axis == 0) {
        PD42S1_SendCommand(addr, PD42_FCT_MOTOR_ENABLE, data, 1);
    } else {
        PD42S1_SendCommand(addr, PD42_FCT_MOTOR_ENABLE, data, 1);
    }
}

/**
 * @brief   速度模式控制
 * @param   axis        0=X轴, 1=Y轴
 * @param   dir         0=顺时针, 1=逆时针
 * @param   speed_rpm   速度 (RPM)
 */
static void motor_set_speed(uint8_t axis, pd42_dir_t dir, float speed_rpm) {
    uint8_t data[6];
    uint32_t speed_int = (uint32_t)(speed_rpm * 100);
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    
    data[0] = dir;
    data[1] = 100;  /* 加速度 */
    data[2] = (uint8_t)(speed_int >> 24);
    data[3] = (uint8_t)(speed_int >> 16);
    data[4] = (uint8_t)(speed_int >> 8);
    data[5] = (uint8_t)(speed_int & 0xFF);
    
    PD42S1_SendCommand(addr, PD42_FCT_SPEED_MODE, data, 6);
}

/**
 * @brief   相对位置模式控制
 * @param   axis        0=X轴, 1=Y轴
 * @param   dir         0=顺时针, 1=逆时针
 * @param   speed       速度 (RPM)
 * @param   pulses      脉冲数 (相对位移)
 */
static void motor_move_rel(uint8_t axis, pd42_dir_t dir, uint16_t speed, uint32_t pulses) {
    uint8_t data[8];
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    
    data[0] = dir;
    data[1] = 100;  /* 加速度 */
    data[2] = (uint8_t)(speed >> 8);
    data[3] = (uint8_t)(speed & 0xFF);
    data[4] = (uint8_t)(pulses >> 24);
    data[5] = (uint8_t)(pulses >> 16);
    data[6] = (uint8_t)(pulses >> 8);
    data[7] = (uint8_t)(pulses & 0xFF);
    
    PD42S1_SendCommand(addr, PD42_FCT_REL_POS_MODE, data, 8);
}

/**
 * @brief   绝对位置模式控制
 * @param   axis        0=X轴, 1=Y轴
 * @param   dir         0=顺时针, 1=逆时针
 * @param   speed       速度 (RPM)
 * @param   target_pos  目标位置 (绝对脉冲数)
 */
static void motor_move_abs(uint8_t axis, pd42_dir_t dir, uint16_t speed, int32_t target_pos) {
    uint8_t data[8];
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    
    data[0] = dir;
    data[1] = 100;  /* 加速度 */
    data[2] = (uint8_t)(speed >> 8);
    data[3] = (uint8_t)(speed & 0xFF);
    data[4] = (uint8_t)(target_pos >> 24);
    data[5] = (uint8_t)(target_pos >> 16);
    data[6] = (uint8_t)(target_pos >> 8);
    data[7] = (uint8_t)(target_pos & 0xFF);
    
    PD42S1_SendCommand(addr, PD42_FCT_ABS_POS_MODE, data, 8);
}

/**
 * @brief   立即停止
 */
static void motor_stop(uint8_t axis) {
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    PD42S1_SendCommand(addr, PD42_FCT_STOP_IMMEDIATE, NULL, 0);
}

/**
 * @brief   位置清零
 */
static void motor_zero(uint8_t axis) {
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    PD42S1_SendCommand(addr, PD42_FCT_ZERO_ANGLE, NULL, 0);
}

/**
 * @brief   读取驱动器状态
 */
static void motor_read_status(uint8_t axis) {
    uint8_t addr = (axis == 0) ? PD42S1_X_ADDR : PD42S1_Y_ADDR;
    PD42S1_SendCommand(addr, PD42_FCT_READ_STATUS, NULL, 0);
}

/* ============================================================================
 * 示例演示函数
 * ============================================================================ */
/**
 * @brief   示例1: 电机使能测试
 */
static void demo_motor_enable(void) {
    /* 使能X轴和Y轴电机 */
    motor_enable(0, true);
    delay_ms(100);
    motor_enable(1, true);
    
    /* 等待稳定 */
    delay_ms(500);
    
    /* 失能 */
    motor_enable(0, false);
    motor_enable(1, false);
}

/**
 * @brief   示例2: 速度模式测试
 * @note    电机将以设定速度持续转动
 */
static void demo_speed_mode(void) {
    motor_enable(0, true);  /* 使能 */
    
    /* 设置速度: 100 RPM 顺时针 */
    motor_set_speed(0, PD42_DIR_CW, 100);
    
    delay_ms(2000);  /* 运行2秒 */
    
    /* 改变方向和速度 */
    motor_set_speed(0, PD42_DIR_CCW, 200);
    
    delay_ms(2000);  /* 运行2秒 */
    
    motor_stop(0);   /* 停止 */
    motor_enable(0, false);
}

/**
 * @brief   示例3: 相对位置控制测试
 * @note    电机移动指定的脉冲数
 */
static void demo_relative_move(void) {
    motor_enable(0, true);  /* 使能 */
    delay_ms(200);
    
    /* 相对移动: 顺时针移动10000个脉冲 (约25圈,假设1600细分) */
    motor_move_rel(0, PD42_DIR_CW, 1000, 10000);
    
    delay_ms(3000);  /* 等待运动完成 */
    
    /* 相对移动: 逆时针移动5000个脉冲 */
    motor_move_rel(0, PD42_DIR_CCW, 1000, 5000);
    
    delay_ms(2000);
    
    motor_enable(0, false);
}

/**
 * @brief   示例4: 绝对位置控制测试
 * @note    电机移动到绝对位置
 */
static void demo_absolute_move(void) {
    motor_enable(0, true);
    motor_enable(1, true);
    delay_ms(200);
    
    /* 先清零当前位置 */
    motor_zero(0);
    motor_zero(1);
    delay_ms(100);
    
    /* 移动到绝对位置 50000 */
    motor_move_abs(0, PD42_DIR_CW, 2000, 50000);
    motor_move_abs(1, PD42_DIR_CW, 2000, 30000);
    
    delay_ms(5000);
    
    /* 移动到原点 */
    motor_move_abs(0, PD42_DIR_CCW, 2000, 0);
    motor_move_abs(1, PD42_DIR_CCW, 2000, 0);
    
    delay_ms(5000);
    
    motor_enable(0, false);
    motor_enable(1, false);
}

/**
 * @brief   示例5: 往返运动测试
 * @note    电机在两点之间往复运动
 */
static void demo_来回运动(void) {
    motor_enable(0, true);
    delay_ms(200);
    motor_zero(0);
    delay_ms(100);
    
    for (uint8_t i = 0; i < 3; i++) {
        /* 正向移动 */
        motor_move_abs(0, PD42_DIR_CW, 1500, 20000);
        delay_ms(3000);
        
        /* 反向移动 */
        motor_move_abs(0, PD42_DIR_CCW, 1500, 0);
        delay_ms(3000);
    }
    
    motor_enable(0, false);
}

/* ============================================================================
 * 主函数
 * ============================================================================ */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();
    
    /* PD42S1 通信初始化 */
    PD42S1_Init(PD42S1_BAUD_RATE);
    
    /* 主循环 */
    while (1) {
        uint8_t key = key_scan();
        
        switch (key) {
            case 1:
                /* KEY1: 演示相对位置控制 */
                demo_relative_move();
                break;
                
            case 2:
                /* KEY2: 演示绝对位置控制 */
                demo_absolute_move();
                break;
                
            default:
                /* LED闪烁 */
                DL_GPIO_togglePins(LED_PORT, LED_PIN);
                delay_ms(500);
                break;
        }
    }
}

/* ============================================================================
 * 中断处理 (预留)
 * ============================================================================ */
/*
 * UART接收中断 - 用于处理驱动器应答
 * 可在SysConfig中启用UART中断，并在此处理应答数据
 */

/* UART2中断处理 (X轴) */
void x_bujin_INST_IRQHandler(void) {
    uint8_t rx_data;
    
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
        rx_data = DL_UART_receiveData(x_bujin_INST);
        PD42S1_UART_Callback(rx_data);
    }
}

/* UART3中断处理 (Y轴) */
void y_bujin_INST_IRQHandler(void) {
    uint8_t rx_data;
    
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX) {
        rx_data = DL_UART_receiveData(y_bujin_INST);
        /* Y轴回调处理 */
    }
}
