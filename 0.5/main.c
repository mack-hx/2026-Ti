/* ============================================================================
 * main.c - MSPM0G3507 工程入口
 * ============================================================================
 *
 * ============================================================================
 * 模块命名含义:
 *   main = 主函数/主循环
 *   BOOT_* = 启动时序常量
 *   wait_until = 等待直到某个时刻
 *   tick_all = 推进所有模块 tick
 *
 * ============================================================================
 * 上电序列:
 *   [1] SYSCFG_DL_init → SysTick_Init → LCD_Init
 *   [2] 各模块 Init (Buzzer/LED/KEY/PD42S1/SM/Huidu/TB6612/Encoder/MPU9250)
 *   [3] 3s wait → X/Y 轴移到启动位置
 *   [4] 5s wait → 进入主页 + 蜂鸣器/LED 提示
 *
 * ============================================================================
 * 主循环每帧 (每帧约几十微秒):
 *   [1] 按键路由 (K5 切页 / K1/K2 菜单选中等)
 *   [2] page on_key (K1~K4 业务逻辑)
 *   [3] 业务 Tick (SM_Tick / Huidu_Task / MPU9250_Task)
 *   [4] K230 坐标接收 (UART1)
 *   [5] X/Y 位置读 (错开 300ms)
 *   [6] UI 渲染
 *   [7] LED 心跳 (每 10 万次翻转)
 *   [8] 200ms 心跳日志
 *
 * ============================================================================
 */

/* ==================== 头文件 ==================== */
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/LED/led.h"
#include "Hardware/Buzzer/buzzer.h"
#include "Hardware/Huidu/huidu.h"
#include "Hardware/TB6612/tb6612.h"
#include "Hardware/Encoder/encoder.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "Hardware/UART_Host/uart_host.h"
#include "user/UI/ui.h"
#include "LCD.h"
#include "system/clock.h"

/* ==================== 启动时序常量 ==================== */
#define BOOT_INIT_WAIT_MS     3000U  /* 前 3s 初始化 */
#define BOOT_TOTAL_WAIT_MS    5000U  /* 第 5s 进入主页 */
#define BOOT_CHIME_PULSE_MS   100U  /* 蜂鸣器脉冲宽度 */
#define BOOT_CHIME_GAP_MS     100U  /* 两声之间的间隔 */

/* ==================== 启动位置参数 ==================== */
#define SM_BOOT_X_DEG   183L   /* X 轴启动角度 (度) */
#define SM_BOOT_Y_DEG   120L   /* Y 轴启动角度 (度) */
#define SM_BOOT_SPEED   15U    /* RPM */
#define SM_BOOT_ACCEL   20U
#define SM_PULSES_PER_REV  51200L

/**
 * @brief   deg_to_pulses - 度 → 脉冲换算
 * @param   deg  角度 (度)
 * @return  对应的脉冲数 (四舍五入)
 */
static int32_t deg_to_pulses(int32_t deg) {
    return (int32_t)(((int64_t)deg * SM_PULSES_PER_REV + 180LL) / 360LL);
}

/* ==================== 启动提示 ==================== */

/**
 * @brief   show_boot_initializing - 显示初始化中
 */
static void show_boot_initializing(void) {
    LCD_ShowString(20U, ROW_Y(4), "Starting...", WHITE, BLUE, LCD_8X16, LCD_modeoff);
}

/**
 * @brief   play_boot_chime - 蜂鸣器 + LED 两声提示
 */
static void play_boot_chime(void) {
    for (uint8_t i = 0U; i < 2U; i++) {
        Buzzer_On();
        LED_On();
        mspm0_delay_ms(BOOT_CHIME_PULSE_MS);
        Buzzer_Off();
        LED_Off();
        if (i < 1U) mspm0_delay_ms(BOOT_CHIME_GAP_MS);
    }
}

/* ==================== 启动等待辅助 ==================== */

/**
 * @brief   tick_all - 推进所有模块 tick (等待期间持续调用)
 */
static void tick_all(void) {
    SM_Tick();
    Huidu_Task();
    MPU9250_Task();
}

/**
 * @brief   wait_until - 等待直到 tick_ms >= deadline_ms
 * @param   deadline_ms  截止时刻 (绝对时间戳, 由 boot_start + 延时计算得到)
 *
 * 注意: tick_ms 不是从 0 开始, 而是 main() 入口时芯片已运行了一段时间.
 * 所以必须用 "当前时刻 + 延时" 的方式计算截止时刻, 而不是直接用 "延时值".
 */
static void wait_until(uint32_t deadline_ms) {
    while (tick_ms < deadline_ms) tick_all();
}

/**
 * @brief   wait_queue_idle - 等待命令队列空闲
 */
static void wait_queue_idle(void) {
    while (!SM_CommandQueueIdle()) tick_all();
}

/**
 * @brief   move_to_start_pos - 启动时移动到初始位置
 */
static void move_to_start_pos(void) {
    wait_queue_idle();
    SM_MoveTo(SM_X, R, SM_BOOT_ACCEL, SM_BOOT_SPEED, deg_to_pulses(SM_BOOT_X_DEG));
    wait_queue_idle();
    SM_MoveTo(SM_Y, R, SM_BOOT_ACCEL, SM_BOOT_SPEED, deg_to_pulses(SM_BOOT_Y_DEG));
    wait_queue_idle();
}

/* ==================== 主循环: 按键路由 ==================== */

/**
 * @brief   handle_navigation_keys - 处理导航按键
 *
 * K5: 菜单页 → 进入选中页 / 详情页 → 返回菜单
 * K1/K2: 仅在菜单页切换选中项
 */
static void handle_navigation_keys(void) {
    if (key(5, down)) {
        if (g_page == 0U) {
            g_page = g_menu_sel;
        } else {
            g_page = 0U;
        }
        UI_ForceRedraw();
        return;
    }
    if (g_page != 0U) return;  /* 详情页 K1/K2 由 page on_key 处理 */

    if (key(1, down)) {
        g_menu_sel = (g_menu_sel <= 1U) ? (uint8_t)MENU_ITEM_COUNT : (uint8_t)(g_menu_sel - 1U);
        UI_ForceRedraw();
    }
    if (key(2, down)) {
        g_menu_sel = (g_menu_sel >= MENU_ITEM_COUNT) ? (uint8_t)1U : (uint8_t)(g_menu_sel + 1U);
        UI_ForceRedraw();
    }
}

/* ==================== 主函数 ==================== */
int main(void) {
    SYSCFG_DL_init();
    SysTick_Init();
    uint32_t boot_start = (uint32_t)tick_ms;

    /* 基础外设初始化 */
    Buzzer_Init();
    LED_Init();
    KEY_Init();
    LCD_Init(BLUE);
    show_boot_initializing();

    /* SysConfig 只 SetPriority, 需要手动 Enable 中断 */
    NVIC_EnableIRQ(x_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(BIANMA2_TIM_INST_INT_IRQN);
    NVIC_EnableIRQ(MPU9250_TIM_INST_INT_IRQN);

    UART_Host_Init();
    PD42S1_Init(PD42S1_BAUD_RATE);
    SM_Init();

    /* 灰度标定 (默认阈值) */
    uint16_t w[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
    uint16_t b[8] = { 500,  500,  500,  500,  500,  500,  500,  500};
    Huidu_Init(w, b);

    TB6612_Init();
    Encoder_Init();
    MPU9250_Init();

    /* 等待 3s 后移动到启动位置 */
    wait_until(boot_start + BOOT_INIT_WAIT_MS);
    move_to_start_pos();

    /* 等待 5s 后进入主页 */
    wait_until(boot_start + BOOT_TOTAL_WAIT_MS);
    UI_Init();
    UI_Render();
    play_boot_chime();

    /* ==================== 主循环 ==================== */
    uint32_t next_read_ms[2] = { tick_ms + 100U, tick_ms + 400U };  /* X/Y 错开 300ms */
    uint32_t loop_count = 0;

    while (1) {
        loop_count++;

        /* [1] 按键路由 */
        handle_navigation_keys();
        pages[g_page].on_key();

        /* [2] 业务 Tick */
        SM_Tick();
        Huidu_Task();
        MPU9250_Task();

        /* [3] K230 坐标接收 */
        if (UART1_RxFlag) {
            UART1_RxFlag = 0;
            K230_ParseCommand(UART1_RxPacket);
        }

        /* [4] X/Y 位置读 (各 300ms, 错开) */
        if (tick_ms >= next_read_ms[0]) {
            SM_ReadPosition(SM_X);
            next_read_ms[0] = tick_ms + 300U;
        }
        if (tick_ms >= next_read_ms[1]) {
            SM_ReadPosition(SM_Y);
            next_read_ms[1] = tick_ms + 300U;
        }

        /* [5] UI 渲染 */
        UI_Render();
    }
}

/* ============================================================================
 * UART 中断处理
 * ============================================================================ */

/* x_bujin_INST: PD42S1 X 轴 UART2 (PB15/PB16) */
void x_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_CallbackFor((uint8_t)SM_X, DL_UART_receiveData(x_bujin_INST));
    }
}

/* y_bujin_INST: PD42S1 Y 轴 UART3 (PB2/PB3) */
void y_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_CallbackFor((uint8_t)SM_Y, DL_UART_receiveData(y_bujin_INST));
    }
}

/* UART_0_INST: 上位机 UART (PA10/PA11) */
void UART_0_INST_IRQHandler(void) {
    if (DL_UART_Main_getEnabledInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX)) {
        UART0_RxCallback(DL_UART_Main_receiveData(UART_0_INST));
        DL_UART_Main_clearInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    }
}

/* K230_INST: K230 UART1 (用于接收 K230 坐标) */
void K230_INST_IRQHandler(void) {
    if (DL_UART_Main_getEnabledInterruptStatus(K230_INST, DL_UART_MAIN_INTERRUPT_RX)) {
        UART1_RxCallback(DL_UART_Main_receiveData(K230_INST));
        DL_UART_Main_clearInterruptStatus(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    }
}

/* BIANMA2_TIM_INST: 编码器定时器 (50us, 20kHz) */
void BIANMA2_TIM_INST_IRQHandler(void) {
    if (DL_TimerA_getPendingInterrupt(BIANMA2_TIM_INST) == DL_TIMER_IIDX_ZERO) {
        Encoder_Update();
    }
}
