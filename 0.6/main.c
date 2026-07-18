/* ============================================================================
 * main.c
 * MSPM0G3507 工程入口: 系统初始化 + 主循环
 *
 *   上电序列:
 *     SYSCFG_DL_init → SysTick_Init → LCD_Init → 显示“初始化中...”
 *       → 其余模块 Init + 后台推进初始化到 3 秒 → X/Y 轴移动到启动绝对位置
 *       → 继续等待到 5 秒 → 进入主页 + 蜂鸣器/LED 完成提示
 *
 *   主循环每帧: 切页路由 (K5/K1/K2) → page on_key → SM_Tick → Huidu_Task
 *              → MPU9250_Task → UI_Render → 200ms [HB] 心跳
 *
 *   本文件不包含任何页的业务逻辑, 都在 user/UI/ui.c 里实现。
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
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

/* 上电时序: 0~3 秒初始化, 第 3 秒下发固定位置, 第 5 秒进入主页。 */
#define BOOT_INIT_WAIT_MS       3000U
#define BOOT_TOTAL_WAIT_MS      5000U
#define BOOT_CHIME_PULSE_MS      100U
#define BOOT_CHIME_GAP_MS        100U

static void show_boot_initializing(void) {
    LCD_ShowString(20U, ROW_Y(4), "初始化中...", WHITE, BLUE,
                   LCD_8X16, LCD_modeoff);
}

static void play_boot_chime(void) {
    for (uint8_t i = 0; i < 2U; i++) {
        Buzzer_On();
        LED_On();
        mspm0_delay_ms(BOOT_CHIME_PULSE_MS);
        Buzzer_Off();
        LED_Off();
        if (i < 1U) {
            mspm0_delay_ms(BOOT_CHIME_GAP_MS);
        }
    }
}

#define SM_BOOT_PULSES_PER_REV  51200L
#define SM_BOOT_X_DEG           183L
#define SM_BOOT_Y_DEG           120L
#define SM_BOOT_SPEED_RPM       15U
#define SM_BOOT_ACCEL           20U

static int32_t boot_degrees_to_pulses(int32_t degrees) {
    return (int32_t)(((int64_t)degrees * SM_BOOT_PULSES_PER_REV + 180LL) / 360LL);
}

static void service_startup_tasks(void) {
    SM_Tick();
    Huidu_Task();
    MPU9250_Task();
}

static void wait_until_ms(uint32_t deadline_ms) {
    while ((int32_t)((uint32_t)tick_ms - deadline_ms) < 0) {
        service_startup_tasks();
    }
}

static void wait_for_sm_command_queue(void) {
    while (!SM_CommandQueueIdle()) {
        service_startup_tasks();
    }
}

static void move_steppers_to_boot_positions(void) {
    wait_for_sm_command_queue();

    SM_MoveTo(SM_X, R, SM_BOOT_ACCEL, SM_BOOT_SPEED_RPM,
              boot_degrees_to_pulses(SM_BOOT_X_DEG));
    wait_for_sm_command_queue();

    SM_MoveTo(SM_Y, R, SM_BOOT_ACCEL, SM_BOOT_SPEED_RPM,
              boot_degrees_to_pulses(SM_BOOT_Y_DEG));
    wait_for_sm_command_queue();
}

/* K5 在详情页按下时, 切回菜单前停掉可能还在转的电机 (防按住 K1/K2 切页丢状态) */
static void stop_actuators_on_exit_page(void) {
    /* main 页 [1] 不需要 stop; pages[] 里 X→[7], Y→[8] */
    if (g_page == 7U)      SM_Stop(SM_X);
    else if (g_page == 8U) SM_Stop(SM_Y);
    TB6612_Stop(TB_MOTOR_L);
    TB6612_Stop(TB_MOTOR_R);
}

/* 主循环开头: K5 切页路由 (菜单页进, 详情页出)
 *   K1/K2 只在菜单页切选中项; 详情页 K1~K4 由 page on_key 接管 */
static void handle_navigation_keys(void) {
    if (key(5, down)) {
        if (g_page == 0U) g_page = g_menu_sel;
        else              { stop_actuators_on_exit_page(); g_page = 0U; }
        UI_ForceRedraw();
        return;
    }
    if (g_page != 0U) return;

    if (key(1, down)) {
        g_menu_sel = (g_menu_sel <= 1U) ? (uint8_t)MENU_ITEM_COUNT
                                        : (uint8_t)(g_menu_sel - 1U);
        UI_ForceRedraw();
    }
    if (key(2, down)) {
        g_menu_sel = (g_menu_sel >= MENU_ITEM_COUNT) ? (uint8_t)1U
                                                    : (uint8_t)(g_menu_sel + 1U);
        UI_ForceRedraw();
    }
}

int main(void) {
    SYSCFG_DL_init();
    SysTick_Init();
    uint32_t boot_start_ms = (uint32_t)tick_ms;

    /* 先让提示输出进入安全态，再优先启动 LCD。 */
    Buzzer_Init();
    LED_Init();
    KEY_Init();
    LCD_Init(BLUE);
    show_boot_initializing();

    /* SysConfig 已知只 SetPriority 不 Enable (见 AGENTS.md §10), 这里显式 enable */
    NVIC_EnableIRQ(x_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(BIANMA2_TIM_INST_INT_IRQN);
    NVIC_EnableIRQ(MPU9250_TIM_INST_INT_IRQN);
    UART_Host_Init();

    PD42S1_Init(PD42S1_BAUD_RATE);
    SM_Init();
    {
        /* 灰度标定: 默认全 0, 二值化阈值会塌掉但 ADC 原始值仍可读 */
        uint16_t w[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
        uint16_t b[8] = { 500,  500,  500,  500,  500,  500,  500,  500 };
        Huidu_Init(w, b);
    }
    TB6612_Init();
    Encoder_Init();
    MPU9250_Init();

    /* 前 3 秒持续推进有时序要求的模块；到点后才允许步进电机移动。 */
    wait_until_ms(boot_start_ms + BOOT_INIT_WAIT_MS);
    move_steppers_to_boot_positions();

    /* 启动提示保持到第 5 秒，然后进入主页并给出可闻可视完成提示。 */
    wait_until_ms(boot_start_ms + BOOT_TOTAL_WAIT_MS);
    UI_Init();
    UI_Render();
    play_boot_chime();

    /* 心跳节流: 200ms 打 [HB] (串口监视用) */
    uint32_t last_hb_ms = 0;
    /* X/Y 各 300ms 错开读位置 (用户 2026-07-18 决定: 不要延时, 但两轴不要同时刷).
     *   next_read_ms[0] = X 起始时间 (now+0), next_read_ms[1] = Y 起始时间 (now+300)
     *   节流器自身 15ms 间隔保证两帧不会背靠背, 不需要 mspm0_delay_ms(). */
    uint32_t next_read_ms[2] = {
        (uint32_t)tick_ms + 100U,
        (uint32_t)tick_ms + 100U + 300U,
    };
    uint32_t loop_count = 0;

    while (1) {
        loop_count++;
        handle_navigation_keys();
        pages[g_page].on_key();
        SM_Tick();
        Huidu_Task();
        MPU9250_Task();

        /* K230 命令处理 (坐标数据) */
        if (UART1_RxFlag) {
            UART1_RxFlag = 0;
            K230_ParseCommand(UART1_RxPacket);
        }

        /* X / Y 各 300ms 错开一次读位置 (主页 ROW 2 显示, 2026-07-18 决定:
         *   每轴 600ms 读一次, X 与 Y 错开 300ms, 不阻塞主循环)
         *   节流器自动 15ms 间隔保证两帧不背靠背, 不需要 mspm0_delay_ms().
         *   uint32_t 时间差比较 (49.7 天 wrap 一次, 量级上不会出现):
         *     (int32_t)(now - next) >= 0 = 已到点 (含相等) */
        if ((int32_t)(tick_ms - next_read_ms[0]) >= 0) {
            SM_ReadPosition(SM_X);
            next_read_ms[0] = (uint32_t)tick_ms + 300U;
        }
        if ((int32_t)(tick_ms - next_read_ms[1]) >= 0) {
            SM_ReadPosition(SM_Y);
            next_read_ms[1] = (uint32_t)tick_ms + 300U;
        }

        UI_Render();
        /* LED toggle: 硬件指示主循环在转 (无需串口) */
        if (loop_count % 100000U == 0U) {
            LED_Toggle();
        }

        /* 200ms 心跳 */
        if ((tick_ms - last_hb_ms) >= 200U) {
            last_hb_ms = tick_ms;
            UART_Printf(UART_CH0, "[HB] T=%lu page=%u loop=%u\r\n",
                        (uint32_t)tick_ms, (unsigned)g_page, (unsigned)(loop_count & 0xFFFFFF));
        }
    }
}

/* ============================================================================
 * UART 中断
 * ============================================================================ */
void x_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_CallbackFor((uint8_t)SM_X, DL_UART_receiveData(x_bujin_INST));
    }
}

void y_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_CallbackFor((uint8_t)SM_Y, DL_UART_receiveData(y_bujin_INST));
    }
}

/* UART_0 上位机 RX 中断 → UART_Host 状态机收包 */
void UART_0_INST_IRQHandler(void) {
    if (DL_UART_Main_getEnabledInterruptStatus(UART_0_INST,
            DL_UART_MAIN_INTERRUPT_RX)) {
        UART0_RxCallback(DL_UART_Main_receiveData(UART_0_INST));
        DL_UART_Main_clearInterruptStatus(UART_0_INST,
            DL_UART_MAIN_INTERRUPT_RX);
    }
}

/* K230 UART1 RX 中断 → UART_Host 状态机收包 */
void K230_INST_IRQHandler(void) {
    if (DL_UART_Main_getEnabledInterruptStatus(K230_INST,
            DL_UART_MAIN_INTERRUPT_RX)) {
        UART1_RxCallback(DL_UART_Main_receiveData(K230_INST));
        DL_UART_Main_clearInterruptStatus(K230_INST,
            DL_UART_MAIN_INTERRUPT_RX);
    }
}

/* ============================================================================
 * BIANMA2_TIM ISR (50us, 20kHz) — 编码器 4 倍频查表
 *   一进中断两路编码器都查一次, 避免 2 个定时器各跑一个 ISR 的开销
 * ============================================================================ */
void BIANMA2_TIM_INST_IRQHandler(void) {
    switch (DL_TimerA_getPendingInterrupt(BIANMA2_TIM_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Encoder_Update();
            break;
        default:
            break;
    }
}
