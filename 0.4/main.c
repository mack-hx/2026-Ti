/* ============================================================================
 * @file    main.c
 * @brief   MSPM0G3507 工程入口 - 系统初始化 + 主循环
 *
 * ============================================================================
 * 调用流程 (从上电到主循环)
 * ============================================================================
 *
 *   1. SYSCFG_DL_init()                  SysConfig 生成的时钟/GPIO/UART/ADC 初始化
 *   2. SysTick_Init()                    1ms tick 给 clock.c (按键扫描用)
 *   3. NVIC_EnableIRQ(...) × 4           手动 enable 4 个中断
 *      - x_bujin_INST_INT_IRQN           UART2 PD42S1 X 轴 RX
 *      - y_bujin_INST_INT_IRQN           UART3 PD42S1 Y 轴 RX
 *      - BIANMA2_TIM_INST_INT_IRQN       TIMA1 50us ISR 推进 Encoder_Update
 *   4. 各模块 Init (顺序无关):
 *        LED_Init() / KEY_Init()
 *        PD42S1_Init(PD42S1_BAUD_RATE)
 *        SM_Init()                       X/Y 两轴使能 + 通信位置模式（不清零）
 *        Huidu_Init(w, b)                灰度标定
 *        TB6612_Init()                   编码电机刹车态
 *        Encoder_Init()                  编码器查表 + 启动 TIMA1
 *        LCD_Init(BLUE)                  1.8寸 TFT 清屏蓝色
 *        UI_Init()                       LCD UI 注册表
 *
 *   while(1) 主循环:
 *     - 菜单页 (g_page=0):
 *         K1 down → g_menu_sel 向上循环 (菜单里选中项上移)
 *         K2 down → g_menu_sel 向下循环
 *         K5 down → 进入选中详情页 (g_page = g_menu_sel)
 *     - 详情页 (g_page>0):
 *         K5 down → 返回菜单页 (g_page = 0) + SM_Stop + TB6612_Stop×2 + UI_ForceRedraw
 *     - 任意页: pages[g_page].on_key()  K1~K4 派发 (菜单页 on_key 是 no-op)
 *     - SM_Tick()                        推进 PD42S1 节流队列 (10ms 间隔发帧)
 *     - Huidu_Task()                     MUX_ADC 模式每帧扫一轮 8 路
 *     - UI_Render()                      按内容 hash 刷 LCD (无变化行零开销)
 *
 * ============================================================================
 * 中断入口 (本文件实现, callback 转发到对应驱动)
 * ============================================================================
 *   x_bujin_INST_IRQHandler → PD42S1_UART_CallbackFor(SM_X, rx_data)
 *   y_bujin_INST_IRQHandler → PD42S1_UART_CallbackFor(SM_Y, rx_data)
 *   BIANMA2_TIM_INST_IRQHandler → Encoder_Update() (50us 周期, 20kHz)
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/LED/led.h"
#include "Hardware/Huidu/huidu.h"
#include "Hardware/TB6612/tb6612.h"
#include "Hardware/Encoder/encoder.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "user/UI/ui.h"
#include "LCD.h"
#include "system/clock.h"

int main(void) {
    SYSCFG_DL_init();
    SysTick_Init();
    NVIC_EnableIRQ(x_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);
    /* BIANMA2_TIM (TIMA1) 挂编码器 4 倍频查表, 50us = 20kHz 采样 (用户 02:55 反馈:
     * "我专门设了两个定时器" — BIANMA1_TIM 留给 TMC2209, BIANMA2_TIM 专给编码器) */
    NVIC_EnableIRQ(BIANMA2_TIM_INST_INT_IRQN);
    /* MPU9250_TIM (TIMG6, 5ms) 推进 MPU9250 9 轴 I2C 状态机,
     * 让 I2C 阻塞不进主循环, UI / 按键 / 编码器不被卡死 (用户 03:00 反馈) */
    NVIC_EnableIRQ(MPU9250_TIM_INST_INT_IRQN);

    LED_Init();
    KEY_Init();
    PD42S1_Init(PD42S1_BAUD_RATE);
    SM_Init();
    {
        /* 灰度标定: 默认全 0, 二值化阈值会塌掉但 ADC 原始值仍可读 */
        uint16_t w[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
        uint16_t b[8] = {500,  500,  500,  500,  500,  500,  500,  500 };
        Huidu_Init(w, b);
    }
    TB6612_Init();          /* TB6612 上电全刹车, PWM=0, 档=[20, 20], selected=L */
    Encoder_Init();         /* 编码器 4 倍频状态机就绪 + 启动 BIANMA2_TIM (50us ISR) */
    MPU9250_Init();         /* 9 轴 IMU I2C0 上电 (失败也行, UI 显示 'no dev') */
    LCD_Init(BLUE);

    UI_Init();    /* g_page = 0 (默认空页), pages 表编译期已注册 */

    while (1) {
        /* K5 路由: 详情页 → 菜单页; 菜单页 → 进入选中项
         *   详情页 K5: 防电机在 K1/K2 按住状态下被带过去 → 切前 SM_Stop + TB6612_Stop×2
         *   菜单页 K5: 直接进 g_menu_sel, 不需要停电机 (菜单页 K1/K2 也不会动电机)
         *
         * 注意: 菜单页 K1/K2 是切换选中项 (g_menu_sel), 不动 g_page; 这部分也放这里集中处理
         *   跟详情页的 K1/K4 业务 (page on_key 派发) 不冲突
         */
        if (key(5, down)) {
            if (g_page == 0U) {
                /* 菜单页 K5 → 进入选中详情页 (g_menu_sel 范围 1..MENU_ITEM_COUNT) */
                g_page = g_menu_sel;
            } else {
                /* 详情页 K5 → 返回菜单；只停止当前步进页对应的轴。 */
                if (g_page == 6U) {
                    SM_Stop(SM_X);
                } else if (g_page == 7U) {
                    SM_Stop(SM_Y);
                }
                TB6612_Stop(TB_MOTOR_L);
                TB6612_Stop(TB_MOTOR_R);
                g_page = 0U;
            }
            UI_ForceRedraw();
        }

        /* 菜单页 K1/K2: 切换选中项 (循环 1 ↔ MENU_ITEM_COUNT)
         *   只在 g_page==0 时生效; 详情页 K1/K2/K3/K4 由 page on_key 接管 (下面派发)
         */
        if (g_page == 0U) {
            if (key(1, down)) {
                g_menu_sel = (g_menu_sel <= 1U)
                    ? (uint8_t)MENU_ITEM_COUNT
                    : (uint8_t)(g_menu_sel - 1U);
                UI_ForceRedraw();
            }
            if (key(2, down)) {
                g_menu_sel = (g_menu_sel >= MENU_ITEM_COUNT)
                    ? (uint8_t)1U
                    : (uint8_t)(g_menu_sel + 1U);
                UI_ForceRedraw();
            }
        }

        /* K1~K4 全部派发到当前页 on_key (菜单页 on_key 是 no-op, 不重复处理) */
        pages[g_page].on_key();

        SM_Tick();
        Huidu_Task();        /* 每主循环采一轮 8 路灰度 (MUX_ADC 模式 ~80us) */
        MPU9250_Task();      /* 9 轴 IMU 阻塞 I2C 事务 (~3ms 一帧, 节流) */
        /* Encoder_Update 由 BIANMA2_TIM ISR (50us) 推进, 不在主循环调 */
        UI_Render();
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
