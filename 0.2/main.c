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
 *      - y_bujin_INST_INT_IRQN           UART3 PD42S1 Y 轴 RX (当前 callback 空)
 *      - BIANMA2_TIM_INST_INT_IRQN       TIMA1 50us ISR 推进 Encoder_Update
 *   4. 各模块 Init (顺序无关):
 *        LED_Init() / KEY_Init()
 *        PD42S1_Init(PD42S1_BAUD_RATE)
 *        SM_Init()                       X 轴闭环步进 (发 4 帧初始化序列)
 *        Huidu_Init(w, b)                灰度标定
 *        TB6612_Init()                   编码电机刹车态
 *        Encoder_Init()                  编码器查表 + 启动 TIMA1
 *        LCD_Init(BLUE)                  1.8寸 TFT 清屏蓝色
 *        UI_Init()                       LCD UI 注册表
 *
 *   while(1) 主循环:
 *     - K5 down → 推进 g_page (末页回 0) + SM_Stop + TB6612_Stop×2 + UI_ForceRedraw
 *     - pages[g_page].on_key()           K1~K4 派发到当前页
 *     - SM_Tick()                        推进 PD42S1 节流队列 (10ms 间隔发帧)
 *     - Huidu_Task()                     MUX_ADC 模式每帧扫一轮 8 路
 *     - UI_Render()                      按内容 hash 刷 LCD (无变化行零开销)
 *
 * ============================================================================
 * 中断入口 (本文件实现, callback 转发到对应驱动)
 * ============================================================================
 *   x_bujin_INST_IRQHandler → PD42S1_UART_Callback(rx_data)
 *   y_bujin_INST_IRQHandler → (void)rx_data  (Y 轴暂未驱动)
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
    LCD_Init(BLUE);

    UI_Init();    /* g_page = 0 (默认空页), pages 表编译期已注册 */

    while (1) {
        /* K5 做页面切换 (每页通用, 不下放到 page on_key):
         *   - down 事件推进一页; 末页按下回 0
         *   - 切页前 SM_Stop: 防电机在 K1/K2 按住状态下被带过去 (即使新页无 on_key) */
        if (key(5, down)) {
            SM_Stop(SM_X);
            TB6612_Stop(TB_MOTOR_L);    /* 顺手把编码电机也停了 (从 TB6612 页切走时) */
            TB6612_Stop(TB_MOTOR_R);
            g_page = (g_page + 1U) % PAGE_COUNT;
            UI_ForceRedraw();
        }

        /* K1~K4 全部派发到当前页 on_key (空页 on_key 是空函数, 自动忽略) */
        pages[g_page].on_key();

        SM_Tick();
        Huidu_Task();        /* 每主循环采一轮 8 路灰度 (MUX_ADC 模式 ~80us) */
        /* Encoder_Update 由 BIANMA2_TIM ISR (50us) 推进, 不在主循环调 */
        UI_Render();
    }
}

/* ============================================================================
 * UART 中断
 * ============================================================================ */
void x_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_Callback(DL_UART_receiveData(x_bujin_INST));
    }
}

void y_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX) {
        (void)DL_UART_receiveData(y_bujin_INST);
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
