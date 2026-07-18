/* ============================================================================
 * @file    tmc2209.c
 * @brief   TMC2209 stepper motor driver - low-level GPIO STEP/DIR + Timer ISR pulse
 * @note    Coexists with PD42S1 (shares UART TX/RX pins, pick one at a time)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   启用 TMC2209 之前 (用户需在 syscfg 里手动改):
 *     1) 禁用 x_bujin/y_bujin UART 模块 (避免 PD42S1 占用这些引脚)
 *     2) 把 BIANMA1_TIM (TIMG7) 改为高速 Timer (推荐 50us / 20kHz 周期),
 *        或新建一个 Timer 并修改本文件中的 TMC_TIMER_INST 宏
 *
 *   上电 (main() 启动序列里调一次):
 *     TMC2209_Init();                    // 重配 TX->STEP / RX->DIR + 启动 Timer ISR
 *
 *   while(1):
 *     TMC2209_SetDir(1, 'r');            // X 轴设方向 (正转)
 *     TMC2209_Step(1, 200);              // X 轴走 200 步 (非阻塞, ISR 推进)
 *     TMC2209_Step(2, 100);              // Y 轴走 100 步
 *     TMC2209_Stop();                    // 两路立即停 (清剩余计数 + STEP 拉低)
 *     uint32_t rem = TMC2209_GetRemaining(1);   // X 轴剩余步数 (判断到位)
 *
 * ============================================================================
 * 引脚配置 (重映射)
 *   SysConfig 中 x_bujin (UART2) 和 y_bujin (UART3) 的 TX/RX 引脚, 本驱动
 *   会重配为 GPIO 输出, 分别作为:
 *     X 轴: STEP = PB15 (原 x_bujin TX), DIR = PB16 (原 x_bujin RX)
 *     Y 轴: STEP = PB2  (原 y_bujin TX), DIR = PB3  (原 y_bujin RX)
 *   EN 引脚: 硬件未接, 默认使能 (软件不控制)
 *
 * ============================================================================
 * STEP 脉冲产生 (Timer ISR)
 *   复用 syscfg 中已有的 BIANMA1_TIM (TIMG7), 当前 syscfg 配 50ms 周期.
 *   NOTE: 这个周期对 STEP 脉冲太慢 (50ms = 20Hz), 实际使用 TMC2209 时需要在
 *         syscfg 中把 BIANMA1_TIM 改成高速 Timer, 推荐:
 *           - clockDivider = 1, prescaler = 0, period = 50us (20kHz, 用于细分脉冲)
 *         或者新增一个 Timer, 然后改本文件中 TMC_TIMER_INST 宏.
 *
 *   分频策略 (移植自 v1.21): 每次 Timer 中断累加 tick, 达到 divider 才发一次
 *   STEP 脉冲. divider 默认 16, 即 20kHz / 16 ~= 1.25kHz 步频.
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/TMC2209/tmc2209.h"

/* 选择用于 STEP 脉冲的 Timer.
 * 默认复用 BIANMA1_TIM (TIMG7). 用户启用 TMC2209 时务必在 syscfg 中
 * 把它改成高速周期 (推荐 50us). */
#define TMC_TIMER_INST             BIANMA1_TIM_INST
#define TMC_TIMER_IRQN             BIANMA1_TIM_INST_INT_IRQN
#define TMC_TIMER_IRQ_HANDLER      BIANMA1_TIM_INST_IRQHandler

/* 分频器: Timer ISR 频率 / divider = 实际步频
 * 默认 divider=16, 若 Timer 配 20kHz -> 1.25k 步/秒 ~= 1.25k pps */
static volatile uint16_t g_tmc_divider  = 16;
static volatile uint16_t g_tmc_tick_cnt = 0;

/* 剩余步数 (volatile: ISR 与主循环跨域访问) */
volatile uint32_t g_tmc_motor1_remaining = 0;
volatile uint32_t g_tmc_motor2_remaining = 0;

/* STEP 引脚最小高电平延时 (TMC2209 要求 > 1us) */
static inline void step_high_hold_short(void) {
    volatile uint32_t n = 200;   /* 80MHz 下约 3us, 满足 >1us 要求 */
    while (n--) { __NOP(); }
}

/* 重配 X 轴: PB15 (TX) -> STEP, PB16 (RX) -> DIR */
static void reconfig_motor1_pins(void) {
    /* TX 引脚: x_bujin_TX_PORT (GPIOB), pin = GPIO_x_bujin_TX_PIN = DL_GPIO_PIN_15 */
    DL_GPIO_initDigitalOutput(GPIO_x_bujin_IOMUX_TX);
    DL_GPIO_enableOutput(GPIO_x_bujin_TX_PORT, GPIO_x_bujin_TX_PIN);
    DL_GPIO_clearPins(GPIO_x_bujin_TX_PORT, GPIO_x_bujin_TX_PIN);

    /* RX 引脚: x_bujin_RX_PORT (GPIOB), pin = GPIO_x_bujin_RX_PIN = DL_GPIO_PIN_16 */
    DL_GPIO_initDigitalOutput(GPIO_x_bujin_IOMUX_RX);
    DL_GPIO_enableOutput(GPIO_x_bujin_RX_PORT, GPIO_x_bujin_RX_PIN);
    DL_GPIO_clearPins(GPIO_x_bujin_RX_PORT, GPIO_x_bujin_RX_PIN);
}

/* 重配 Y 轴: PB2 (TX) -> STEP, PB3 (RX) -> DIR */
static void reconfig_motor2_pins(void) {
    DL_GPIO_initDigitalOutput(GPIO_y_bujin_IOMUX_TX);
    DL_GPIO_enableOutput(GPIO_y_bujin_TX_PORT, GPIO_y_bujin_TX_PIN);
    DL_GPIO_clearPins(GPIO_y_bujin_TX_PORT, GPIO_y_bujin_TX_PIN);

    DL_GPIO_initDigitalOutput(GPIO_y_bujin_IOMUX_RX);
    DL_GPIO_enableOutput(GPIO_y_bujin_RX_PORT, GPIO_y_bujin_RX_PIN);
    DL_GPIO_clearPins(GPIO_y_bujin_RX_PORT, GPIO_y_bujin_RX_PIN);
}

/* ============================================================================
 * API
 * ============================================================================ */
void TMC2209_Init(void) {
    /* 1. 重配引脚: TX->STEP(输出), RX->DIR(输出) */
    reconfig_motor1_pins();
    reconfig_motor2_pins();

    /* 2. 默认方向 = 正转 (高电平), STEP 拉低 */
    DL_GPIO_setPins(GPIO_x_bujin_RX_PORT, GPIO_x_bujin_RX_PIN);
    DL_GPIO_setPins(GPIO_y_bujin_RX_PORT, GPIO_y_bujin_RX_PIN);

    /* 3. 清剩余计数 */
    g_tmc_motor1_remaining = 0;
    g_tmc_motor2_remaining = 0;
    g_tmc_tick_cnt = 0;

    /* 4. 启动 STEP 脉冲 Timer ISR */
    DL_TimerG_enableInterrupt(TMC_TIMER_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerG_clearInterruptStatus(TMC_TIMER_INST, DL_TIMER_IIDX_ZERO);
    NVIC_EnableIRQ(TMC_TIMER_IRQN);
    DL_TimerG_startCounter(TMC_TIMER_INST);

    /* EN 默认使能: 不做任何 GPIO 操作, 由外部硬件保证 */
}

void TMC2209_SetDir(uint8_t motor, char dir) {
    if (motor == 1) {
        if (dir == 'r' || dir == 'R') {
            DL_GPIO_setPins(GPIO_x_bujin_RX_PORT, GPIO_x_bujin_RX_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_x_bujin_RX_PORT, GPIO_x_bujin_RX_PIN);
        }
    } else {
        if (dir == 'r' || dir == 'R') {
            DL_GPIO_setPins(GPIO_y_bujin_RX_PORT, GPIO_y_bujin_RX_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_y_bujin_RX_PORT, GPIO_y_bujin_RX_PIN);
        }
    }
}

void TMC2209_Step(uint8_t motor, uint32_t steps) {
    if (motor == 1) {
        g_tmc_motor1_remaining += steps;
    } else {
        g_tmc_motor2_remaining += steps;
    }
}

void TMC2209_Stop(void) {
    g_tmc_motor1_remaining = 0;
    g_tmc_motor2_remaining = 0;
    /* STEP 拉低 (PWM 模式下由硬件自动恢复, 但初始化时拉一下确保状态) */
    DL_GPIO_clearPins(GPIO_x_bujin_TX_PORT, GPIO_x_bujin_TX_PIN);
    DL_GPIO_clearPins(GPIO_y_bujin_TX_PORT, GPIO_y_bujin_TX_PIN);
}

uint32_t TMC2209_GetRemaining(uint8_t motor) {
    if (motor == 1) {
        return g_tmc_motor1_remaining;
    }
    return g_tmc_motor2_remaining;
}

/* ============================================================================
 * ISR: Timer 中断产生 STEP 脉冲
 *   每个 Tick 累加 g_tmc_tick_cnt, 达到 divider 才真正发 STEP 脉冲.
 *   两路电机: 同时检查 remaining, 任一路有剩余就 step_high + count--.
 * ============================================================================ */
void TMC_TIMER_IRQ_HANDLER(void) {
    switch (DL_TimerG_getPendingInterrupt(TMC_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_tmc_tick_cnt++;
            if (g_tmc_tick_cnt >= g_tmc_divider) {
                g_tmc_tick_cnt = 0;

                /* X 轴脉冲 */
                if (g_tmc_motor1_remaining > 0) {
                    DL_GPIO_setPins(GPIO_x_bujin_TX_PORT, GPIO_x_bujin_TX_PIN);
                    step_high_hold_short();
                    DL_GPIO_clearPins(GPIO_x_bujin_TX_PORT, GPIO_x_bujin_TX_PIN);
                    g_tmc_motor1_remaining--;
                }

                /* Y 轴脉冲 */
                if (g_tmc_motor2_remaining > 0) {
                    DL_GPIO_setPins(GPIO_y_bujin_TX_PORT, GPIO_y_bujin_TX_PIN);
                    step_high_hold_short();
                    DL_GPIO_clearPins(GPIO_y_bujin_TX_PORT, GPIO_y_bujin_TX_PIN);
                    g_tmc_motor2_remaining--;
                }
            }
            break;
        default:
            break;
    }
}