/* ============================================================================
 * @file    encoder.c
 * @brief   2 路 AB 相正交编码器 - 四倍频查表法 (L=左 / R=右)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     Encoder_Init();                    // 读初始状态 + 清零 + 启动 BIANMA2_TIM ISR
 *
 *   BIANMA2_TIM ISR (50us 周期, syscfg 配好, main.c 已 enable NVIC):
 *     Encoder_Update();                  // 4 倍频查表 + 累加, 不需主循环调
 *
 *   主循环 / LCD 读累计值:
 *     int32_t posL = Encoder_GetCountL();   // 左电机 PB4/PB5
 *     int32_t posR = Encoder_GetCountR();   // 右电机 PA28/PA31
 *
 *   清零:
 *     Encoder_ResetL();                  // 单路清零
 *     Encoder_ResetR();
 *     Encoder_ResetAll();                // 两路都清
 *
 * ============================================================================
 * 硬件引脚 (来自 empty.syscfg)
 *   左电机 L (编码器 L):
 *     A 相: PB4  (bianma2_read_B04)
 *     B 相: PB5  (bianma2_read_B05)
 *   右电机 R (编码器 R):
 *     A 相: PA28 (bianma1_read_A28)
 *     B 相: PA31 (bianma1_read_A31)
 *   引脚方向: SysConfig 已配 INPUT (注意必须显式 direction = "INPUT",
 *            SysConfig 对 associatedPins 的 GPIO 默认 OUTPUT,
 *            否则 MCU 会主动拉这两个脚覆盖编码器信号, 详见 AGENTS.md §3)
 *
 * ============================================================================
 * 四倍频原理
 *   AB 相 4 种状态组合 (00/01/10/11) 各代表一个稳定位置, 相邻状态之间转过
 *   1/4 齿. 通过 [上次状态 → 当前状态] 查表, 可以同时判断方向和增量:
 *     direction = encoder_table[last * 4 + curr]
 *     count += direction
 *   查表法的好处: 不需要外部中断, 任意周期轮询都能稳定解码.
 *
 * ============================================================================
 * 命名 + 方向约定 (用户 02:55 反馈)
 *   - PB4/PB5 是左电机 (L), PA28/PA31 是右电机 (R)
 *   - 编码器读数方向: L += dir, R -= dir (R 路取反是为了符合用户"越大越快")
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/Encoder/encoder.h"

/* ============================================================================
 * 计数值 (全局, 供 LCD 显示 / 算法使用)
 *   L = 左电机 (PB4/PB5 = bianma2)
 *   R = 右电机 (PA28/PA31 = bianma1)
 * ============================================================================ */
volatile int32_t Encoder_CountL = 0;
volatile int32_t Encoder_CountR = 0;

/* 上次 AB 相状态 (用于查表) */
static uint8_t s_encL_last = 0;   /* 左电机 bianma2 */
static uint8_t s_encR_last = 0;   /* 右电机 bianma1 */

/* ============================================================================
 * 四倍频查表 (移植自 v1.21)
 * 索引 = (last_state << 2) | current_state, 0..15
 * 值   = +1 正向 / -1 反向 / 0 无效或无变化
 * 编码: 0=00  1=01  2=10  3=11 (bit0=A相, bit1=B相)
 *
 * 方向策略 (用户 04:06 反馈):
 *   - R (右电机, bianma1): 上一轮已经取反 (-=dir), 用户满意, 保持
 *   - L (左电机, bianma2): 上一轮取反后实测按 K2 编码值变化方向反 → 改回 +dir
 * ============================================================================ */
static const int8_t kEncoderTable[16] = {
    0,   /* 00->00: 无变化 */
    1,   /* 00->01: 正向 */
   -1,   /* 00->10: 反向 */
    0,   /* 00->11: 无效状态 */
   -1,   /* 01->00: 反向 */
    0,   /* 01->01: 无变化 */
    0,   /* 01->10: 无效状态 */
    1,   /* 01->11: 正向 */
    1,   /* 10->00: 正向 */
    0,   /* 10->01: 无效状态 */
    0,   /* 10->10: 无变化 */
   -1,   /* 10->11: 反向 */
    0,   /* 11->00: 无效状态 */
   -1,   /* 11->01: 反向 */
    1,   /* 11->10: 正向 */
    0,   /* 11->11: 无变化 */
};

/* ============================================================================
 * 内部: 读编码器当前 AB 状态 (返回 0..3)
 * ============================================================================ */
static inline uint8_t read_encL_state(void) {
    /* 左电机 = bianma2 = PB4/PB5 */
    uint32_t raw = DL_GPIO_readPins(bianma2_PORT,
                                    bianma2_read_B04_PIN | bianma2_read_B05_PIN);
    uint8_t a = (raw & bianma2_read_B04_PIN) ? 1U : 0U;
    uint8_t b = (raw & bianma2_read_B05_PIN) ? 2U : 0U;
    return (uint8_t)(a | b);
}

static inline uint8_t read_encR_state(void) {
    /* 右电机 = bianma1 = PA28/PA31 */
    uint32_t raw = DL_GPIO_readPins(bianma1_PORT,
                                    bianma1_read_A28_PIN | bianma1_read_A31_PIN);
    uint8_t a = (raw & bianma1_read_A28_PIN) ? 1U : 0U;
    uint8_t b = (raw & bianma1_read_A31_PIN) ? 2U : 0U;
    return (uint8_t)(a | b);
}

/* ============================================================================
 * API 实现
 * ============================================================================ */
void Encoder_InitStateOnly(void) {
    /* SysConfig 已经把引脚配成 INPUT, 这里只读初始状态 + 清零 */
    s_encL_last = read_encL_state();
    s_encR_last = read_encR_state();
    Encoder_CountL = 0;
    Encoder_CountR = 0;
}

void Encoder_StartTimer(void) {
    /* syscfg 只 init 了 Timer, 没 enable 中断和 start (startTimer=STOP)
     * 这里手动: enable ZERO_EVENT 中断 + 启动计数器 */
    DL_TimerA_enableInterrupt(BIANMA2_TIM_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(BIANMA2_TIM_INST);
}

void Encoder_Init(void) {
    Encoder_InitStateOnly();
    Encoder_StartTimer();
}

void Encoder_Update(void) {
    /* 左电机 (L = bianma2): 上一轮取反, 实测方向反 → 改回 +dir
     *   R (右电机 = bianma1): 保持 -dir (上一轮已调过, 用户 04:06 未提 R 反)
     */
    uint8_t curr;
    curr = read_encL_state();
    if (curr != s_encL_last) {
        uint8_t idx = (uint8_t)((s_encL_last << 2) | curr);
        int8_t  dir = kEncoderTable[idx];
        if (dir != 0) {
            Encoder_CountL += dir;   /* L 路: 改回 +dir (用户 04:06) */
        }
        s_encL_last = curr;
    }

    /* 右电机 (R = bianma1) */
    curr = read_encR_state();
    if (curr != s_encR_last) {
        uint8_t idx = (uint8_t)((s_encR_last << 2) | curr);
        int8_t  dir = kEncoderTable[idx];
        if (dir != 0) {
            Encoder_CountR -= dir;   /* R 路: 保持 -dir (用户 02:55 反馈: 越大越快) */
        }
        s_encR_last = curr;
    }
}

int32_t Encoder_GetCountL(void) { return Encoder_CountL; }
int32_t Encoder_GetCountR(void) { return Encoder_CountR; }

void Encoder_ResetL(void) { Encoder_CountL = 0; }
void Encoder_ResetR(void) { Encoder_CountR = 0; }
void Encoder_ResetAll(void) {
    Encoder_CountL = 0;
    Encoder_CountR = 0;
}