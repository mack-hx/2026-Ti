/* ============================================================================
 * @file    tmc2209.h
 * @brief   TMC2209 步进电机驱动底层 (移植自 v1.21/Hardware/stepmotor.h)
 * @note    与 PD42S1 共用 UART TX/RX 引脚, 二选一使用 (不会同时上电)
 *
 *   应用层用法 (主循环):
 *     1. syscfg 禁用 PD42S1 UART, 把 BIANMA1_TIM 改高速 50us
 *     2. main() 上电调 TMC2209_Init();
 *     3. TMC2209_SetDir(1/2, 'r'/'l');   一次性设方向
 *     4. TMC2209_Step(1/2, n);            走 n 步 (非阻塞, ISR 推进)
 *     5. TMC2209_Stop();                  两路立即停
 *     6. TMC2209_GetRemaining(1);         查剩余步数判断到位
 *
 * ============================================================================
 */
#ifndef __TMC2209_H__
#define __TMC2209_H__

#include <stdint.h>

/* ============================================================================
 * 应用层 API
 * ============================================================================ */

/* 初始化: 把 TX 引脚重配为 GPIO 输出 (STEP), RX 引脚重配为 GPIO 输出 (DIR),
 *          启动 STEP Timer 中断. main() 启动时调一次. */
void TMC2209_Init(void);

/* 走 microsteps 步 (非阻塞, 由 Timer ISR 推进).
 *   motor: 1 或 2
 *   microsteps: 步数 (一次调用追加到剩余计数) */
void TMC2209_Step(uint8_t motor, uint32_t microsteps);

/* 单独设置方向, 后续 Step 调用按此方向走.
 *   dir: 'r'/'R' 正转, 'l'/'L' 反转 */
void TMC2209_SetDir(uint8_t motor, char dir);

/* 立即停两路电机 (清剩余计数 + STEP 引脚拉低) */
void TMC2209_Stop(void);

/* 查询剩余步数 */
uint32_t TMC2209_GetRemaining(uint8_t motor);

/* 全局变量 (调试 / LCD 显示用) */
extern volatile uint32_t g_tmc_motor1_remaining;
extern volatile uint32_t g_tmc_motor2_remaining;

#endif /* __TMC2209_H__ */