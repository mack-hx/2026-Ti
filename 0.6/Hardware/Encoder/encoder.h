/* ============================================================================
 * @file    encoder.h
 * @brief   编码电机驱动 - 2 路 AB 相正交编码 (左右两电机) 四倍频查表法
 *
 *   应用层用法 (主循环):
 *     1. main() 里 Encoder_Init()                  ← 初始化 + 启动 BIANMA2_TIM
 *        (BIANMA2_TIM ISR 已在 main.c 中挂 NVIC + 调 Encoder_Update)
 *     2. Encoder_GetCountL/R() 读累计值
 *     3. Encoder_ResetL/R/All() 清零
 *
 * ============================================================================
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 编码器计数值全局变量 (volatile: 中断/主循环跨域访问)
 *   L = 左电机 (PB4/PB5), R = 右电机 (PA28/PA31)
 * ============================================================================ */
extern volatile int32_t Encoder_CountL;   /* 左电机累计计数 (有符号, 值越大越快) */
extern volatile int32_t Encoder_CountR;   /* 右电机累计计数 (有符号, 值越大越快) */

/* ============================================================================
 * API
 * ============================================================================ */

/* 初始化: 读初始 AB 相状态 + 清零 + 启动 BIANMA2_TIM (50us 周期) + enable ZERO_EVENT 中断.
 * 调用后 Encoder_Update() 会被 ISR 自动调, 应用层不需要再主动调.
 * 如果想用别的定时器/不要启动 BIANMA2_TIM, 改用 Encoder_InitStateOnly(). */
void Encoder_Init(void);

/* 仅初始化状态 (清零 + 读 baseline); 不动 BIANMA2_TIM. 给"自己接 ISR"的人用. */
void Encoder_InitStateOnly(void);

/* 启动 BIANMA2_TIM (50us, ZERO_EVENT 中断). 如果用其他定时器, 跳过这一步自己挂 ISR. */
void Encoder_StartTimer(void);

/* 四倍频检测 + 计数累加. 周期调用 (建议 50us, 挂在 BIANMA2_TIM ISR).
 * 注: 移植自 v1.21, 查表结果已取反以符合用户"越大越快"约定 */
void Encoder_Update(void);

/* 读两个编码器的累计值 */
int32_t Encoder_GetCountL(void);
int32_t Encoder_GetCountR(void);

/* 清零: 单路 / 全部 */
void Encoder_ResetL(void);
void Encoder_ResetR(void);
void Encoder_ResetAll(void);

#endif /* __ENCODER_H__ */