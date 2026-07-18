/* ============================================================================
 * @file    stepmotor.h
 * @brief   PD42S1 步进电机应用层 - 高层语义接口 (一次函数调用 = 一次运动意图)
 *
 *   应用层用法 (主循环):
 *     1. main() 上电调 SM_Init();              ← 入队 4 帧初始化
 *     2. 主循环每帧 SM_Tick();                 ← 推进节流队列, 实际发帧
 *     3. 按键回调调 SM_Run / SM_Stop / SM_Move / SM_MoveTo
 *     4. 回零调 SM_zeroset / SM_zero / SM_infzero / SM_limithome
 *     5. SM_ReadPosition 非阻塞读当前位置
 *     6. sm_pos / sm_err / sm_state 全局变量给 LCD 渲染读
 *
 * ============================================================================
 */
#ifndef __STEPMOTOR_H__
#define __STEPMOTOR_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 电机地址
 * ============================================================================ */
typedef enum {
    SM_X = 0x01,   /* X 轴电机 (默认驱动器地址) */
    SM_Y = 0x02,   /* Y 轴电机 (UART3, 驱动器地址 0x02) */
} sm_motor_t;

/* 方向 (R = Right 正转 / L = Left 反转) */
typedef enum {
    R = 1,         /* 正转 (CW) */
    L = 2,         /* 反转 (CCW) */
} sm_dir_t;

/* 回零模式 (用于 SM_zero, 手册 4.5.3)
 *
 * 应用层语义值: 0=HN 就近 / 1=HS 单圈 / 2=HM 多圈 (按"短词优先"排, 主循环写 0/1/2 直白)
 * 协议层语义值 (PD42_HOME_*): 0=SINGLE / 1=NEAREST / 2=MULTI (手册规定)
 * stepmotor.c 内部用 sm_home_mode_to_pd42() 做映射, 不要直接 (uint8_t) 转协议字节 */
typedef enum {
    HN = 0,   /* 就近回零: 从当前位置朝最近原点位置移动 */
    HS = 1,   /* 单圈回零: 按完整一圈找原点信号 */
    HM = 2,   /* 多圈回零: 找到绝对 0 点 */
} sm_home_mode_t;

/* 回零撞哪边 (手册 4.5.2 Byte1, 用于 SM_infzero / SM_limithome)
 *
 *   SM_infzero  : 撞机械结构, 靠电流 ≥ limit_ma 判堵转停机
 *   SM_limithome: 撞外部限位开关停机
 *
 * 旋转方向 (0x91 Byte2) 由 side 内部推导: HL→CW, HR→CCW。
 * 0x91 Byte1 取值: HL/HR=0/1 (无限位), HL/HR=2/3 (有限位) */
typedef enum {
    HL = 0,   /* 撞左 (CW 正转) */
    HR = 1,   /* 撞右 (CCW 反转) */
} sm_home_side_t;

/* 注: home_mode 应用层值 (HN/HS/HM = 0/1/2) 与 PD42S1 协议层字节 (手册 4.5.3
 * SINGLE=0/NEAREST=1/MULTI=2) 顺序不同, 映射在 stepmotor.c 内部完成。 */

/* ============================================================================
 * 显示用只读变量 (LCD 渲染读这些)
 * ============================================================================ */
extern volatile int32_t  sm_pos;       /* 最近一次更新的轴位置 (兼容旧代码) */
extern volatile int32_t  sm_speed;     /* 实时速度 */
extern volatile uint8_t  sm_err;       /* 最近一次应答错误码 */
extern volatile uint8_t  sm_state;     /* 0=IDLE 1=FWD 2=REV 3=POS */

/* 按轴读取最近一次成功的 0x2A 位置应答 */
int32_t SM_GetPosition(sm_motor_t motor);

/* ============================================================================
 * 应用层 API
 * ============================================================================ */

/* 上电初始化：两轴使能 + 通信位置模式；保留驱动器已有零点 */
void SM_Init(void);

/* 立即刹车 (内部处理 STOP_IMM + CLEAR_STATUS 链式, 电机/驱动器安全) */
void SM_Stop(sm_motor_t motor);

/* 速度模式: 按指定方向和速度持续转, 松开发刹车 */
void SM_Run(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed);

/* 绝对位置模式: 转到驱动器侧编码器的绝对位置 */
void SM_MoveTo(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses);

/* 相对位置模式: 从当前位置走一段相对位移 */
void SM_Move(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses);

/* 把当前位置设为坐标原点 (立即发 0xF8) */
void SM_ResetPosition(sm_motor_t motor);

/* 读一次实时位置 (0x2A), 自动重试直到应答或 500ms 超时 */
void SM_ReadPosition(sm_motor_t motor);

/* 通知节流器已收到应答 (UI_Render 在解析完 0x2A 应答后调用, 停止重试) */
void SM_AckFrame(uint8_t func);

/* 电机使能/失能 */
void SM_Enable(sm_motor_t motor, bool enable);

/* 非阻塞到位检查 (当前 stub=true, 等 RX 完整化) */
bool SM_IsArrived(sm_motor_t motor);

/* ============================================================================
 * 回零 API
 * ============================================================================ */

/* 把当前 X/Y 轴原点坐标 + 超时 + 上电自动回零 + 左右限位开关一次性写入驱动器并落盘。
 *
 * 串行 6 帧 (节流器 10ms 间隔, 约 60ms):
 *   1) 0x90 设左限位原点坐标 = left_pulses  (int32, 51200=一圈, 有符号)
 *   2) 0x98 设右限位原点坐标 = right_pulses (int32, 51200=一圈, 有符号)
 *   3) 0x95 设回零超时       = timeout_ms   (uint32 ms, 默认 10000)
 *   4) 0x97 设上电自动回零   = auto_home_on (true=开, false=关)
 *   5) 0x99 开关左右限位     = limit_on     (true=开, false=关)
 *   6) 0x04 SaveParams 落盘  (掉电不丢)
 *
 *   当前轴位置不变 — 函数只改写驱动器的原点坐标寄存器, 不发 0xF8 清零。
 *   X/Y 轴的"左/右原点"由 left/right_pulses 决定, 与当前位置无关。
 *   后续 SM_zero(HM) 多圈回零时, 驱动器会去 left/right_pulses 这个坐标停下。
 *
 *   典型用法 (X 轴 0..1800°, Y 轴 -105..75°, 上电自动回零 + 开限位 + 10 秒超时):
 *     SM_zeroset(SM_X,    DEG_TO_PULSES(0),    DEG_TO_PULSES(1800), 10000U, true, true);
 *     SM_zeroset(SM_Y,    DEG_TO_PULSES(-105), DEG_TO_PULSES(75),   10000U, true, true);
 *
 *   关于 timeout_ms 的语义 (用户 2026-07-15 04:35 反馈):
 *     这是驱动器侧回零动作的"最长等待时间"。回零动作期间, 驱动器开始旋转找原点,
 *     一旦找到 (驱动侧 ack) 或超时 (timeout_ms 毫秒), 驱动器自动停机并把"已回零"
 *     标志置位。
 *     推荐值 10000~30000 ms (= 10~30 秒):
 *       - 10000 ms: 默认, 适用于行程 < 半圈 (例如 X 轴 0..1800° 远小于 5 圈)
 *       - 30000 ms: 行程较长 / 启动慢的电机, 给堵转/爬行留余量
 *       - < 5000 ms: 太短, 慢速回零/长行程场景容易"假超时未到原点就停"
 *     它只控制 SM_zero / SM_limithome / SM_infzero 触发的回零动作,
 *     **不影响** SM_zeroset 写寄存器本身的耗时 (那 6 帧 60ms 写完就完事)。
 *
 * @param motor         SM_X / SM_Y
 * @param left_pulses   左限位原点坐标 (int32, 单位: 脉冲, 51200=一圈, 有符号)
 * @param right_pulses  右限位原点坐标 (int32, 单位: 脉冲, 51200=一圈, 有符号)
 * @param timeout_ms    回零超时时间 (uint32 ms, 推荐 10000~30000)
 * @param auto_home_on  true=驱动器下次上电自动回零 (0x97); false=不自动
 * @param limit_on      true=开启左右限位 (0x99), 行程被框在 [left, right];
 *                      false=关闭限位 (全行程)
 *
 * @note  顺序写死: 0x90 → 0x98 → 0x95 → 0x97 → 0x99 → 0x04。驱动器按顺序应用,
 *        但左/右原点坐标本身独立, 先后无依赖。
 */
void SM_zeroset(sm_motor_t motor,
                int32_t left_pulses,  int32_t right_pulses,
                uint32_t timeout_ms,  bool auto_home_on, bool limit_on);

/* 立刻触发回零 (单帧 0x92, 驱动器自执行)。
 * 触发前需先用 SM_zeroset 设过原点位置。 */
void SM_zero(sm_motor_t motor, sm_home_mode_t mode);

/* 无限位回零: 撞机械结构, 电流 ≥ limit_ma 判堵转停机 (手册 4.5.2+4.5.3)。
 * 串行 2 帧: 0x91 (mode=0/1 左/右无限位) + 0x92 触发。
 * dir 由 side 内部推导 (LEFT→CW, RIGHT→CCW)。 */
void SM_infzero(sm_motor_t motor, sm_home_side_t side,
               uint16_t speed_rpm, uint16_t limit_ma);

/* 有限位回零: 撞外部限位开关停机 (手册 4.5.2+4.5.3)。
 * 串行 2 帧: 0x91 (mode=2/3 左/右有限位) + 0x92 触发。
 * 需要硬件侧已接好限位开关, 否则驱动器不会自己停。 */
void SM_limithome(sm_motor_t motor, sm_home_side_t side, uint16_t speed_rpm);

/* 主循环每帧调用一次: 推进节流队列, 实际发出被缓存的协议帧 */
void SM_Tick(void);

#endif