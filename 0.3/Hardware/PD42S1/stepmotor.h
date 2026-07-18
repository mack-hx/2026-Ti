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
    SM_Y = 0x02,   /* Y 轴电机 (第二个驱动器, 当前未启用) */
} sm_motor_t;

/* 方向 (R = Right 正转 / L = Left 反转) */
typedef enum {
    R = 1,         /* 正转 (CW) */
    L = 2,         /* 反转 (CCW) */
} sm_dir_t;

/* 原点位置类型 (用于 SM_zeroset) */
typedef enum {
    OL = 0,   /* 左限位原点 (手册 0x90) */
    OR = 1,   /* 右限位原点 (手册 0x98) */
} sm_origin_t;

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
extern volatile int32_t  sm_pos;       /* 当前位置 */
extern volatile int32_t  sm_speed;     /* 实时速度 */
extern volatile uint8_t  sm_err;        /* 错误码 */
extern volatile uint8_t  sm_state;      /* 0=IDLE 1=FWD 2=REV 3=POS */

/* ============================================================================
 * 应用层 API
 * ============================================================================ */

/* 上电初始化 (使能 + 工作模式 + 清零, 串行 4 帧, 10ms 间隔) */
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

/* 把当前位置写入原点坐标寄存器 + 设回零参数 + 上电自动回零 + 落盘。
 * 串行 4 帧 (节流器 10ms 间隔, 约 40ms):
 *   1) 0x90/0x98 设原点坐标 = sm_pos (当前位置)
 *   2) 0x95 超时  3) 0x97 上电自动回零  4) 0x04 SaveParams 落盘
 * 注意: 当前位置不变 (K3 读出来还是原值), 只是原点坐标寄存器被改写。
 * 后续 SM_zero(HM) 多圈回零时驱动器会去 sm_pos 这个坐标停下。 */
void SM_zeroset(sm_motor_t motor, sm_origin_t origin,
               uint32_t timeout_ms, bool auto_home_on);

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