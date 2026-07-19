/* ============================================================================
 * @file    stepmotor.h
 * @brief   PD42S1 步进电机应用层 - 高层语义接口 (一次函数调用 = 一次运动意图)
 *
 *   模块命名含义:
 *     - SM = StepMotor (步进电机抽象层)
 *     - PD42 = 驱动器型号 (正点原子 PD42S1 闭环步进)
 *     - 所有 SM_* 函数 = 应用层一次运动意图，非阻塞
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
 * 电机地址 (枚举命名: SM_X/SM_Y 表示哪个轴)
 * ============================================================================ */
typedef enum {
    SM_X = 0x01,   /* X 轴电机 (默认驱动器地址 0x01, 接 UART2) */
    SM_Y = 0x02,   /* Y 轴电机 (驱动器地址 0x02, 接 UART3) */
} sm_motor_t;

/* 方向 (枚举命名: R=Right 正转 / L=Left 反转)
 *   - R (Right) = 顺时针 CW (ClockWise)
 *   - L (Left)  = 逆时针 CCW (Counter-ClockWise) */
typedef enum {
    R = 1,         /* 正转 (CW, ClockWise) */
    L = 2,         /* 反转 (CCW, Counter-ClockWise) */
} sm_dir_t;

/* 回零模式 (枚举命名: HN=Home Nearest / HS=Home Single / HM=Home Multi)
 *
 * 应用层语义值: 0=HN 就近 / 1=HS 单圈 / 2=HM 多圈 (按"短词优先"排, 主循环写 0/1/2 直白)
 * 协议层语义值 (PD42_HOME_*): 0=SINGLE / 1=NEAREST / 2=MULTI (手册规定)
 * stepmotor.c 内部用 sm_home_mode_to_pd42() 做映射, 不要直接 (uint8_t) 转协议字节 */
typedef enum {
    HN = 0,   /* 就近回零 (Home Nearest): 从当前位置朝最近原点位置移动 */
    HS = 1,   /* 单圈回零 (Home Single): 按完整一圈找原点信号 */
    HM = 2,   /* 多圈回零 (Home Multi): 找到绝对 0 点 */
} sm_home_mode_t;

/* 回零撞哪边 (枚举命名: HL=Home Left / HR=Home Right)
 *
 *   SM_infzero  : 撞机械结构, 靠电流 ≥ limit_ma 判堵转停机
 *   SM_limithome: 撞外部限位开关停机
 *
 * 旋转方向 (0x91 Byte2) 由 side 内部推导: HL→CW, HR→CCW。
 * 0x91 Byte1 取值: HL/HR=0/1 (无限位), HL/HR=2/3 (有限位) */
typedef enum {
    HL = 0,   /* 撞左 (Home Left): CW 正转撞左限位 */
    HR = 1,   /* 撞右 (Home Right): CCW 反转撞右限位 */
} sm_home_side_t;

/* 注: home_mode 应用层值 (HN/HS/HM = 0/1/2) 与 PD42S1 协议层字节 (手册 4.5.3
 * SINGLE=0/NEAREST=1/MULTI=2) 顺序不同, 映射在 stepmotor.c 内部完成。 */

/* ============================================================================
 * 显示用只读变量 (LCD 渲染读这些)
 * ============================================================================ */
extern volatile int32_t  sm_pos;       /* sm_GetPosition: 最近一次更新的轴位置 (X/Y 轴共用, 实际用 s_axis_pos) */
extern volatile int32_t  sm_speed;     /* 预留: 实时速度 (当前未启用) */
extern volatile uint8_t  sm_err;       /* 驱动器应答错误码 (0x01=OK, 0xE3=帧尾错等) */
extern volatile uint8_t  sm_state;     /* 电机状态: 0=IDLE 1=FWD 2=REV 3=POS (LCD 显示用) */

/**
 * @brief   SM_GetPosition - 获取指定轴的当前位置
 * @param   motor  SM_X (X 轴) 或 SM_Y (Y 轴)
 * @return  该轴的当前位置 (脉冲数, int32), 由 SM_ReadPosition 更新
 * @note    每轴独立缓存 (s_axis_pos[0]=X, s_axis_pos[1]=Y), 解决同时读 X/Y 时互相覆盖的问题
 */
int32_t SM_GetPosition(sm_motor_t motor);

/* ============================================================================
 * 应用层 API (命名: SM_* = StepMotor)
 * ============================================================================ */

/**
 * @brief   SM_Init - 上电初始化
 * @note    入队 4 帧初始化序列: X使能 → Y使能 → X位置模式 → Y位置模式
 *          保留驱动器已有零点 (不清零、不保存参数)
 * @usage   main() 里只调一次
 */
void SM_Init(void);

/**
 * @brief   SM_Stop - 立即刹车
 * @param   motor  SM_X 或 SM_Y
 * @note    内部自动发送 STOP_IMM + CLEAR_STATUS 链式命令 (10ms 间隔)
 *          手册 4.4.13: 刹停后必须 ClearStatus, 否则电机发烫
 * @usage   松开按键时调用
 */
void SM_Stop(sm_motor_t motor);

/**
 * @brief   SM_Run - 速度模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转 CW) 或 L (反转 CCW)
 * @param   accel  加速度 (0~200, 0=直接启动)
 * @param   speed  运行速度 (0~6000 RPM)
 * @note    按住连续转, 松开需调 SM_Stop 刹车
 * @usage   按住按键期间调用
 */
void SM_Run(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed);

/**
 * @brief   SM_MoveTo - 绝对位置模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转) 或 L (反转)
 * @param   accel  加速度 (0~200)
 * @param   speed  运行速度 (0~6000 RPM)
 * @param   pulses 目标绝对位置 (脉冲数, int32, 51200=一圈)
 * @note    转到驱动器侧编码器的绝对位置
 * @usage   绝对定位时调用 (当前位置不变, 只关心目标点)
 */
void SM_MoveTo(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses);

/**
 * @brief   SM_Move - 相对位置模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转) 或 L (反转)
 * @param   accel  加速度 (0~200)
 * @param   speed  运行速度 (0~6000 RPM)
 * @param   pulses 相对位移脉冲数 (有符号, int32, 51200=一圈)
 * @note    从当前位置走一段相对位移, 常用于步进电机调试
 * @usage   按一下到位时调用 (相对移动)
 */
void SM_Move(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed, int32_t pulses);

/**
 * @brief   SM_ResetPosition - 把当前位置设为坐标原点
 * @param   motor  SM_X 或 SM_Y
 * @note    立即发送 0xF8 指令, 当前位置变为 0
 * @usage   需要在当前位置建立零点时调用
 */
void SM_ResetPosition(sm_motor_t motor);

/**
 * @brief   SM_ReadPosition - 读一次实时位置
 * @param   motor  SM_X 或 SM_Y
 * @note    发送 0x2A 指令, 自动重试直到应答或 500ms 超时
 * @usage   按下 K3 短按时调用
 */
void SM_ReadPosition(sm_motor_t motor);

/**
 * @brief   SM_AckFrame - 通知节流器已收到应答
 * @param   func  功能码 (0x2A=读位置)
 * @note    UI_Render 在解析完 0x2A 应答后调用, 停止位置重试
 * @usage   通常由 UI 层自动调用
 */
void SM_AckFrame(uint8_t func);

/**
 * @brief   SM_Enable - 电机使能/失能
 * @param   motor  SM_X 或 SM_Y
 * @param   enable true=使能, false=失能
 * @note    发送 0xFA 指令
 * @usage   需要禁用电机时调用
 */
void SM_Enable(sm_motor_t motor, bool enable);

/**
 * @brief   SM_CommandQueueIdle - 查询初始化序列完成
 * @return  true=初始化完成且节流器空闲, false=还有命令在发
 * @note    非阻塞, 节流器中没有待发送/待确认的命令
 * @usage   启动时等待两轴初始化完成
 */
bool SM_CommandQueueIdle(void);

/**
 * @brief   SM_IsArrived - 非阻塞到位检查
 * @param   motor  SM_X 或 SM_Y
 * @return  true=到位 (当前 stub=true, 等 RX 完整化)
 * @note    当前位置为 stub, 需后续完善 0x30 ARRIVED 应答解析
 * @usage   位置控制后查询是否到达目标
 */
bool SM_IsArrived(sm_motor_t motor);

/* ============================================================================
 * 回零 API (命名: zeroset=设零点 / zero=触发回零 / infzero=无限位 / limithome=限位)
 * ============================================================================ */

/**
 * @brief   SM_zeroset - 配置回零参数并落盘
 * @param   motor         SM_X 或 SM_Y
 * @param   left_pulses   左限位原点坐标 (int32, 单位: 脉冲, 51200=一圈)
 * @param   right_pulses  右限位原点坐标 (int32, 单位: 脉冲, 51200=一圈)
 * @param   timeout_ms    回零超时时间 (uint32 ms, 推荐 10000~30000)
 * @param   auto_home_on  true=驱动器下次上电自动回零 (0x97); false=不自动
 * @param   limit_on      true=开启左右限位 (0x99), 行程被框在 [left, right];
 *                        false=关闭限位 (全行程)
 * @note    串行发送 6 帧 (节流器 15ms 间隔, 约 90ms):
 *          1) 0x90 设左限位原点坐标 = left_pulses
 *          2) 0x98 设右限位原点坐标 = right_pulses
 *          3) 0x95 设回零超时 = timeout_ms
 *          4) 0x97 设上电自动回零 = auto_home_on
 *          5) 0x99 开关左右限位 = limit_on
 *          6) 0x04 SaveParams 落盘 (掉电不丢)
 *
 *          当前轴位置不变 — 函数只改写驱动器的原点坐标寄存器, 不发 0xF8 清零。
 *          X/Y 轴的"左/右原点"由 left/right_pulses 决定, 与当前位置无关。
 *
 *          关于 timeout_ms 的语义:
 *            这是驱动器侧回零动作的"最长等待时间"。回零动作期间, 驱动器开始旋转找原点,
 *            一旦找到 (驱动侧 ack) 或超时 (timeout_ms 毫秒), 驱动器自动停机并把"已回零"
 *            标志置位。
 *            推荐值 10000~30000 ms (= 10~30 秒):
 *              - 10000 ms: 默认, 适用于行程 < 半圈 (例如 X 轴 0..1800° 远小于 5 圈)
 *              - 30000 ms: 行程较长 / 启动慢的电机, 给堵转/爬行留余量
 *              - < 5000 ms: 太短, 慢速回零/长行程场景容易"假超时未到原点就停"
 *            它只控制 SM_zero / SM_limithome / SM_infzero 触发的回零动作,
 *            **不影响** SM_zeroset 写寄存器本身的耗时 (那 6 帧 90ms 写完就完事)。
 *
 *          关于 0x97 auto_home_on:
 *            仅在**下次驱动器上电**时生效, 当前进程不会自动回零。
 *            要立刻回零请调 SM_zero。
 *
 * @par     典型用法:
 *          SM_zeroset(SM_X,    DEG_TO_PULSES(0),    DEG_TO_PULSES(1800), 10000U, true, true);
 *          SM_zeroset(SM_Y,    DEG_TO_PULSES(-105), DEG_TO_PULSES(75),   10000U, true, true);
 *
 * @note    顺序写死: 0x90 → 0x98 → 0x95 → 0x97 → 0x99 → 0x04。
 *          驱动器按顺序应用, 但左/右原点坐标本身独立, 先后无依赖。
 */
void SM_zeroset(sm_motor_t motor,
                int32_t left_pulses,  int32_t right_pulses,
                uint32_t timeout_ms,  bool auto_home_on, bool limit_on);

/**
 * @brief   SM_zero - 触发驱动器立刻执行回零动作
 * @param   motor  SM_X 或 SM_Y
 * @param   mode   回零模式: HS (单圈) / HN (就近) / HM (多圈)
 * @note    发送单帧 0x92, 驱动器自执行回零动作
 *          触发前需先用 SM_zeroset 设过原点位置, 否则驱动器不知道"零"在哪。
 *          回零过程由驱动器自执行, 主循环通过 SM_IsArrived 查询到位状态。
 * @usage   需要立即回零时调用 (K3 长按)
 */
void SM_zero(sm_motor_t motor, sm_home_mode_t mode);

/**
 * @brief   SM_infzero - 无限位回零 (撞机械结构判堵转停机)
 * @param   motor      SM_X 或 SM_Y
 * @param   side      HL (撞左, dir 推导为 CW) / HR (撞右, dir 推导为 CCW)
 * @param   speed_rpm 持续旋转速度 (0~6000 RPM)
 * @param   limit_ma  电流阈值 (0~3000 mA), ≥ 此值判堵转停机
 * @note    串行发送 2 帧: 0x91 (mode=0/1 左/右无限位) + 0x92 触发。
 *          dir 由 side 内部推导 (LEFT→CW, RIGHT→CCW)。
 *          物理含义: 不限行程, 靠**堵转电流**判到位。
 *          电机按 side 推得的 dir + speed_rpm 一直旋转 → 驱动器持续检测相电流
 *          → 当电流 ≥ limit_ma 时判堵转 → 停机 → 回零完成。
 *          适用前提: 机械结构上撞墙后电机会堵转 (电流会上升)。
 * @usage   没有限位开关时用 (K4 长按)
 */
void SM_infzero(sm_motor_t motor, sm_home_side_t side,
               uint16_t speed_rpm, uint16_t limit_ma);

/**
 * @brief   SM_limithome - 有限位回零 (撞外部限位开关停机)
 * @param   motor      SM_X 或 SM_Y
 * @param   side       HL (撞左限位开关, dir=CW) / HR (撞右限位开关, dir=CCW)
 * @param   speed_rpm  回零速度 (0~6000 RPM)
 * @note    串行发送 2 帧: 0x91 (mode=2/3 左/右有限位) + 0x92 触发。
 *          物理含义: 撞到对应侧的**外部限位开关**就停机, 不靠电流检测。
 *          适用前提: 机械结构上对应侧**已接好限位开关**, 否则触发后驱动器不会自己停。
 *          与 SM_infzero 的区别: 不传 limit_ma (到位判定靠外部限位开关, 不靠电流)
 * @usage   有限位开关时用
 */
void SM_limithome(sm_motor_t motor, sm_home_side_t side, uint16_t speed_rpm);

/**
 * @brief   SM_Tick - 节流器推进 (主循环每帧调用一次)
 * @note    推进节流队列, 实际发出被缓存的协议帧
 *          处理: 上电初始化序列 / 读位置自动重试 / zeroset 6帧序列 /
 *                zero 单帧触发 / infzero/limithome 2帧序列 / 位置命令应答超时重发
 * @usage   main() 主循环每帧调用一次
 */
void SM_Tick(void);

#endif