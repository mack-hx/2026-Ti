/* ============================================================================
 * @file    stepmotor.c
 * @brief   PD42S1 步进电机应用层 - 高层封装 (非阻塞, 节流器串行发帧)
 * @note    所有延时由底层驱动管，应用层不 sleep 也不读按键
 *
 * ============================================================================
 * 模块命名含义:
 *   SM_* = StepMotor 步进电机抽象层 API
 *   thr  = throttle 节流器 (命令队列, 保证帧间隔 ≥15ms)
 *   op   = operation 原语/操作 (节流器里的任务类型)
 *   pos  = position 位置
 *   ack  = acknowledgment 应答
 *   ih   = infzero/limithome 无限位/限位回零
 *   zs   = zeroset 零点设置
 *
 * ============================================================================
 * 调用方法 (主循环只调用这些函数即可驱动电机，不接触协议细节)
 * ============================================================================
 *
 * --- 上电一次 -----------------------------------------------------------
 *     SM_Init();                  // 两轴使能 + 通信位置模式，不清零、不保存参数
 *
 * --- 主循环每帧调一次 ---------------------------------------------------
 *     SM_Tick();                  // 推进节流队列, 实际发帧
 *
 * ============================================================================
 * 运动 API (一次函数调用 = 一个完整运动意图)
 * ============================================================================
 *
 * --- 速度模式 (按住连续转, 松开发刹车) ---------------------------------
 *     SM_Run(motor, dir, accel, speed)
 *         motor  → SM_X / SM_Y
 *         dir    → R (正转 CW) / L (反转 CCW)
 *         accel  → 加速度 (0~200, 0=直接启动)
 *         speed  → 运行速度 (0~6000 RPM, 入队速度帧, 不阻塞)
 *     例: SM_Run(SM_X, R, 100, 60);            // 入队速度帧, 不阻塞
 *         SM_Stop(SM_X);                        // 入队 StopImmediate+ClearStatus
 *
 * --- 相对位置模式 (按一下到位) -------------------------------------------
 *     SM_Move(motor, dir, accel, speed, pulses)
 *         motor  → SM_X / SM_Y
 *         dir    → R / L
 *         accel  → 加速度 (0~200)
 *         speed  → 运行速度 (0~6000 RPM)
 *         pulses → 相对位移脉冲数 (有符号, 51200=一圈)
 *     例: SM_Move(SM_X, R, 100, 60, 25600);   // 正转走半圈
 *
 * --- 绝对位置模式 -------------------------------------------------------
 *     SM_MoveTo(motor, dir, accel, speed, pulses)
 *         motor  → SM_X / SM_Y
 *         dir    → R / L
 *         accel  → 加速度 (0~200)
 *         speed  → 运行速度 (0~6000 RPM)
 *         pulses → 目标绝对位置 (51200=一圈)
 *     例: SM_MoveTo(SM_X, R, 100, 60, 51200);  // 转到绝对位置 51200
 *
 * --- 读一次实时位置 (节流器自动重试到应答) ------------------------------
 *     SM_ReadPosition(motor);                  // 按下沿调一次即可
 *
 * ============================================================================
 * 回零 API
 * ============================================================================
 *
 * --- SM_zeroset:设左/右限位原点 + 超时 + 上电自动回零 + 限位开关 + 落盘 --
 *     调用方直接传左/右原点坐标 (单位: 脉冲, 51200=一圈), 不读当前位置。
 *     函数只把左/右坐标写入驱动器寄存器, 不发 0xF8 清零 — 当前位置保持不变。
 *
 *     SM_zeroset(motor, left_pulses, right_pulses,
 *                timeout_ms, auto_home_on, limit_on)
 *         motor         → SM_X / SM_Y
 *         left_pulses   → 左限位原点 (0x90), int32 脉冲, 有符号
 *         right_pulses  → 右限位原点 (0x98), int32 脉冲, 有符号
 *         timeout_ms    → 回零超时 (ms), 0x95, 推荐 10000~30000
 *         auto_home_on  → true=打开上电自动回零 (0x97), false=关闭
 *         limit_on      → true=开左右限位 (0x99), false=关
 *
 *     串行发 6 帧 (节流器自动 15ms 间隔, 约 90ms):
 *       step1: 0x90 设左限位原点坐标 = left_pulses
 *       step2: 0x98 设右限位原点坐标 = right_pulses
 *       step3: 0x95 设回零超时
 *       step4: 0x97 设上电自动回零
 *       step5: 0x99 开关左右限位 (受 limit_on 控制)
 *       step6: 0x04 SaveParams 落盘
 *
 *     注意: 0x97 仅在下次驱动器上电时生效, 要立刻回零见 SM_zero。
 *     典型用法:
 *       SM_zeroset(SM_X, DEG_TO_PULSES(0),    DEG_TO_PULSES(1800),
 *                  10000U, true, true);
 *       SM_zeroset(SM_Y, DEG_TO_PULSES(-105), DEG_TO_PULSES(75),
 *                  10000U, true, true);
 *
 *     timeout_ms 推荐 10000~30000 ms:
 *       - 10000 ms: 默认, 行程 < 半圈
 *       - 30000 ms: 行程较长 / 启动慢的电机
 *       - < 5000 ms: 太短, 慢速回零/长行程容易"假超时未到原点就停"
 *
 * --- SM_zero: 立刻触发驱动器执行回零动作 -------------------------------
 *     单帧 0x92, 驱动器自执行, 主循环通过 SM_IsArrived 查询到位状态。
 *     触发前需先用 SM_zeroset 设过原点位置。
 *
 *     SM_zero(motor, home_mode)
 *         motor     → SM_X / SM_Y
 *         home_mode → HS (单圈) / HN (就近) /
 *                     HM (多圈)
 *
 *     典型用法: SM_zero(SM_X, HM);   // 多圈回零
 *
 * --- SM_infzero: 无限位回零 (不限行程, 电流达限位阈值即停) -----------------
 *     手册 4.5.2 Byte1 = 0/1 (左/右无限位)。
 *     不限制行程, 靠**堵转电流**判到位: 电机按 dir+speed 一直旋转 →
 *     电流 ≥ limit_ma 时判堵转 → 停机 → 回零完成。
 *     适用条件: 机械结构上撞墙/原点后电机会堵转。
 *     串行发 2 帧:
 *       step1: 0x91 设置 mode=左/右无限位, dir, speed, limit_ma
 *       step2: 0x92 触发回零
 *
 *     SM_infzero(motor, side, speed_rpm, limit_ma)
 *         motor      → SM_X / SM_Y
 *         side       → HL  (撞左, dir=CW 正转) /
 *                      HR (撞右, dir=CCW 反转)
 *         speed_rpm  → 持续旋转速度 (0~6000 RPM)
 *         limit_ma   → 电流阈值 (0~3000 mA), ≥ 此值判堵转停机
 *
 *     典型用法: SM_infzero(SM_X, HL, 100, 10);
 *
 * --- SM_limithome: 有限位回零 (撞外部限位开关停机) -----------------------
 *     手册 4.5.2 Byte1 = 2/3 (左/右有限位)。
 *     适用条件: 机械结构上对应侧**已接好限位开关**。
 *     与 SM_infzero 区别: 不传 limit_ma (到位判定靠开关触发)。
 *     串行发 2 帧:
 *       step1: 0x91 设置 mode=左/右有限位, dir, speed
 *       step2: 0x92 触发回零
 *
 *     SM_limithome(motor, side, speed_rpm)
 *         motor      → SM_X / SM_Y
 *         side       → HL / HR
 *         speed_rpm  → 回零速度 (0~6000 RPM)
 *
 *     典型用法: SM_limithome(SM_X, HL, 100);
 *
 * ============================================================================
 *
 * 协议关键时序:
 *   - 相邻命令间隔 >= 10ms (<10ms 会丢第二帧), 节流器自动拉开。
 *   - Stop 之后必须 ClearStatus (手册 4.4.13 警告), 否则电机发烫。
 *     SM_Stop 把这两个原语串成一个序列, 由 SM_Tick 节流发出。
 *
 * ============================================================================
 * 应用层不直接调用这些底层函数 (pd42s1.c 里):
 * - PD42S1_SendCommand / PD42S1_UART_Callback  → 协议帧收发
 * - PD42S1_SpeedMode / RelPosMode / AbsPosMode → 帧封装
 * - PD42S1_ZeroPosition / MotorEnable / SaveParams → 寄存器动作
 * - PD42S1_SetLeftLimitOrigin / SetRightLimitOrigin / SetZeroTimeout /
 *   SetAutoHome / TriggerHome / SetLimitHome     → 回零指令封装
 * 这些都已被本文件组合封装。
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "system/clock.h"
#include "Hardware/UART_Host/uart_host.h"

/* ============================================================================
 * 方向翻转开关 (命名: DIR_REVERSE = 方向翻转)
 *
 * SM_DIR_REVERSE = 0 时: dir 不翻转 (手册默认)
 * SM_DIR_REVERSE = 1 时: 所有 SM_Run/Move/MoveTo/infzero/limithome 的 dir 字节翻转
 *
 * 遇到"换驱动器批次/接线相序反"等场景, 改这一行即可恢复
 * ============================================================================ */
#define SM_DIR_REVERSE   0

#if SM_DIR_REVERSE
  #define SM_DIR_NORMALIZE(d)  ((d) == PD42_DIR_CW ? PD42_DIR_CCW : PD42_DIR_CW)
#else
  #define SM_DIR_NORMALIZE(d)  (d)
#endif

/* ============================================================================
 * 全局状态 (LCD 渲染读这些)
 * ============================================================================ */
volatile int32_t  sm_pos   = 0;    /* 最近一次更新的轴位置 (X/Y 共用, 兼容旧代码) */
volatile int32_t  sm_speed = 0;    /* 预留: 实时速度 (当前未启用) */
volatile uint8_t  sm_err   = 0;    /* 驱动器应答错误码 (0x01=OK, 其他=失败) */
volatile uint8_t  sm_state = 0;    /* 电机状态: 0=IDLE 1=FWD 2=REV 3=POS (LCD 用) */

/* 每轴独立位置缓存 (X=s_axis_pos[0], Y=s_axis_pos[1]) */
static volatile int32_t s_axis_pos[2] = { 0, 0 };

/* axis_index: 电机地址 → 数组下标 (X=0, Y=1) */
static inline uint8_t sm_axis_index(sm_motor_t motor) {
    return (motor == SM_Y) ? 1U : 0U;
}

/* SM_GetPosition: 获取指定轴的当前位置缓存 (s_axis_pos) */
int32_t SM_GetPosition(sm_motor_t motor) {
    return s_axis_pos[sm_axis_index(motor)];
}

/* ============================================================================
 * 节流器 (throttle) - 保证命令帧间隔 ≥ 15ms
 * ============================================================================
 *
 * 命名含义:
 *   s_thr = throttle state 节流器状态
 *   pending = 当前待发送的原语
 *   next_ms = 最早可发时间 (帧间隔守卫)
 *
 * 设计要点:
 *   - 单例模式: 全局一个节流器, 缓存当前要发的命令
 *   - 非阻塞: 所有 SM_* 函数立即返回, 实际发帧由 SM_Tick 推进
 *   - 帧间隔: 相邻帧必须 ≥ SM_FRAME_GAP_MS (15ms)
 *
 * ============================================================================
 * SM_FRAME_GAP_MS = 15 ms (从 10ms 拉开, 2026-07-15 修复 0xF3 被驱动器拒收丢步 bug):
 *   - 10ms 是协议规定的"相邻命令最小间隔", 实测在 MSPM0 @80MHz + 库函数 + ISR
 *     调度累计延迟下临界, 偶尔背靠背导致驱动器内部 FIFO 上溢, 返回 0xE3
 *     FOOTER_ERR NAK, 0xF3 相对位置命令被驱动器丢弃 → 丢步.
 *   - 15ms 留 5ms 余量, 满足"≥ 10ms"约束, 几乎不感知延迟, PID 控制时
 *     15ms 控制周期仍然足够 (3.3ms / 步 @ 3000 步/秒 即 60 RPM).
 *   - PID 写完后若嫌慢, 可以从 15 调到 12, 但必须实测确认 0xE3 不复现.
 * ============================================================================ */
#define SM_FRAME_GAP_MS    15U   /* 帧间隔: ≥ 10ms 协议要求, 加 5ms 余量 */

/* 0xF3 / 0xF2 位置命令的重试参数:
 *   - 发了位置命令后, 驱动器应在 SM_POS_ACK_TIMEOUT_MS 内回 0xF3/0xF2 + ERR=0x01
 *     (7 字节短帧, 含 ERR + CHK + TAIL). 如果超时没收到 → 重发.
 *   - SM_POS_MAX_RETRIES = 3 次, 加上首次一共 4 次发, 失败就放弃 (驱动器可能掉线)
 *   - 重试占用节流器窗口, 不会和别的命令背靠背
 *
 * 注意: 0xF3 应答 ERR=0x01 是"驱动器受理", 不代表电机到位; 到位要等 0x30 ARRIVED
 *       应答. 但 0xF3 命令被受理 = 后续 0x30 会按正确状态机推进, PID 控制时正确性
 *       靠 0x30 ARRIVED 闭环.
 */
#define SM_POS_ACK_TIMEOUT_MS   80U   /* 0xF3/0xF2 应答超时 (驱动器典型 < 20ms) */
#define SM_POS_MAX_RETRIES      3U    /* 最多重发 3 次 (含首次共 4 次) */

/* 节流器原语类型 (sm_op_t = StepMotor Operation Type)
 * 每个原语对应一帧协议命令, 由 sm_emit() 实际发送 */
typedef enum {
    SM_OP_NONE = 0,           /* 无待发命令 (节流器空闲) */
    SM_OP_ENABLE,             /* 0xFA 电机使能/失能 */
    SM_OP_SET_MODE,           /* 0xF3 设工作模式 (位置模式/速度模式) */
    SM_OP_SAVE_PARAMS,        /* 0x04 保存参数到 EEPROM */
    SM_OP_ZERO_POS,           /* 0xF8 位置清零 */
    SM_OP_SPEED,              /* 0xF1 速度模式 */
    SM_OP_STOP_IMM,           /* 0xFC 立即停止 */
    SM_OP_CLEAR_STATUS,       /* 0xFB 清除状态 (STOP 后必须调, 否则电机发烫) */
    SM_OP_READ_POS,           /* 0x2A 读一次实时位置 (非循环) */
    SM_OP_REL_POS,            /* 0xF3 相对位置模式 (走节流器等应答+重试) */
    SM_OP_ABS_POS,            /* 0xF2 绝对位置模式 (走节流器等应答+重试) */
    /* 回零类原语 */
    SM_OP_SET_LEFT_ORIGIN,    /* 0x90 设置左限位原点位置 */
    SM_OP_SET_RIGHT_ORIGIN,   /* 0x98 设置右限位原点位置 */
    SM_OP_SET_LIMIT_HOME,     /* 0x91 设置有无限位回零 (mode/dir/speed/limit_ma) */
    SM_OP_SET_ZERO_TIMEOUT,   /* 0x95 修改回零超时时间 */
    SM_OP_SET_AUTO_HOME,      /* 0x97 设置上电自动回零 */
    SM_OP_SET_LIMIT_SWITCH,   /* 0x99 开关左右限位 */
    SM_OP_TRIGGER_HOME,       /* 0x92 触发回零 */
} sm_op_t;

/* SM_infzero / SM_limithome 共用的参数 (ih = infzero/limithome)
 *   inf_mode = false: SM_infzero  (0x91 Byte1=0/1 左/右无限位, 用 limit_ma)
 *   inf_mode = true : SM_limithome (0x91 Byte1=2/3 左/右有限位, limit_ma 不用) */
typedef struct {
    uint8_t          addr;           /* 驱动器地址 (SM_X=0x01 / SM_Y=0x02) */
    sm_home_side_t   side;           /* 撞左(HL) 还是撞右(HR) */
    uint16_t         speed_rpm;      /* 回零速度 (RPM) */
    uint16_t         limit_ma;      /* 电流阈值 (mA, 仅无限位模式用) */
    bool             inf_mode;       /* true=无限位, false=有限位 */
} sm_ih_param_t;

/* 速度指令参数 (被打断后还能再发) */
typedef struct {
    uint8_t    addr;               /* 驱动器地址 */
    pd42_dir_t dir;                /* 方向 CW/CCW */
    uint8_t    accel;             /* 加速度 (0~200) */
    uint8_t    b3, b2, b1, b0;   /* 速度字节 (IEEE754 float, 大端) */
} sm_speed_param_t;

/* 位置模式指令参数 (pos = position)
 *   - 与 sm_speed_param_t 几乎一样, 多了 pulses (int32, 大端字节序已经展开)
 *   - 节流器先挂 SM_OP_REL_POS/SM_OP_ABS_POS, SM_Tick 发送时调 PD42S1_RelPosMode/AbsPosMode
 *   - 发完后起 80ms 应答超时计时器, 超时未收到 0xF3/0xF2 + ERR=0x01 → 重发, 最多 3 次 */
typedef struct {
    uint8_t    addr;               /* 驱动器地址 */
    pd42_dir_t dir;                /* 方向 CW/CCW */
    uint8_t    accel;             /* 加速度 (0~200) */
    uint16_t   speed;             /* 速度 RPM (发送时再拆成 2 字节) */
    int32_t    pulses;            /* 脉冲数 (有符号, 大端) */
} sm_pos_param_t;

/* 位置命令应答追踪 (pos_ack = position acknowledgment)
 *   - 节流器每发一帧 0xF3/0xF2 后, 等 SM_POS_ACK_TIMEOUT_MS 看是否收到 ERR=0x01
 *   - 没收到 → 标记 pending_ack, 等下个 SM_Tick 检查超时, 触发重发
 *   - 重发上限 SM_POS_MAX_RETRIES (3 次), 超过后清状态放弃
 *
 * 注意:
 *   - 只追踪最近一帧位置命令的应答, 旧的会被覆盖 (节流器一次只发一帧)
 *   - 0xF3 + ERR=0x01 才是受理; ERR=0xE3 是 NAK, 不算"受理", 不清 pending_ack
 *     (因为命令本身被驱动器拒了, 重发才对) */
typedef struct {
    bool     active;         /* 当前在等应答 */
    uint8_t  addr;          /* 等哪一轴 (0x01 / 0x02) */
    uint8_t  func;          /* 0xF3 或 0xF2 */
    uint8_t  retries;       /* 已重试次数, 0..SM_POS_MAX_RETRIES */
    uint32_t sent_ms;       /* 发出时刻 (tick_ms) */
    sm_pos_param_t param;   /* 重发时复用 */
} sm_pos_ack_t;

/* SM_zeroset 的参数 (zs = zeroset)
 *   - 跨多帧缓存, 节流器按步序发出
 *   - 6 帧序列: 0x90→0x98→0x95→0x97→0x99→0x04 */
typedef struct {
    uint8_t     addr;           /* 驱动器地址 */
    int32_t     left_pulses;    /* 左限位原点坐标 (0x90) */
    int32_t     right_pulses;   /* 右限位原点坐标 (0x98) */
    uint32_t    timeout_ms;     /* 回零超时 (0x95) */
    bool        auto_home;      /* true=上电自动回零 (0x97) */
    bool        limit_on;       /* true=开左右限位 (0x99) */
} sm_zeroset_param_t;

/* 节流器单例状态 (s_thr = throttle state)
 *
 * 字段命名:
 *   pending = 当前待发送的原语 (op)
 *   speed/pos/zeroset/ih/home = 各类型命令的参数
 *   init_step = 上电初始化序列步序
 *   next_ms = 帧间隔守卫 (最早可发时间)
 *   pos_ack = 位置命令应答追踪
 *   read_* = 读位置参数 (per-axis)
 */
typedef struct {
    sm_op_t          pending;     /* 当前待发的原语 (NONE = 空) */
    sm_speed_param_t speed;       /* SM_OP_SPEED 的参数 */
    sm_pos_param_t   pos;         /* SM_OP_REL_POS / SM_OP_ABS_POS 的参数 */
    uint32_t         next_ms;     /* 最早可发时间 (tick_ms 基准, 帧间隔守卫) */

    /* 位置命令应答追踪 (0xF3 / 0xF2 发出后等 ERR=0x01, 超时重发)
     *   用户 2026-07-15 加: 解决驱动器偶发 0xE3 FOOTER_ERR NAK 丢 0xF3 步进命令,
     *   导致"按了 K1 但电机没动"的丢步问题. 节流器间隔已从 10ms 拉到 15ms,
     *   但仍有偶发, 加重试兜底保证命令到达, 为后续 PID 控制做准备. */
    sm_pos_ack_t     pos_ack;

    /* 上电初始化序列: X使能 → Y使能 → X位置模式 → Y位置模式 */
    uint8_t          init_step;   /* 0..4, 4=完成 */

    /* SM_zeroset 串行序列: SET_LEFT_ORIGIN → SET_RIGHT_ORIGIN →
     *                          SET_TIMEOUT → SET_AUTO_HOME →
     *                          SET_LIMIT_SWITCH → SAVE, 6 步 */
    uint8_t          zs_step;     /* 0=不在序列, 1..6=第几步, 7=完成 */
    sm_zeroset_param_t zs;

    /* SM_zero 单帧序列: 触发一次回零即可 */
    uint8_t          home_step;   /* 0=不在序列, 1=挂 TRIGGER_HOME, 2=完成 */
    uint8_t          home_addr;   /* 从机地址 (SM_X=0x01 / SM_Y=0x02) */
    sm_home_mode_t   home_mode;   /* 回零模式 (HN/HS/HM) */

    /* SM_infzero / SM_limithome 串行序列: SET_LIMIT_HOME → TRIGGER_HOME, 2 步 */
    uint8_t          ih_step;     /* 0=不在序列, 1=挂 SET_LIMIT_HOME, 2=挂 TRIGGER_HOME, 3=完成 */
    sm_ih_param_t    ih;

    /* 读位置参数 (per-axis, 解决 X/Y 同时读互相覆盖的问题)
     *   - 2026-07-18 用户反馈: "主页 X 轴脉冲值不更新" → 同一 tick_ms 内 X/Y
     *     同时 SM_ReadPosition, 旧字段共享导致后调用的轴覆盖先调用的轴.
     *   - read_pending[axis]: 该轴是否在等应答
     *   - read_last_ms[axis]: 上次发送时刻 (计算帧间隔)
     *   - read_start_ms[axis]: 首次发送时刻 (计算总超时)
     *   - read_addr[axis]: 调用方写入的目标从机地址
     *   - read_active_addr: SM_Tick 当前选中的目标从机地址 */
    bool             read_pending[2];
    uint32_t         read_last_ms[2];
    uint32_t         read_start_ms[2];
    uint8_t          read_addr[2];
    uint8_t          read_active_addr;
    /* 已废弃字段 (2026-07-15 SM_zeroset 改为传参):
     *   之前用于"读 0x2A 后启动 0x90/0x98 序列"的旧逻辑,
     *   保留字段避免 layout 大改, SM_ReadPosition 入口仍置 false 兜底. */
    bool             read_for_zeroset[2];
} sm_throttle_t;

#define SM_READ_TIMEOUT_MS  500U   /* 读位置最多连续发 500ms, 防止驱动掉线时无限刷屏 */

static sm_throttle_t s_thr = {
    SM_OP_NONE,
    { 0, PD42_DIR_CW, 0, 0,0,0,0 },
    { 0, PD42_DIR_CW, 0, 0, 0 },
    0,
    { false, 0, 0, 0, 0, { 0, PD42_DIR_CW, 0, 0, 0 } },
    4,
    0, { 0, 0, 0, 0, false, false },
    0, 0, HS,
    0, { 0, HL, 0, 0, true },
    { false, false }, { 0, 0 }, { 0, 0 }, { 0, 0 }, 0, { false, false }
};

/* 应用层 home_mode (HN/HS/HM) → 协议字节 (SINGLE/NEAREST/MULTI) */
static uint8_t sm_home_mode_to_pd42(sm_home_mode_t m);

/* 立即发一帧 (内部用, 假设节流窗口已到)。
 * 返回 op (用于 SM_Tick 决定下一步, 例如 STOP_IMM 后接 CLEAR_STATUS) */
static sm_op_t sm_emit(sm_op_t op) {
    switch (op) {
        case SM_OP_ENABLE:
            PD42S1_MotorEnable(s_thr.speed.addr, true);
            break;
        case SM_OP_SET_MODE:
            PD42S1_SetWorkMode(s_thr.speed.addr, PD42_MODE_POS_LOOP);
            break;
        case SM_OP_SAVE_PARAMS:
            PD42S1_SaveParams(s_thr.speed.addr);
            break;
        case SM_OP_ZERO_POS:
            PD42S1_ZeroPosition(s_thr.speed.addr);
            break;
        case SM_OP_SPEED:
            PD42S1_SendSpeedRaw(s_thr.speed.addr, s_thr.speed.dir,
                                s_thr.speed.accel,
                                s_thr.speed.b3, s_thr.speed.b2,
                                s_thr.speed.b1, s_thr.speed.b0);
            break;
        case SM_OP_REL_POS:
            /* 0xF3 相对位置: 走节流器后, 应答检测由 pos_ack 追踪
             *   首次发 (active=false) → active=true, retries=0
             *   重发 (active=true 且 func 一致) → 复用原 param, 不重置 retries
             *   用 s_thr.pos_ack.active 来区分两种情况 */
            PD42S1_RelPosMode(s_thr.pos.addr, s_thr.pos.dir, s_thr.pos.accel,
                              s_thr.pos.speed, s_thr.pos.pulses);
            if (!s_thr.pos_ack.active ||
                s_thr.pos_ack.func != PD42_FCT_REL_POS_MODE) {
                s_thr.pos_ack.active  = true;
                s_thr.pos_ack.addr    = s_thr.pos.addr;
                s_thr.pos_ack.func    = PD42_FCT_REL_POS_MODE;
                s_thr.pos_ack.retries = 0;
                s_thr.pos_ack.param   = s_thr.pos;
            }
            s_thr.pos_ack.sent_ms = tick_ms;
            break;
        case SM_OP_ABS_POS:
            /* 0xF2 绝对位置: 同 REL_POS */
            PD42S1_AbsPosMode(s_thr.pos.addr, s_thr.pos.dir, s_thr.pos.accel,
                              s_thr.pos.speed, s_thr.pos.pulses);
            if (!s_thr.pos_ack.active ||
                s_thr.pos_ack.func != PD42_FCT_ABS_POS_MODE) {
                s_thr.pos_ack.active  = true;
                s_thr.pos_ack.addr    = s_thr.pos.addr;
                s_thr.pos_ack.func    = PD42_FCT_ABS_POS_MODE;
                s_thr.pos_ack.retries = 0;
                s_thr.pos_ack.param   = s_thr.pos;
            }
            s_thr.pos_ack.sent_ms = tick_ms;
            break;
        case SM_OP_STOP_IMM:
            PD42S1_StopImmediate(s_thr.speed.addr);
            break;
        case SM_OP_CLEAR_STATUS:
            PD42S1_ClearStatus(s_thr.speed.addr);
            break;
        case SM_OP_READ_POS:
            /* s_thr.read_active_addr 在 SM_Tick 选轴阶段已写好 (从机地址) */
            PD42S1_ReadPosition(s_thr.read_active_addr);
            break;
        case SM_OP_SET_LEFT_ORIGIN:
            PD42S1_SetLeftLimitOrigin(s_thr.zs.addr, s_thr.zs.left_pulses);
            break;
        case SM_OP_SET_RIGHT_ORIGIN:
            PD42S1_SetRightLimitOrigin(s_thr.zs.addr, s_thr.zs.right_pulses);
            break;
        case SM_OP_SET_LIMIT_HOME: {
            /* 0x91 SET_LIMIT_HOME:
             *   Byte1 = mode: 0=左无限位, 1=右无限位, 2=左有限位, 3=右有限位
             *   Byte2 = dir : 0=CW, 1=CCW
             *   Byte3~6 = speed_rpm (float 大端)
             *   Byte7~8 = limit_ma (无限位时用作电流阈值; 有限位时填 0 占位)
             * mode 推导:  base = side (0/1), 无限位→+0, 有限位→+2
             * dir  推导:  LEFT→CW, RIGHT→CCW, 再过一次 SM_DIR_NORMALIZE */
            uint8_t base_mode = (uint8_t)s_thr.ih.side;
            uint8_t mode      = s_thr.ih.inf_mode
                                ? base_mode
                                : (uint8_t)(base_mode + 2U);
            pd42_dir_t raw_dir = (s_thr.ih.side == HL)
                                  ? PD42_DIR_CW
                                  : PD42_DIR_CCW;
            pd42_dir_t dir     = SM_DIR_NORMALIZE(raw_dir);
            PD42S1_SetLimitHome(s_thr.ih.addr, mode, (uint8_t)dir,
                                s_thr.ih.speed_rpm, s_thr.ih.limit_ma);
            break;
        }
        case SM_OP_SET_ZERO_TIMEOUT:
            PD42S1_SetZeroTimeout(s_thr.zs.addr, s_thr.zs.timeout_ms);
            break;
        case SM_OP_SET_AUTO_HOME:
            PD42S1_SetAutoHome(s_thr.zs.addr, s_thr.zs.auto_home);
            break;
        case SM_OP_SET_LIMIT_SWITCH:
            /* 0x99 开关左右限位: 设了原点坐标后开启才有效 */
            PD42S1_SetLimitSwitch(s_thr.zs.addr, s_thr.zs.limit_on);
            break;
        case SM_OP_TRIGGER_HOME:
            /* 映射: HN/HS/HM (应用层 0/1/2) → 协议 SINGLE/NEAREST/MULTI (0/1/2) */
            PD42S1_TriggerHome(s_thr.home_addr, sm_home_mode_to_pd42(s_thr.home_mode));
            break;
        default: break;
    }
    return op;
}

/* ============================================================================
 * 内部辅助 - 应用层 home_mode → 协议 mode 字节
 * ============================================================================
 *
 * 应用层顺序 (按"短词优先", 主循环写 0/1/2 直白):
 *   HN = 0 (Home Nearest 就近) / HS = 1 (Home Single 单圈) / HM = 2 (Home Multi 多圈)
 *
 * 协议层顺序 (手册 4.5.3):
 *   PD42_HOME_SINGLE   = 0 (单圈) / PD42_HOME_NEAREST = 1 (就近) / PD42_HOME_MULTI = 2 (多圈)
 *
 * 两者顺序不同, 不能直接 (uint8_t) 强转, 必须查表映射 */
static uint8_t sm_home_mode_to_pd42(sm_home_mode_t m) {
    switch (m) {
        case HN: return PD42_HOME_NEAREST;  /* 应用 0 → 协议 1 */
        case HS: return PD42_HOME_SINGLE;   /* 应用 1 → 协议 0 */
        case HM: return PD42_HOME_MULTI;    /* 应用 2 → 协议 2 (巧合一致) */
        default: return PD42_HOME_SINGLE;   /* 兜底 */
    }
}

/* ============================================================================
 * 内部辅助 - float RPM → 大端字节
 * ============================================================================
 *
 * RPM 用 IEEE 754 float, big-endian。
 * 例: 60.0f → 0x42700000 → [B3=0x42][B2=0x70][B1=0x00][B0=0x00]
 */
static void rpm_to_bytes(float rpm, uint8_t *b3, uint8_t *b2, uint8_t *b1, uint8_t *b0) {
    union { float f; uint32_t u; } u;
    u.f = rpm;
    /* MSPM0 是 little-endian MCU, 要按 big-endian 输出到串口, 所以字节翻转 */
    *b3 = (uint8_t)(u.u >> 24);
    *b2 = (uint8_t)(u.u >> 16);
    *b1 = (uint8_t)(u.u >>  8);
    *b0 = (uint8_t)(u.u);
}

/* ============================================================================
 * API 实现
 * ============================================================================
 *
 * 所有 SM_* 函数都是非阻塞的: 缓存意图立即返回。
 * 主循环需每帧调一次 SM_Tick() 来推进节流状态机。
 */

/* ============================================================================
 * SM_Tick - 节流器推进 (主循环每帧调用一次)
 * ============================================================================
 *
 * 函数功能: 推进节流状态机, 发出被缓存的协议帧
 *
 * 处理逻辑 (按优先级顺序):
 *   1. 消费 RX 应答 (0x2A 位置 / 0xF3/0xF2 位置命令应答)
 *   2. 上电初始化序列 (X使能→Y使能→X模式→Y模式)
 *   3. 读位置自动重试 (per-axis, 500ms 超时)
 *   4. SM_zeroset 6帧序列
 *   5. SM_zero 单帧触发
 *   6. SM_infzero/limithome 2帧序列
 *   7. 位置命令应答超时重发 (80ms, 最多3次)
 *   8. 帧间隔守卫 (next_ms)
 *   9. 发出当前 pending 原语
 *   10. STOP_IMM 后自动链式 CLEAR_STATUS
 *
 * 注意: 所有 SM_* 函数都是非阻塞的, 缓存意图立即返回.
 *       实际发帧由本函数推进.
 */
void SM_Tick(void) {
    uint32_t now = tick_ms;
    pd42_frame_t frame;

    /* [1] 消费 RX 应答, 位置缓存按轴保存 */
    for (uint8_t axis = 0U; axis < 2U; axis++) {
        sm_motor_t motor = (axis == 0U) ? SM_X : SM_Y;
        if (PD42S1_TakeFrameFor((uint8_t)motor, &frame)) {
            /* 0x2A 读位置应答: 5 字节 data, 第 0 字节 ERR, 后 4 字节 int32 大端位置 */
            if (frame.function_code == PD42_FCT_READ_POSITION &&
                frame.data_len >= 5U) {
                sm_err = frame.data[0];
                if (sm_err == PD42_ACK_OK) {
                    int32_t pos = (int32_t)((uint32_t)frame.data[1] << 24)
                                | ((uint32_t)frame.data[2] << 16)
                                | ((uint32_t)frame.data[3] << 8)
                                |  (uint32_t)frame.data[4];
                    s_axis_pos[axis] = pos;
                    sm_pos = pos;
                    if (s_thr.read_pending[axis]) {
                        s_thr.read_pending[axis] = false;
                        /* 旧版 "读到应答后启动 zs_step=1" 路径已废弃:
                         *   SM_zeroset 现在直接传 left/right_pulses, 不再读 0x2A.
                         *   SM_ReadPosition 入口仍置 read_for_zeroset=false 兜底 */
                    }
                }
            }
            /* 0xF3 / 0xF2 位置命令应答 (7 字节短帧):
             *   ERR=0x01 = 驱动器受理, 清 pos_ack.active
             *   ERR=0xE3 = 驱动器拒收, 不清, 等超时重发
             *   ERR=其他 = 同上不受理, 等超时重发 */
            else if (s_thr.pos_ack.active &&
                     s_thr.pos_ack.addr == (uint8_t)motor &&
                     frame.function_code == s_thr.pos_ack.func &&
                     frame.error_code == PD42_ACK_OK) {
                s_thr.pos_ack.active = false;
            }
        }
    }

    /* [2] 上电初始化序列: X使能→Y使能→X位置模式→Y位置模式 */
    if (s_thr.init_step < 4 && s_thr.pending == SM_OP_NONE) {
        if (now < s_thr.next_ms) return;
        switch (s_thr.init_step) {
            case 0:
                s_thr.speed.addr = SM_X;
                s_thr.pending = SM_OP_ENABLE;
                s_thr.init_step = 1;
                break;
            case 1:
                s_thr.speed.addr = SM_Y;
                s_thr.pending = SM_OP_ENABLE;
                s_thr.init_step = 2;
                break;
            case 2:
                s_thr.speed.addr = SM_X;
                s_thr.pending = SM_OP_SET_MODE;
                s_thr.init_step = 3;
                break;
            case 3:
                s_thr.speed.addr = SM_Y;
                s_thr.pending = SM_OP_SET_MODE;
                s_thr.init_step = 4;
                break;
        }
    }

    /* [3] 读位置自动重试 (per-axis, 500ms 超时) */
    if (s_thr.pending == SM_OP_NONE) {
        for (uint8_t axis = 0U; axis < 2U; axis++) {
            if (!s_thr.read_pending[axis]) continue;
            if ((uint32_t)(now - s_thr.read_start_ms[axis]) > SM_READ_TIMEOUT_MS) {
                s_thr.read_pending[axis] = false;
                s_thr.read_for_zeroset[axis] = false;
                continue;
            }
            if (now - s_thr.read_last_ms[axis] >= SM_FRAME_GAP_MS) {
                /* 选中该轴: 写入 read_active_addr, 由 sm_emit 转发到 PD42S1_ReadPosition */
                s_thr.read_active_addr    = s_thr.read_addr[axis];
                s_thr.pending            = SM_OP_READ_POS;
                s_thr.read_last_ms[axis] = now;
                s_thr.read_start_ms[axis] = now;
                break;
            }
        }
    }

    /* [4] SM_zeroset 6帧序列: 0x90→0x98→0x95→0x97→0x99→0x04 */
    if (s_thr.pending == SM_OP_NONE && s_thr.zs_step >= 1 && s_thr.zs_step <= 6) {
        switch (s_thr.zs_step) {
            case 1: s_thr.pending = SM_OP_SET_LEFT_ORIGIN;   break;
            case 2: s_thr.pending = SM_OP_SET_RIGHT_ORIGIN;  break;
            case 3: s_thr.pending = SM_OP_SET_ZERO_TIMEOUT;  break;
            case 4: s_thr.pending = SM_OP_SET_AUTO_HOME;     break;
            case 5: s_thr.pending = SM_OP_SET_LIMIT_SWITCH;  break;
            case 6: s_thr.pending = SM_OP_SAVE_PARAMS;       break;
        }
        s_thr.zs_step++;
    }

    /* [5] SM_zero 单帧触发 (0x92) */
    if (s_thr.pending == SM_OP_NONE && s_thr.home_step == 1) {
        s_thr.pending = SM_OP_TRIGGER_HOME;
        s_thr.home_step = 2;
    }

    /* [6] SM_infzero/limithome 2帧序列: 0x91→0x92 */
    if (s_thr.pending == SM_OP_NONE && s_thr.ih_step >= 1 && s_thr.ih_step <= 2) {
        switch (s_thr.ih_step) {
            case 1: s_thr.pending = SM_OP_SET_LIMIT_HOME; break;
            case 2: s_thr.pending = SM_OP_TRIGGER_HOME;  break;
        }
        s_thr.ih_step++;
    }

    /* [7] 位置命令应答超时重发 (80ms, 最多3次) */
    if (s_thr.pending == SM_OP_NONE && s_thr.pos_ack.active &&
        (uint32_t)(now - s_thr.pos_ack.sent_ms) > SM_POS_ACK_TIMEOUT_MS) {
        if (s_thr.pos_ack.retries < SM_POS_MAX_RETRIES) {
            /* 重新挂回节流器: 复用 param */
            s_thr.pos = s_thr.pos_ack.param;
            s_thr.pending = (s_thr.pos_ack.func == PD42_FCT_ABS_POS_MODE)
                          ? SM_OP_ABS_POS
                          : SM_OP_REL_POS;
            s_thr.pos_ack.retries++;
        } else {
            /* 重试用完: 放弃, 让 LCD 显示 ERR 帮助调试 */
            s_thr.pos_ack.active = false;
        }
    }

    /* [8] 帧间隔守卫 */
    if (s_thr.pending == SM_OP_NONE) return;
    if (now < s_thr.next_ms) return;

    /* [9] 发出当前 pending 原语 */
    sm_op_t op = s_thr.pending;
    s_thr.pending = SM_OP_NONE;

    sm_emit(op);
    s_thr.next_ms = now + SM_FRAME_GAP_MS;

    /* [10] STOP_IMM 后自动链式 CLEAR_STATUS (手册 4.4.13: 否则电机发烫) */
    if (op == SM_OP_STOP_IMM) {
        s_thr.pending = SM_OP_CLEAR_STATUS;
    }
}

/**
 * @brief   SM_Init - 上电初始化
 * @note    入队 4 帧初始化序列: X使能 → Y使能 → X位置模式 → Y位置模式
 *          保留驱动器已有零点 (不清零、不保存参数)
 * @usage   main() 里只调一次
 */
void SM_Init(void) {
    /* 驱动器冷启动需要稳定时间: Y 轴典型 200-500ms
     * 用户 2026-07-16 反馈 cold-boot hang: 100ms 仅够单轴,
     * 拉到 500ms 给两片驱动器都稳定时间 */
    DL_Common_delayCycles(40000000);  /* ~500ms @ 80MHz */

    s_thr.speed.addr = SM_X;
    s_thr.init_step = 0;            /* 启动初始化序列 */
    s_thr.pending    = SM_OP_NONE;
    s_thr.next_ms    = tick_ms;

    s_axis_pos[0] = 0;
    s_axis_pos[1] = 0;
    sm_pos = 0;
    sm_state = 0;

    /* 清位置应答追踪, 防止上电重置前一次重试状态泄漏 */
    s_thr.pos_ack.active  = false;
    s_thr.pos_ack.retries = 0;
}

/**
 * @brief   SM_Stop - 立即刹车
 * @param   motor  SM_X 或 SM_Y
 * @note    内部自动发送 STOP_IMM + CLEAR_STATUS 链式命令 (15ms 间隔)
 *          手册 4.4.13: 刹停后必须 ClearStatus, 否则电机发烫
 * @usage   松开按键时调用
 */
void SM_Stop(sm_motor_t motor) {
    /* 入队两帧: STOP_IMM + CLEAR_STATUS, 节流器保证 15ms 间隔。
     * 手册 4.4.13: 刹停后必须 ClearStatus, 否则电机发烫。
     * 这里把 CLEAR_STATUS 压成 pending 的"下一帧": STOP_IMM 发完后,
     * 节流器在下个 15ms 窗口自动发出 CLEAR_STATUS。 */
    s_thr.speed.addr = (uint8_t)motor;
    s_thr.pending    = SM_OP_STOP_IMM;
    sm_state = 0;
}

/* ============================================================================
 * 回零 API
 * ============================================================================
 * (SM_DIR_NORMALIZE / SM_DIR_REVERSE 见文件头, 这里不重复)
 */

/**
 * @brief   SM_Run - 速度模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转 CW) 或 L (反转 CCW)
 * @param   accel  加速度 (0~200, 0=直接启动)
 * @param   speed  运行速度 (0~6000 RPM)
 * @note    按住连续转, 松开需调 SM_Stop 刹车
 * @usage   按住按键期间调用
 */
void SM_Run(sm_motor_t motor, sm_dir_t dir, uint8_t accel, uint16_t speed) {
    uint8_t b3, b2, b1, b0;
    rpm_to_bytes((float)speed, &b3, &b2, &b1, &b0);
    /* 缓存参数, 由 SM_Tick 节流发出 */
    s_thr.speed.addr  = (uint8_t)motor;
    s_thr.speed.dir   = SM_DIR_NORMALIZE(
                          (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    s_thr.speed.accel = accel;
    s_thr.speed.b3    = b3;
    s_thr.speed.b2    = b2;
    s_thr.speed.b1    = b1;
    s_thr.speed.b0    = b0;
    s_thr.pending     = SM_OP_SPEED;
    sm_state = (dir == R) ? 1 : 2;
}

/**
 * @brief   SM_MoveTo - 绝对位置模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转) 或 L (反转)
 * @param   accel  加速度 (0~200)
 * @param   speed  运行速度 (0~6000 RPM)
 * @param   pulses 目标绝对位置 (脉冲数, int32, 51200=一圈)
 * @note    转到驱动器侧编码器的绝对位置, 走节流器等应答+超时重发
 * @usage   绝对定位时调用
 */
void SM_MoveTo(sm_motor_t motor, sm_dir_t dir, uint8_t accel,
               uint16_t speed, int32_t pulses) {
    /* 走节流器等应答 + 超时重发, 解决偶发 0xE3 丢步问题 */
    s_thr.pos.addr   = (uint8_t)motor;
    s_thr.pos.dir    = SM_DIR_NORMALIZE(
                           (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    s_thr.pos.accel  = accel;
    s_thr.pos.speed  = (speed > 6000) ? 6000 : speed;
    s_thr.pos.pulses = pulses;
    s_thr.pending    = SM_OP_ABS_POS;
    sm_state         = 3;
}

/**
 * @brief   SM_Move - 相对位置模式
 * @param   motor  SM_X 或 SM_Y
 * @param   dir    R (正转) 或 L (反转)
 * @param   accel  加速度 (0~200)
 * @param   speed  运行速度 (0~6000 RPM)
 * @param   pulses 相对位移脉冲数 (有符号, int32, 51200=一圈)
 * @note    从当前位置走一段相对位移, 走节流器等应答+超时重发
 * @usage   按一下到位时调用 (相对移动)
 */
void SM_Move(sm_motor_t motor, sm_dir_t dir, uint8_t accel,
             uint16_t speed, int32_t pulses) {
    /* 走节流器等应答 + 超时重发, 同 SM_MoveTo */
    s_thr.pos.addr   = (uint8_t)motor;
    s_thr.pos.dir    = SM_DIR_NORMALIZE(
                           (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    s_thr.pos.accel  = accel;
    s_thr.pos.speed  = (speed > 6000) ? 6000 : speed;
    s_thr.pos.pulses = pulses;
    s_thr.pending    = SM_OP_REL_POS;
    sm_state         = 3;
}

/**
 * @brief   SM_ResetPosition - 把当前位置设为坐标原点
 * @param   motor  SM_X 或 SM_Y
 * @note    立即发送 0xF8 指令, 当前位置变为 0
 * @usage   需要在当前位置建立零点时调用
 */
void SM_ResetPosition(sm_motor_t motor) {
    PD42S1_ZeroPosition(motor);
}

/**
 * @brief   SM_ReadPosition - 读一次实时位置
 * @param   motor  SM_X 或 SM_Y
 * @note    发送 0x2A 指令, 自动重试直到应答或 500ms 超时
 * @usage   按下 K3 短按时调用
 */
void SM_ReadPosition(sm_motor_t motor) {
    /* per-axis pending cache: 两轴同时 SM_ReadPosition 不会互相覆盖 */
    uint8_t axis = sm_axis_index(motor);
    s_thr.read_addr[axis]        = (uint8_t)motor;
    s_thr.read_pending[axis]     = true;
    s_thr.read_for_zeroset[axis] = false;
    s_thr.read_last_ms[axis]     = tick_ms - SM_FRAME_GAP_MS;
    s_thr.read_start_ms[axis]    = tick_ms;
}

/**
 * @brief   SM_AckFrame - 通知节流器已收到应答
 * @param   func  功能码 (0x2A=读位置)
 * @note    UI_Render 在解析完 0x2A 应答后调用, 停止位置重试
 * @usage   通常由 UI 层自动调用
 */
void SM_AckFrame(uint8_t func) {
    /* 兼容旧 API: 收到 READ_POSITION 应答时清对应轴 pending */
    if (func == PD42_FCT_READ_POSITION) {
        for (uint8_t axis = 0U; axis < 2U; axis++) {
            if (s_thr.read_pending[axis]) {
                s_thr.read_pending[axis]     = false;
                s_thr.read_for_zeroset[axis] = false;
            }
        }
    }
}

/**
 * @brief   SM_Enable - 电机使能/失能
 * @param   motor  SM_X 或 SM_Y
 * @param   enable true=使能, false=失能
 * @note    发送 0xFA 指令
 * @usage   需要禁用电机时调用
 */
void SM_Enable(sm_motor_t motor, bool enable) {
    PD42S1_MotorEnable(motor, enable);
}

/**
 * @brief   SM_CommandQueueIdle - 查询初始化序列完成
 * @return  true=初始化完成且节流器空闲, false=还有命令在发
 * @note    非阻塞, 节流器中没有待发送/待确认的命令
 * @usage   启动时等待两轴初始化完成
 */
bool SM_CommandQueueIdle(void) {
    bool zeroset_idle = (s_thr.zs_step == 0U || s_thr.zs_step > 6U);
    bool home_idle    = (s_thr.home_step == 0U || s_thr.home_step > 1U);
    bool infhome_idle = (s_thr.ih_step == 0U || s_thr.ih_step > 2U);

    return s_thr.init_step >= 4U &&
           s_thr.pending == SM_OP_NONE &&
           !s_thr.pos_ack.active &&
           !s_thr.read_pending[0] && !s_thr.read_pending[1] &&
           zeroset_idle && home_idle && infhome_idle;
}

/**
 * @brief   SM_IsArrived - 非阻塞到位检查
 * @param   motor  SM_X 或 SM_Y
 * @return  true=到位 (当前 stub=true, 等 RX 完整化)
 * @note    当前位置为 stub, 需后续完善 0x30 ARRIVED 应答解析
 * @usage   位置控制后查询是否到达目标
 */
bool SM_IsArrived(sm_motor_t motor) {
    (void)motor;
    /* PD42S1_ReadArrived 是发命令→等应答，目前没实现完整 RX 解析,
     * 这里返回 true 表示已完成, 等后续完善. */
    return true;
}

/* ============================================================================
 * 回零 API 实现
 * ============================================================================ */

/**
 * @brief   SM_zeroset - 配置回零参数并落盘
 * @param   motor         SM_X 或 SM_Y
 * @param   left_pulses   左限位原点坐标 (脉冲数, 51200=一圈)
 * @param   right_pulses  右限位原点坐标 (脉冲数, 51200=一圈)
 * @param   timeout_ms    回零超时 (ms, 推荐 10000~30000)
 * @param   auto_home_on  true=下次上电自动回零
 * @param   limit_on      true=开启左右限位
 * @note    串行发送 6 帧 (节流器 15ms 间隔, 约 90ms):
 *          1) 0x90 设左限位原点
 *          2) 0x98 设右限位原点
 *          3) 0x95 设回零超时
 *          4) 0x97 设上电自动回零
 *          5) 0x99 开关左右限位
 *          6) 0x04 SaveParams 落盘
 * @usage   需要配置回零参数时调用 (K4 长按)
 */
void SM_zeroset(sm_motor_t motor,
                int32_t left_pulses, int32_t right_pulses,
                uint32_t timeout_ms, bool auto_home_on, bool limit_on) {
    /* 入参 → 节流器字段, zs_step 由 SM_Tick 推进 (1..6) */
    s_thr.zs.addr          = (uint8_t)motor;
    s_thr.zs.left_pulses   = left_pulses;
    s_thr.zs.right_pulses  = right_pulses;
    s_thr.zs.timeout_ms    = timeout_ms;
    s_thr.zs.auto_home     = auto_home_on;
    s_thr.zs.limit_on      = limit_on;
    s_thr.zs_step          = 1;    /* 启动 6 帧序列 */
}

/**
 * @brief   SM_zero - 触发驱动器立刻执行回零动作
 * @param   motor  SM_X 或 SM_Y
 * @param   mode   回零模式: HS (单圈) / HN (就近) / HM (多圈)
 * @note    发送单帧 0x92, 驱动器自执行回零动作
 *          触发前需先用 SM_zeroset 设过原点位置
 * @usage   需要立即回零时调用 (K3 长按)
 */
void SM_zero(sm_motor_t motor, sm_home_mode_t mode) {
    s_thr.home_addr = (uint8_t)motor;
    s_thr.home_mode = mode;
    s_thr.home_step = 1;    /* 启动单帧序列 */
}

/**
 * @brief   SM_infzero - 无限位回零 (撞机械结构判堵转停机)
 * @param   motor      SM_X 或 SM_Y
 * @param   side       HL (撞左) / HR (撞右)
 * @param   speed_rpm  持续旋转速度 (RPM)
 * @param   limit_ma  电流阈值 (mA), ≥ 此值判堵转停机
 * @note    串行发送 2 帧: 0x91 + 0x92
 *          物理含义: 不限行程, 靠堵转电流判到位
 * @usage   没有限位开关时用 (K4 长按)
 */
void SM_infzero(sm_motor_t motor, sm_home_side_t side,
                uint16_t speed_rpm, uint16_t limit_ma) {
    s_thr.speed.addr    = (uint8_t)motor;
    s_thr.ih.addr       = (uint8_t)motor;
    s_thr.ih.side       = side;
    s_thr.ih.speed_rpm  = speed_rpm;
    s_thr.ih.limit_ma   = limit_ma;
    s_thr.ih.inf_mode   = true;                    /* 无限位模式 */
    s_thr.home_addr     = (uint8_t)motor;
    s_thr.home_mode     = HS;                      /* 单圈启动 */
    s_thr.ih_step       = 1;                      /* 启动 2 步序列 */
    sm_state            = 0;
}

/**
 * @brief   SM_limithome - 有限位回零 (撞外部限位开关停机)
 * @param   motor      SM_X 或 SM_Y
 * @param   side       HL (撞左限位开关) / HR (撞右限位开关)
 * @param   speed_rpm  回零速度 (RPM)
 * @note    串行发送 2 帧: 0x91 + 0x92
 *          物理含义: 靠外部限位开关停机, 不靠电流检测
 * @usage   有限位开关时用
 */
void SM_limithome(sm_motor_t motor, sm_home_side_t side, uint16_t speed_rpm) {
    s_thr.speed.addr    = (uint8_t)motor;
    s_thr.ih.addr       = (uint8_t)motor;
    s_thr.ih.side       = side;
    s_thr.ih.speed_rpm  = speed_rpm;
    s_thr.ih.limit_ma   = 0;                       /* 有限位模式不用 */
    s_thr.ih.inf_mode   = false;                   /* 有限位模式 */
    s_thr.home_addr     = (uint8_t)motor;
    s_thr.home_mode     = HS;                      /* 单圈启动 */
    s_thr.ih_step       = 1;                      /* 启动 2 步序列 */
    sm_state            = 0;
}