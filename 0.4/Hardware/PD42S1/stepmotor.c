/* ============================================================================
 * @file    stepmotor.c
 * @brief   PD42S1 步进电机应用层 - 高层封装 (非阻塞, 节流器串行发帧)
 * @note    所有延时由底层驱动管，应用层不 sleep 也不读按键
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
 *     串行发 6 帧 (节流器自动 10ms 间隔, 约 60ms):
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

/* ============================================================================
 * 方向翻转开关 (保险开关, 平时 = 0 = 手册默认)
 *
 * 改成 1 时所有 SM_Run/Move/MoveTo/infzero/limithome 的 dir 字节会翻转。
 * 遇到"换驱动器批次/接线相序反"等场景, 一行恢复翻转。
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
volatile int32_t  sm_pos   = 0;
volatile int32_t  sm_speed = 0;
volatile uint8_t  sm_err   = 0;
volatile uint8_t  sm_state = 0;
static volatile int32_t s_axis_pos[2] = { 0, 0 };

static uint8_t sm_axis_index(sm_motor_t motor) {
    return (motor == SM_Y) ? 1U : 0U;
}

int32_t SM_GetPosition(sm_motor_t motor) {
    return s_axis_pos[sm_axis_index(motor)];
}

/* ============================================================================
 * 节流器
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

/* 单条原语 (每条发一帧) */
typedef enum {
    SM_OP_NONE = 0,
    SM_OP_ENABLE,
    SM_OP_SET_MODE,
    SM_OP_SAVE_PARAMS,
    SM_OP_ZERO_POS,
    SM_OP_SPEED,
    SM_OP_STOP_IMM,
    SM_OP_CLEAR_STATUS,
    SM_OP_READ_POS,        /* 0x2A 读一次实时位置 (非循环) */
    SM_OP_REL_POS,         /* 0xF3 相对位置模式 (用户 2026-07-15 改走节流器) */
    SM_OP_ABS_POS,         /* 0xF2 绝对位置模式 (同 REL_POS, 走节流器等应答) */
    /* 回零类原语 (手册 4.5) */
    SM_OP_SET_LEFT_ORIGIN,    /* 0x90 设置左限位原点位置 */
    SM_OP_SET_RIGHT_ORIGIN,   /* 0x98 设置右限位原点位置 */
    SM_OP_SET_LIMIT_HOME,     /* 0x91 设置有无限位回零 (mode/dir/speed/limit_ma) */
    SM_OP_SET_ZERO_TIMEOUT,   /* 0x95 修改回零超时时间 */
    SM_OP_SET_AUTO_HOME,      /* 0x97 设置上电自动回零 */
    SM_OP_SET_LIMIT_SWITCH,   /* 0x99 开关左右限位 */
    SM_OP_TRIGGER_HOME,       /* 0x92 触发回零 */
} sm_op_t;

/* SM_infzero / SM_limithome 共用的参数 (跨 2 帧缓存)
 *   inf_mode = false: SM_infzero  (0x91 Byte1=0/1 左/右无限位, 用 limit_ma)
 *   inf_mode = true : SM_limithome (0x91 Byte1=2/3 左/右有限位, limit_ma 不用) */
typedef struct {
    uint8_t          addr;
    sm_home_side_t   side;
    uint16_t         speed_rpm;
    uint16_t         limit_ma;
    bool             inf_mode;     /* true=无限位, false=有限位 */
} sm_ih_param_t;

/* Speed 指令的参数 (被打断后还能再发) */
typedef struct {
    uint8_t    addr;
    pd42_dir_t dir;
    uint8_t    accel;
    uint8_t    b3, b2, b1, b0;
} sm_speed_param_t;

/* 位置模式指令的参数 (0xF3 REL_POS / 0xF2 ABS_POS 共用, 走节流器等应答 + 重试)
 *   - 与 sm_speed_param_t 几乎一样, 多了 pulses (int32, 大端字节序已经展开)
 *   - 节流器先挂 SM_OP_REL_POS/SM_OP_ABS_POS, SM_Tick 发送时调 PD42S1_RelPosMode/AbsPosMode
 *   - 发完后起 80ms 应答超时计时器, 超时未收到 0xF3/0xF2 + ERR=0x01 → 重发, 最多 3 次 */
typedef struct {
    uint8_t    addr;
    pd42_dir_t dir;
    uint8_t    accel;
    uint16_t   speed;        /* 原 uint16, 不展开成 4 字节, 发送时再拆 */
    int32_t    pulses;
} sm_pos_param_t;

/* 位置命令应答检测状态 (用户 2026-07-15 新增, 解决 0xE3 FOOTER_ERR 丢步)
 *   - 节流器每发一帧 0xF3/0xF2 后, 等 SM_POS_ACK_TIMEOUT_MS 看是否收到 0xF3/0xF2 + ERR=0x01
 *   - 没收到 → 标记 pending_ack, 等下个 SM_Tick 检查超时, 触发重发
 *   - 重发上限 SM_POS_MAX_RETRIES (3 次), 超过后清状态放弃
 *
 * 注意:
 *   - 只追踪最近一帧位置命令的应答, 旧的会被覆盖 (节流器一次只发一帧)
 *   - 如果节流器中位置命令前还有未发的命令 (speed / enable), 位置命令发出时
 *     pending_ack 已经激活, ISR 收到的 0xF3 应答只清 pending_ack 不动节流器
 *   - 0xF3 + ERR=0x01 才是受理; ERR=0xE3 是 NAK, 不算"受理", 不清 pending_ack
 *     (因为命令本身被驱动器拒了, 重发才对) */
typedef struct {
    bool     active;         /* 当前在等应答 */
    uint8_t  addr;           /* 等哪一轴 (0x01 / 0x02) */
    uint8_t  func;           /* 0xF3 或 0xF2 */
    uint8_t  retries;        /* 已重试次数, 0..SM_POS_MAX_RETRIES */
    uint32_t sent_ms;        /* 发出时刻 (tick_ms) */
    sm_pos_param_t param;    /* 重发时复用 */
} sm_pos_ack_t;

/* SM_zeroset 的参数 (跨多帧缓存, 节流器按步序发出) */
typedef struct {
    uint8_t     addr;
    int32_t     left_pulses;     /* 左限位原点坐标 (0x90) */
    int32_t     right_pulses;    /* 右限位原点坐标 (0x98) */
    uint32_t    timeout_ms;      /* 回零超时 */
    bool        auto_home;       /* true=上电自动回零 */
    bool        limit_on;        /* true=开左右限位 (0x99) */
} sm_zeroset_param_t;

/* 节流器单例 */
typedef struct {
    sm_op_t          pending;     /* 当前要发的原语 (NONE = 空) */
    sm_speed_param_t speed;       /* SM_OP_SPEED 的参数 */
    sm_pos_param_t   pos;         /* SM_OP_REL_POS / SM_OP_ABS_POS 的参数 */
    uint32_t         next_ms;     /* 最早可发时间 (tick_ms 基准) */

    /* 位置命令应答追踪 (0xF3 / 0xF2 发出后等 ERR=0x01, 超时重发)
     *   用户 2026-07-15 加: 解决驱动器偶发 0xE3 FOOTER_ERR NAK 丢 0xF3 步进命令,
     *   导致"按了 K1 但电机没动"的丢步问题. 节流器间隔已从 10ms 拉到 15ms,
     *   但仍有偶发, 加重试兜底保证命令到达, 为后续 PID 控制做准备. */
    sm_pos_ack_t     pos_ack;

    /* 上电初始化序列: X enable → Y enable → X position mode → Y position mode */
    uint8_t          init_step;   /* 0..4, 4 表示完成 */

    /* SM_zeroset 串行序列: SET_LEFT_ORIGIN → SET_RIGHT_ORIGIN →
     *                          SET_TIMEOUT → SET_AUTO_HOME →
     *                          SET_LIMIT_SWITCH → SAVE, 6 步 */
    uint8_t          zs_step;     /* 0 = 不在序列中, 1..6 表示第几步, 7 = 完 */
    sm_zeroset_param_t zs;

    /* SM_zero 单帧序列: 触发一次回零即可 */
    uint8_t          home_step;   /* 0 = 不在序列, 1 = 挂 SM_OP_TRIGGER_HOME, 2 = 完 */
    uint8_t          home_addr;
    sm_home_mode_t   home_mode;

    /* SM_infzero / SM_limithome 串行序列: SET_LIMIT_HOME → TRIGGER_HOME, 2 步 */
    uint8_t          ih_step;     /* 0 = 不在序列, 1 = 挂 SET_LIMIT_HOME, 2 = 挂 TRIGGER_HOME, 3 = 完 */
    sm_ih_param_t    ih;

    /* 读位置自动重试 (K3 短按 / 长按触发读当前位置)。 */
    bool             read_pending;
    uint32_t         read_last_ms;
    uint32_t         read_start_ms;
    uint8_t          read_addr;
    /* 已废弃字段 (2026-07-15 SM_zeroset 改为传参): 之前用于"读 0x2A 后启动
     * 0x90/0x98 序列"的旧逻辑, 现在 zs_step 由 SM_zeroset 直接置 1 启动.
     * 保留字段避免 layout 大改, SM_ReadPosition 入口仍置 false 兜底. */
    bool             read_for_zeroset;
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
    false, 0, 0, 0, false
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
            PD42S1_ReadPosition(s_thr.read_addr);
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
 *   HN = 0 (就近)  HS = 1 (单圈)  HM = 2 (多圈)
 *
 * 协议层顺序 (手册 4.5.3, PD42S1_TriggerHome 直接发出去):
 *   PD42_HOME_SINGLE  = 0 (单圈)  PD42_HOME_NEAREST = 1 (就近)  PD42_HOME_MULTI = 2 (多圈)
 *
 * 两者顺序不同, 不能直接 (uint8_t) 强转, 必须查表。 */
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

void SM_Tick(void) {
    uint32_t now = tick_ms;
    pd42_frame_t frame;

    /* 两路 UART 的应答分别消费，位置缓存按轴保存。 */
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
                    if (s_thr.read_pending && s_thr.read_addr == (uint8_t)motor) {
                        s_thr.read_pending = false;
                        /* 旧版 "读到应答后启动 zs_step=1" 路径已废弃:
                         *   SM_zeroset 现在直接传 left/right_pulses, 不再读 0x2A.
                         *   SM_ReadPosition 入口仍置 read_for_zeroset=false 兜底 */
                    }
                }
            }
            /* 0xF3 / 0xF2 位置命令应答 (7 字节短帧 [C5][ADDR][FUNC][ERR][CHK][5C]):
             *   ERR=0x01 (PD42_ACK_OK) = 驱动器受理, 清 pos_ack.active
             *   ERR=0xE3 (PD42_ACK_FOOTER_ERR) = 驱动器拒收, 不清, 等超时重发
             *   ERR=其他 = 同上不受理, 等超时重发
             * 用户 2026-07-15: 解决 0xE3 NAK 偶发丢 0xF3 步进命令的丢步问题 */
            else if (s_thr.pos_ack.active &&
                     s_thr.pos_ack.addr == (uint8_t)motor &&
                     frame.function_code == s_thr.pos_ack.func &&
                     frame.error_code == PD42_ACK_OK) {
                s_thr.pos_ack.active = false;
            }
        }
    }

    /* 上电依次使能 X/Y 并切换到通信位置模式；不改驱动器已保存的零点。 */
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

    /* 2. K3 读位置自动重试: 收到应答 (SM_AckFrame) 或 500ms 超时停 */
    if (s_thr.pending == SM_OP_NONE && s_thr.read_pending) {
        if ((uint32_t)(now - s_thr.read_start_ms) > SM_READ_TIMEOUT_MS) {
            s_thr.read_pending = false;
            s_thr.read_for_zeroset = false;
        } else if (now - s_thr.read_last_ms >= SM_FRAME_GAP_MS) {
            s_thr.pending = SM_OP_READ_POS;
        }
    }

    /* 3. SM_zeroset 序列 (6 帧, 节流器 10ms 间隔, 约 60ms 写完):
     *    SET_LEFT_ORIGIN → SET_RIGHT_ORIGIN → SET_ZERO_TIMEOUT →
     *    SET_AUTO_HOME → SET_LIMIT_SWITCH → SAVE_PARAMS
     *    注意: 不再发 0xF8, 左/右原点坐标由调用方传入 (left_pulses/right_pulses) */
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

    /* 4. SM_zero 触发回零 (1 帧) */
    if (s_thr.pending == SM_OP_NONE && s_thr.home_step == 1) {
        s_thr.pending = SM_OP_TRIGGER_HOME;
        s_thr.home_step = 2;
    }

    /* 5. SM_infzero / SM_limithome 序列: SET_LIMIT_HOME → TRIGGER_HOME */
    if (s_thr.pending == SM_OP_NONE && s_thr.ih_step >= 1 && s_thr.ih_step <= 2) {
        switch (s_thr.ih_step) {
            case 1: s_thr.pending = SM_OP_SET_LIMIT_HOME; break;
            case 2: s_thr.pending = SM_OP_TRIGGER_HOME;  break;
        }
        s_thr.ih_step++;
    }

    /* 6. 位置命令应答超时重发 (用户 2026-07-15 加, 解决 0xF3 被驱动器拒收丢步)
     *    条件:
     *      - pos_ack.active = true (有位置命令在等应答)
     *      - 已发出 SM_POS_ACK_TIMEOUT_MS (默认 80ms) 还没收到 ERR=0x01 应答
     *      - retries < SM_POS_MAX_RETRIES (默认 3, 含首次共 4 次)
     *    动作: 把同一帧重新挂回节流器 (节流器 15ms 间隔自动保证不背靠背),
     *          retries++, sent_ms = now, param 不变 (已存)
     *    如果 retries 满了: 放弃, active=false (驱动器可能掉线, 让用户从 LCD 看 ERR)
     *
     *    注意: 这个分支只在 pending=NONE 时才挂 (不能打断正在发的命令) */
    if (s_thr.pending == SM_OP_NONE && s_thr.pos_ack.active &&
        (uint32_t)(now - s_thr.pos_ack.sent_ms) > SM_POS_ACK_TIMEOUT_MS) {
        if (s_thr.pos_ack.retries < SM_POS_MAX_RETRIES) {
            /* 重新挂回节流器: 复用 param, 节流器按当前 func 决定 SM_OP_REL_POS 或 ABS_POS */
            s_thr.pos = s_thr.pos_ack.param;
            s_thr.pending = (s_thr.pos_ack.func == PD42_FCT_ABS_POS_MODE)
                          ? SM_OP_ABS_POS
                          : SM_OP_REL_POS;
            /* 重试时: 节流器 next_ms 还要等到, 这里不强制; 由第 7 步的 next_ms 守卫把关 */
            s_thr.pos_ack.retries++;
            /* sent_ms 在 sm_emit 中重设 */
        } else {
            /* 重试用完: 放弃, 让 LCD 显示 ERR 帮助调试. 不重置 state, 让用户感知 */
            s_thr.pos_ack.active = false;
        }
    }

    if (s_thr.pending == SM_OP_NONE) return;
    if (now < s_thr.next_ms) return;

    sm_op_t op = s_thr.pending;
    s_thr.pending = SM_OP_NONE;

    if (op == SM_OP_READ_POS) {
        s_thr.read_last_ms = now;
    }

    sm_emit(op);
    s_thr.next_ms = now + SM_FRAME_GAP_MS;

    /* STOP_IMM 必须跟 CLEAR_STATUS (手册 4.4.13 警告, 否则电机发烫) */
    if (op == SM_OP_STOP_IMM) {
        s_thr.pending = SM_OP_CLEAR_STATUS;
    }
}

void SM_Init(void) {
    DL_Common_delayCycles(8000000);  /* 驱动器上电稳定约 100ms @ 80MHz */

    s_thr.speed.addr = SM_X;
    s_thr.init_step = 0;            /* 依次使能 X/Y 并切通信位置模式 */
    s_thr.pending    = SM_OP_NONE;
    s_thr.next_ms    = tick_ms;

    s_axis_pos[0] = 0;
    s_axis_pos[1] = 0;
    sm_pos = 0;
    sm_state = 0;

    /* 用户 2026-07-15: 清位置应答追踪, 防止上电重置前一次重试状态泄漏 */
    s_thr.pos_ack.active  = false;
    s_thr.pos_ack.retries = 0;
}

void SM_Stop(sm_motor_t motor) {
    /* SM_Stop 入队两帧: STOP_IMM + CLEAR_STATUS, 节流器保证 10ms 间隔。
     * 手册 4.4.13: 刹停后必须 ClearStatus, 否则电机发烫。
     * 这里把 CLEAR_STATUS 压成 pending 的"下一帧": STOP_IMM 发完后,
     * 节流器在下个 10ms 窗口自动发出 CLEAR_STATUS。 */
    s_thr.speed.addr = (uint8_t)motor;
    s_thr.pending    = SM_OP_STOP_IMM;
    sm_state = 0;
}

/* ============================================================================
 * 回零 API
 * ============================================================================
 * (SM_DIR_NORMALIZE / SM_DIR_REVERSE 见文件头, 这里不重复)
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

void SM_MoveTo(sm_motor_t motor, sm_dir_t dir, uint8_t accel,
               uint16_t speed, int32_t pulses) {
    /* 用户 2026-07-15 改: 不再直接发, 走节流器等应答 + 超时重发
     *   原来直接调 PD42S1_AbsPosMode, 按 K1/K2 时与节流器其他帧背靠背
     *   导致驱动器偶发 0xE3 FOOTER_ERR, 命令被丢弃, 丢步.
     *   现在缓存入队, 节流器按 15ms 间隔发出, 自动等待应答, 超时自动重发. */
    s_thr.pos.addr   = (uint8_t)motor;
    s_thr.pos.dir    = SM_DIR_NORMALIZE(
                           (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    s_thr.pos.accel  = accel;
    s_thr.pos.speed  = (speed > 6000) ? 6000 : speed;
    s_thr.pos.pulses = pulses;
    s_thr.pending    = SM_OP_ABS_POS;
    sm_state         = 3;
}

void SM_Move(sm_motor_t motor, sm_dir_t dir, uint8_t accel,
             uint16_t speed, int32_t pulses) {
    /* 用户 2026-07-15 改: 同 SM_MoveTo, 走节流器等应答 + 超时重发 */
    s_thr.pos.addr   = (uint8_t)motor;
    s_thr.pos.dir    = SM_DIR_NORMALIZE(
                           (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    s_thr.pos.accel  = accel;
    s_thr.pos.speed  = (speed > 6000) ? 6000 : speed;
    s_thr.pos.pulses = pulses;
    s_thr.pending    = SM_OP_REL_POS;
    sm_state         = 3;
}

void SM_ResetPosition(sm_motor_t motor) {
    PD42S1_ZeroPosition(motor);
}

void SM_ReadPosition(sm_motor_t motor) {
    s_thr.read_addr        = (uint8_t)motor;
    s_thr.read_pending     = true;
    s_thr.read_for_zeroset = false;
    s_thr.read_last_ms     = tick_ms - SM_FRAME_GAP_MS;
    s_thr.read_start_ms    = tick_ms;
}

void SM_AckFrame(uint8_t func) {
    if (s_thr.read_pending && func == PD42_FCT_READ_POSITION) {
        s_thr.read_pending = false;
        s_thr.read_for_zeroset = false;
    }
}

void SM_Enable(sm_motor_t motor, bool enable) {
    PD42S1_MotorEnable(motor, enable);
}

bool SM_IsArrived(sm_motor_t motor) {
    (void)motor;
    /* PD42S1_ReadArrived 是发命令→等应答，目前没实现完整 RX 解析，
     * 这里返回 true 表示已完成，等后续 ISR 解析好帧再来。 */
    return true;
}

/* ============================================================================
 * 回零 API
 * ============================================================================
 *
 * SM_zeroset: 串行 6 帧入节流队列 (节流器 10ms 间隔, 约 60ms 写完)
 *   1) 0x90 设左限位原点  = left_pulses  (调用方传入, 不读当前位置)
 *   2) 0x98 设右限位原点  = right_pulses (调用方传入)
 *   3) 0x95 设回零超时    = timeout_ms
 *   4) 0x97 设上电自动回零 = auto_home_on
 *   5) 0x99 开关左右限位  = limit_on
 *   6) 0x04 SaveParams 落盘, 掉电不丢失
 *
 * 注意: 当前位置 (s_axis_pos[motor-1]) 不变 — 不发 0xF8, 只改写原点寄存器。
 * 上电自动回零标志 0x97 仅在**下次驱动器上电**时生效, 当前进程不会自动回零。
 */
void SM_zeroset(sm_motor_t motor,
                int32_t left_pulses, int32_t right_pulses,
                uint32_t timeout_ms, bool auto_home_on, bool limit_on) {
    /* 入参 → 节流器字段
     *   motor        → s_thr.zs.addr
     *   left_pulses  → s_thr.zs.left_pulses  (0x90)
     *   right_pulses → s_thr.zs.right_pulses (0x98)
     *   timeout_ms   → s_thr.zs.timeout_ms   (0x95)
     *   auto_home_on → s_thr.zs.auto_home    (0x97)
     *   limit_on     → s_thr.zs.limit_on     (0x99)
     * zs_step 由 SM_Tick 推进 (1..6), 写满 6 帧后停在 7 (= 完成).
     *
     * 不再读 0x2A: 旧版本先读当前位置再写原点, 现在调用方直接传坐标, 节省一次往返 */
    s_thr.zs.addr          = (uint8_t)motor;
    s_thr.zs.left_pulses   = left_pulses;
    s_thr.zs.right_pulses  = right_pulses;
    s_thr.zs.timeout_ms    = timeout_ms;
    s_thr.zs.auto_home     = auto_home_on;
    s_thr.zs.limit_on      = limit_on;
    s_thr.zs_step          = 1;    /* 启动 6 帧序列 */
}

/**
 * @brief   触发驱动器立刻执行回零动作 (手册 4.5.3, 0x92)
 * @param   motor  SM_X / SM_Y
 * @param   mode   HS (单圈) / HN (就近) / HM (多圈)
 * @note    触发前需先用 SM_zeroset 设过原点位置, 否则驱动器不知道"零"在哪。
 *          回零过程由驱动器自执行, 主循环通过 SM_IsArrived 查询到位状态。
 */
void SM_zero(sm_motor_t motor, sm_home_mode_t mode) {
    /* 入参 → 节流器字段
     *   motor → s_thr.home_addr (从机地址, 0x92 TRIGGER_HOME)
     *   mode  → s_thr.home_mode (回零模式字节, 0/1/2) */
    s_thr.home_addr = (uint8_t)motor;               /* 0x92 指令的从机地址 */
    s_thr.home_mode = mode;                         /* 0=单圈 / 1=就近 / 2=多圈 */
    s_thr.home_step = 1;                            /* 启动单帧序列 */
}

/**
 * @brief   无限位回零 (手册 4.5.2 + 4.5.3, 0x91 mode=0/1 + 0x92)
 *
 * 物理含义: 不限行程, 靠**堵转电流**判到位。
 *   电机按 side 推得的 dir + speed_rpm 一直旋转 → 驱动器持续检测相电流
 *   → 当电流 ≥ limit_ma 时判堵转 → 停机 → 回零完成。
 *
 * 一次调用 = 一个完整的"无限位回零"意图, 节流器串行发 2 帧:
 *   step1: 0x91 设置参数 (mode=左/右无限位, dir=由 side 推导, speed, limit_ma)
 *   step2: 0x92 触发回零 (HS 启动回零动作,
 *          之后驱动器按 dir+speed 一直转, 电流到 limit_ma 停)
 *
 * 适用前提: 机械结构上撞墙后电机会堵转 (电流会上升)。
 *
 * @param   motor      SM_X / SM_Y
 * @param   side       HL (撞左, dir 推导为 CW) /
 *                     HR (撞右, dir 推导为 CCW)
 * @param   speed_rpm  持续旋转速度 (0~6000 RPM)
 * @param   limit_ma   电流阈值 (0~3000 mA), ≥ 此值判堵转停机
 */
void SM_infzero(sm_motor_t motor, sm_home_side_t side,
                uint16_t speed_rpm, uint16_t limit_ma) {
    /* 入参 → 节流器字段
     *   motor      → s_thr.ih.addr (0x91 SET_LIMIT_HOME 的从机地址)
     *   side       → s_thr.ih.side (0=撞左 / 1=撞右,
     *                                0x91 Byte1 = 0/1 左/右无限位,
     *                                dir 由 side 推导: LEFT→CW, RIGHT→CCW)
     *   speed_rpm  → s_thr.ih.speed_rpm (0x91 Byte3~6: float RPM 大端)
     *   limit_ma   → s_thr.ih.limit_ma  (0x91 Byte7~8: 限位电流 mA 大端)
     *   inf_mode=true: 无限位 (0x91 Byte1=0/1, limit_ma 参与堵转检测)
     * step2 的 0x92 TRIGGER_HOME 复用 s_thr.home_addr/home_mode 字段,
     * 这里一次性把 home 部分也填好, mode 写死 HS。 */
    s_thr.speed.addr    = (uint8_t)motor;           /* 共用 addr 字段 */
    s_thr.ih.addr       = (uint8_t)motor;           /* 0x91 指令专用 */
    s_thr.ih.side       = side;                     /* 撞左/撞右, 推导 0x91 Byte1+dir */
    s_thr.ih.speed_rpm  = speed_rpm;                /* 持续旋转速度 RPM */
    s_thr.ih.limit_ma   = limit_ma;                 /* 电流 ≥ 此值 → 判堵转停机 */
    s_thr.ih.inf_mode   = true;                     /* 无限位模式 */
    s_thr.home_addr     = (uint8_t)motor;           /* 0x92 指令的从机地址 */
    s_thr.home_mode     = HS;           /* 0x92 单圈方式启动回零动作 */
    s_thr.ih_step       = 1;                        /* 启动 2 步序列 */
    sm_state            = 0;
}

/**
 * @brief   有限位回零 (手册 4.5.2 + 4.5.3, 0x91 mode=2/3 + 0x92)
 *
 * 物理含义: 撞到对应侧的**外部限位开关**就停机, 不靠电流检测。
 *   适用前提: 机械结构上对应侧**已接好限位开关**, 否则触发后驱动器不会自己停。
 *
 * 一次调用 = 一个完整的"有限位回零"意图, 节流器串行发 2 帧:
 *   step1: 0x91 设置参数 (mode=左/右有限位, dir=由 side 推导, speed; limit_ma 不用)
 *   step2: 0x92 触发回零
 *
 * @param   motor      SM_X / SM_Y
 * @param   side       HL (撞左限位开关, dir=CW) /
 *                     HR (撞右限位开关, dir=CCW)
 * @param   speed_rpm  回零速度 (0~6000 RPM)
 *
 * @note    与 SM_infzero 的区别:
 *   - 不需要 limit_ma (到位判定靠外部限位开关, 不靠电流)
 *   - 需要硬件侧**已接好限位开关**
 *   - inf_mode=false → 0x91 Byte1 = 2/3 (有限位)
 */
void SM_limithome(sm_motor_t motor, sm_home_side_t side, uint16_t speed_rpm) {
    /* 入参 → 节流器字段
     *   motor      → s_thr.ih.addr (0x91 SET_LIMIT_HOME 的从机地址)
     *   side       → s_thr.ih.side (0=撞左 / 1=撞右,
     *                                0x91 Byte1 = 2/3 左/右有限位,
     *                                dir 由 side 推导: LEFT→CW, RIGHT→CCW)
     *   speed_rpm  → s_thr.ih.speed_rpm (0x91 Byte3~6: float RPM 大端)
     *   limit_ma   → 任意值 (有限位模式不使用, 这里填 0 占位)
     *   inf_mode=false: 有限位 (0x91 Byte1=2/3, limit_ma 不参与判定)
     * step2 的 0x92 TRIGGER_HOME 复用 s_thr.home_addr/home_mode 字段。 */
    s_thr.speed.addr    = (uint8_t)motor;           /* 共用 addr 字段 */
    s_thr.ih.addr       = (uint8_t)motor;           /* 0x91 指令专用 */
    s_thr.ih.side       = side;                     /* 撞哪边限位开关, 推导 0x91 Byte1+dir */
    s_thr.ih.speed_rpm  = speed_rpm;                /* 回零速度 RPM */
    s_thr.ih.limit_ma   = 0;                        /* 有限位模式不用, 占位 */
    s_thr.ih.inf_mode   = false;                    /* 有限位模式 */
    s_thr.home_addr     = (uint8_t)motor;           /* 0x92 指令的从机地址 */
    s_thr.home_mode     = HS;           /* 0x92 启动回零动作 */
    s_thr.ih_step       = 1;                        /* 启动 2 步序列 */
    sm_state            = 0;
}