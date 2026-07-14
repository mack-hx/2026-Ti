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
 * --- SM_zeroset: 把当前位置清零 + 设上限位原点 + 超时 + 上电自动回零 + 落盘 --
 *     K3 按下的瞬间, 节流器先发一帧 0x2A 读当前位置, 收到合法应答后才把
 *     sm_pos 锁存到 origin_pulses, 再串行 4 帧写入驱动器。这样保证
 *     "0x90/0x98 设的就是按下这一刻的位置", 不会因为 LCD 读取和按下沿
 *     错位而把"已经过时的 sm_pos"写回原点。
 *
 *     SM_zeroset(motor, origin, timeout_ms, auto_home_on)
 *         motor       → SM_X / SM_Y
 *         origin      → OL (左限位) / OR (右限位)
 *         timeout_ms  → 回零超时 (ms)
 *         auto_home_on→ true=打开上电自动回零, false=关闭
 *
 *     串行发 4 帧 (节流器自动 10ms 间隔):
 *       step1: 0x90/0x98 设左/右限位原点坐标 = sm_pos (当前位置)
 *       step2: 0x95      设回零超时
 *       step3: 0x97      设上电自动回零
 *       step4: 0x04      SaveParams 落盘
 *
 *     注意: 0x97 仅在下次驱动器上电时生效, 要立刻回零见 SM_zero。
 *     典型用法: SM_zeroset(SM_X, OL, 10000, true);
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
 * ============================================================================ */
#define SM_FRAME_GAP_MS    10U   /* 帧间隔: 与之前 delay(10) 一致 */

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
    /* 回零类原语 (手册 4.5) */
    SM_OP_SET_LEFT_ORIGIN,    /* 0x90 设置左限位原点位置 */
    SM_OP_SET_RIGHT_ORIGIN,   /* 0x98 设置右限位原点位置 */
    SM_OP_SET_LIMIT_HOME,     /* 0x91 设置有无限位回零 (mode/dir/speed/limit_ma) */
    SM_OP_SET_ZERO_TIMEOUT,   /* 0x95 修改回零超时时间 */
    SM_OP_SET_AUTO_HOME,      /* 0x97 设置上电自动回零 */
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

/* SM_zeroset 的参数 (跨多帧缓存, 节流器按步序发出) */
typedef struct {
    uint8_t     addr;
    sm_origin_t origin;         /* LEFT/RIGHT → 选 0x90 还是 0x98 */
    int32_t     origin_pulses;  /* 原点位置 (传 0 表示"当前位置即原点") */
    uint32_t    timeout_ms;     /* 回零超时 */
    bool        auto_home;      /* true=上电自动回零 */
} sm_zeroset_param_t;

/* 节流器单例 */
typedef struct {
    sm_op_t          pending;     /* 当前要发的原语 (NONE = 空) */
    sm_speed_param_t speed;       /* SM_OP_SPEED 的参数 */
    uint32_t         next_ms;     /* 最早可发时间 (tick_ms 基准) */

    /* 上电初始化序列: X enable → Y enable → X position mode → Y position mode */
    uint8_t          init_step;   /* 0..4, 4 表示完成 */

    /* SM_zeroset 串行序列: SET_ORIGIN → SET_TIMEOUT → SET_AUTO_HOME → SAVE, 4 步 */
    uint8_t          zs_step;     /* 0 = 不在序列中, 1..4 表示第几步, 5 = 完 */
    sm_zeroset_param_t zs;

    /* SM_zero 单帧序列: 触发一次回零即可 */
    uint8_t          home_step;   /* 0 = 不在序列, 1 = 挂 SM_OP_TRIGGER_HOME, 2 = 完 */
    uint8_t          home_addr;
    sm_home_mode_t   home_mode;

    /* SM_infzero / SM_limithome 串行序列: SET_LIMIT_HOME → TRIGGER_HOME, 2 步 */
    uint8_t          ih_step;     /* 0 = 不在序列, 1 = 挂 SET_LIMIT_HOME, 2 = 挂 TRIGGER_HOME, 3 = 完 */
    sm_ih_param_t    ih;

    /* 读位置自动重试；purpose 区分普通刷新与“先读当前位置再设置原点”。 */
    bool             read_pending;
    uint32_t         read_last_ms;
    uint32_t         read_start_ms;
    uint8_t          read_addr;
    bool             read_for_zeroset;
} sm_throttle_t;

#define SM_READ_TIMEOUT_MS  500U   /* 读位置最多连续发 500ms, 防止驱动掉线时无限刷屏 */

static sm_throttle_t s_thr = {
    SM_OP_NONE, { 0, PD42_DIR_CW, 0, 0,0,0,0 }, 0, 4,
    0, { 0, OL, 0, 0, false },
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
            PD42S1_SetLeftLimitOrigin(s_thr.zs.addr, s_thr.zs.origin_pulses);
            break;
        case SM_OP_SET_RIGHT_ORIGIN:
            PD42S1_SetRightLimitOrigin(s_thr.zs.addr, s_thr.zs.origin_pulses);
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
        if (PD42S1_TakeFrameFor((uint8_t)motor, &frame) &&
            frame.function_code == PD42_FCT_READ_POSITION &&
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
                    if (s_thr.read_for_zeroset) {
                        s_thr.zs.origin_pulses = pos;
                        s_thr.zs_step = 1;
                    }
                    s_thr.read_for_zeroset = false;
                }
            }
        }
    }

    /* 上电依次使能 X/Y 并切换到通信位置模式；不改驱动器已保存的零点。 */
    if (s_thr.init_step < 4 && s_thr.pending == SM_OP_NONE) {
        if ((int32_t)(now - s_thr.next_ms) < 0) return;
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
        } else if ((int32_t)(now - s_thr.read_last_ms) >= (int32_t)SM_FRAME_GAP_MS) {
            s_thr.pending = SM_OP_READ_POS;
        }
    }

    /* 3. SM_zeroset 序列: SET_ORIGIN → SET_TIMEOUT → SET_AUTO_HOME → SAVE
     *    注意: 不再发 0xF8, 当前位置直接作为原点坐标 (SM_zeroset 入口读 sm_pos) */
    if (s_thr.pending == SM_OP_NONE && s_thr.zs_step >= 1 && s_thr.zs_step <= 4) {
        switch (s_thr.zs_step) {
            case 1:
                s_thr.pending = (s_thr.zs.origin == OL)
                                ? SM_OP_SET_LEFT_ORIGIN
                                : SM_OP_SET_RIGHT_ORIGIN;
                break;
            case 2: s_thr.pending = SM_OP_SET_ZERO_TIMEOUT; break;
            case 3: s_thr.pending = SM_OP_SET_AUTO_HOME;    break;
            case 4: s_thr.pending = SM_OP_SAVE_PARAMS;     break;
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

    if (s_thr.pending == SM_OP_NONE) return;
    if ((int32_t)(now - s_thr.next_ms) < 0) return;

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
    pd42_dir_t d = SM_DIR_NORMALIZE(
                      (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    uint8_t sp = (speed > 6000) ? 6000 : (uint8_t)speed;
    PD42S1_AbsPosMode(motor, d, accel, sp, pulses);
    sm_state = 3;
}

void SM_Move(sm_motor_t motor, sm_dir_t dir, uint8_t accel,
             uint16_t speed, int32_t pulses) {
    pd42_dir_t d = SM_DIR_NORMALIZE(
                      (dir == R) ? PD42_DIR_CW : PD42_DIR_CCW);
    uint8_t sp = (speed > 6000) ? 6000 : (uint8_t)speed;
    PD42S1_RelPosMode(motor, d, accel, sp, pulses);
    sm_state = 3;
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
 * SM_zeroset: 串行 4 帧入节流队列
 *   1) 设原点位置 (0x90 左 或 0x98 右) = sm_pos (当前位置)
 *   2) 设回零超时 (0x95)
 *   3) 设上电自动回零标志 (0x97)
 *   4) SaveParams (0x04) 落盘, 掉电不丢失
 *
 * 注意: 当前位置在入口处读 sm_pos 并写入 0x90/0x98。
 * 上电自动回零标志 0x97 仅在**下次驱动器上电**时生效, 当前进程不会自动回零。
 */
void SM_zeroset(sm_motor_t motor, sm_origin_t origin,
                uint32_t timeout_ms, bool auto_home_on) {
    /* K3 的语义是“按下这一刻的位置”。先读 0x2A；只有收到成功应答后，
     * SM_Tick 才把新位置锁存到 origin_pulses 并启动 0x90/0x98 序列。 */
    s_thr.speed.addr       = (uint8_t)motor;
    s_thr.zs.addr          = (uint8_t)motor;
    s_thr.zs.origin        = origin;
    s_thr.zs.timeout_ms    = timeout_ms;
    s_thr.zs.auto_home     = auto_home_on;
    s_thr.zs_step          = 0;
    s_thr.read_addr        = (uint8_t)motor;
    s_thr.read_pending     = true;
    s_thr.read_for_zeroset = true;
    s_thr.read_last_ms     = tick_ms - SM_FRAME_GAP_MS;
    s_thr.read_start_ms    = tick_ms;
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