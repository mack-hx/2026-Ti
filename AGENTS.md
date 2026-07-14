# AGENTS.md - 给 AI 助手的项目说明

> **新对话窗口请先读本文件 +** `LOG.md`**。本文件只保留协作必需的稳态信息，过程记录见** `LOG.md`**。**

---

## 1. 项目背景

- **赛事**：2026 TI 电赛（嵌入式赛道）
- **平台**：MSPM0G3507 (LQFP-64)，TI MSPM0 SDK 2.10.00.04
- **工程版本**:
- `0.4/` (**当前最新, 实验田; 0.3 的快照 + 后续改动都在这里**)
 - `0.3/` (**稳态分支, 锁住不再改; 0.4 出问题时的回退基准**)
 - `0.1/` / `0.2/` (历史版本, 不再改动)
- **IDE**：CCS (Code Composer Studio)，SysConfig 图形化配置
- **主机**：macOS 15 (darwin 25.5.0)
- **注：每次修改仅修改最新版本的文件**

---

## 2. LCD 渲染约束（最容易忘）

**1.8寸 TFT LCD, 竖屏 128×160,** `USE_HORIZONTAL=0`**, 字体** `LCD_8X16` **(8 像素宽 × 16 像素高)。**

- **每行最多 16 个字符**（128 / 8 = 16）
- **最多 10 行**（160 / 16 = 10）
- 行高 16 px（与字模一致, **无额外间距**） → **每行占 16 px**

写 UI 时所有 `snprintf` / `LCD_ShowString` 必须按 16 字符上限规划，**不许超界**（超出会显示到下一行或被截断, 且无运行时检查）。

`ROW_Y(n) = n * 16`（定义在 `0.4/user/UI/ui.h`）。当前布局参考 `0.4/user/UI/ui.c`（motor 页）：ROW 0..5（y=0/16/32/48/64/80）+ **页脚 ROW 9 y=144** 显示 `P<n>/<N> <name>`。新增页插入 `pages[]` 注册表，K5 切页详见 §6。

**注**：早期版本用 18 px/行（含 2 px 行间距），只能挤 9 行——已统一改为 16 px/行 1:1 满屏。历史截图上看到的"行间空隙"实际是 `LCD_Fill` 留的，不是字体自带。

---



## 3. 硬件清单（已验证）


| 模块           | 接口                     | 引脚                                                       | 备注                                         |
| ------------ | ---------------------- | -------------------------------------------------------- | ------------------------------------------ |
| 1.8寸 TFT LCD | SPI1 (Motorola 3-wire) | PB9=SCLK, PB8=PICO, PB10=RES, PB11=DC, PB14=CS, PB26=BLK | SPI 20MHz，依赖 80MHz 主频                      |
| PD42S1 X轴步进  | UART2                  | PB15=TX, PB16=RX                                         | 115200 baud，**NVIC 必须手动** `NVIC_EnableIRQ` |
| PD42S1 Y轴步进  | UART3                  | PB2=TX, PB3=RX                                           | 115200 baud，同上                             |
| 灰度 ADC       | ADC12                  | A27 (PB25)                                               | DMA 模式                                     |
| 按键 KEY1      | GPIO                   | PB00                                                     | 低电平有效 (速度正转)                               |
| 按键 KEY2      | GPIO                   | PB01                                                     | 低电平有效 (速度反转)                               |
| 按键 KEY3      | GPIO                   | PA22                                                     | 低电平有效 (读位置 0x2A)                           |
| 按键 KEY4      | GPIO                   | PB24                                                     | 低电平有效 (触发 SM_zero 回零)                      |
| 按键 KEY5      | GPIO                   | PB21                                                     | 低电平有效 (无限位回零 SM_infzero)                   |
| LED          | GPIO                   | PB22                                                     | 状态指示                                       |
| TB6612 电机 A | GPIO + PWM (TIMG12_CCP0) | AIN1=PB6, AIN2=PB7, PWMA=PB13                          | 编码器 A: PA28/PA31, IN=11 刹车, IN=01 正转 |
| TB6612 电机 B | GPIO + PWM (TIMG12_CCP1) | BIN1=PB23, BIN2=PB27, PWMB=PA25                        | 编码器 B: PB4/PB5                    |
| 编码电机 (L=左) | AB 相正交 | PA=PB4, PB=PB5 (bianma2) | 左电机 L, 4 倍频查表 |
| 编码电机 (R=右) | AB 相正交 | PA=PA28, PB=PA31 (bianma1) | 右电机 R, 4 倍频查表 |
| **MPU-9250 (0.4 第 4 页)** | **I2C0** | **SDA=PA0, SCL=PA1** | **9 轴 IMU (加速度+陀螺+磁力计), MPU6500=0x68 + AK8963=0x0C (bypass 直通), 第 4 页显示物理量** |

**编码器方向 (用户 03:53 反馈)**:
- 物理正转时, `Encoder_CountL -= dir` 让 count **正向增加** (值越大越快), 物理反转时 count 减小 (往负方向走)
- 这是查表结果 `dir` 在 `kEncoderTable` 中已对应好物理方向, `-= dir` 翻转符号
- 命名约定: `L = PB4/PB5 = 左电机编码器`, `R = PA28/PA31 = 右电机编码器`, 永不变

**TB6612 注意点**:
- BIN1=PB23 复用 Huidu MUX_ADC 的 AD2 引脚 (syscfg 里都是 `TB6612_BIN1_B23_IOMUX=51`); 同一时刻只能用 TB6612 页或 Huidu 页,不会两个同时跑
- PWM 周期 2000 / clock 40MHz, 占空比 = compare/2000, 3 档 20%/40%/60% = 400/800/1200
- TB6612_PWM_INST = TIMG12, syscfg 已 enable, 不需要再开
- `TB6612_Run(motor, TB_DIR_FWD)` 内部 `set_in(motor, 0, 1)` (IN1=0, IN2=1) — 软件翻了 IN, 因为物理接线导致"按 K2 电机反"
- 编码电机 ISR 50us 周期挂 `BIANMA2_TIM (TIMA1)`, 需 `NVIC_EnableIRQ(BIANMA2_TIM_INST_INT_IRQN)` 手动 enable
- **编码器引脚必须显式 `direction = "INPUT"` (syscfg 默认 OUTPUT!)**: 用户 03:53 反馈"编码器没有值" 即 PB5/PA31 被配 OUTPUT, 主动拉脚覆盖编码器信号


**已删除**：PA18（BOOT 引脚）作为普通 GPIO 的配置——已确认不影响 SWD 调试。

---



## 4. 时钟配置（empty.syscfg）

- `EXHFMUX = XTAL` / `EXLFMUX = XTAL`
- `PLL_PDIV = 2`, `PLL_QDIV = 5`, `UDIV = 2`
- `HSCLKMUX = SYSPLL2X` → **主频 80MHz**

---



## 5. 烧录链路（JLink 老固件救命脚本）

本项目使用 **JLink 老克隆器**（S/N: 20090928, Hardware V7.00, Firmware 2012）。CPU 锁死（PLL 错配、GPIO 复用冲突、WDT 死锁等导致 JLink halt 失败）时只能用 AIRCR 软复位。

**救命脚本** `/tmp/burn_mspm0.jlink`：

```text
device MSPM0G3507
if SWD
speed 100
connect
r
sleep 200
w4 0xE000ED0C 0x05FA0004
sleep 200
h
loadbin <OUT_FILE> 0
r
g
q
```

**执行**：`/Applications/SEGGER/JLink_V850/JLinkExe -commanderscript /tmp/burn_mspm0.jlink`

**根因**：MSPM0G3507 + JLink 老固件组合下，必须先发 **AIRCR 软复位**（`0xE000ED0C` 写 `0x05FA0004`）才能 halt。

---



## 6. 代码架构

```
0.4/
├── main.c                 # 入口：init + while(1) K5 切页 + 派发 pages[g_page].on_key → SM_Tick → UI_Render
├── empty.syscfg           # SysConfig 配置源（**不要手改，时钟和外设都在这里**）
├── system/                # 系统层
│   └── clock.c / clock.h  # SysTick-based ms 延时 + KEY_Tick 推进
├── user/                  # 用户层 (经常改的代码)
│   └── UI/                # 多页面 LCD 渲染 (UI_Init / UI_Render + pages[])
│       ├── ui.h           # page_t 框架: name + render + on_key + footer_hook
│       └── ui.c           # pages[] 注册表 + MenuPage / EmptyPage / MotorX/YPage / HuiduTBPage / TB6612Page / MPU9250Page 实现
└── Hardware/              # 外设驱动层
    ├── KEY/               # 按键事件型 API
    ├── LED/               # LED 控制
    ├── LCD_Hardware_SPI/  # 1.8寸 TFT LCD 驱动（移植版, 含字库）
    ├── PD42S1/            # PD42S1 闭环步进电机
    │   ├── pd42s1.c / pd42s1.h         # 协议底层
    │   └── stepmotor.c / stepmotor.h  # 高层应用 API (一次调用 = 一个意图)
    ├── Buzzer/            # BEEP_A29 (PA12) 蜂鸣器 (移植自 v1.21)
    ├── Encoder/           # 编码电机 bianma1 (PA28/PA31) + bianma2 (PB4/PB5)
    ├── Huidu/             # 灰度检测 (PB23/PB25/PA24/PA26/PA27, 3 种模式)
    ├── TB6612/            # TB6612 有刷直流电机 (AIN1/2 BIN1/2 GPIO + TIMG12 PWM C0/C1)
    │                      # 3 档占空比 (20/40/60%) + selected 电机指针
    ├── TMC2209/           # TMC2209 步进底层 (共用 PD42S1 UART 引脚, 二选一)
    └── UART_Host/         # 上位机 (UART_0) + K230 (UART_1) 状态机收包
└── Debug/                 # CCS 编译产物 (不要手动 commit)
```

**多页面 UI 架构** (用户 2026-07-15 重构: 菜单 + 详情页两层):

- **两层状态**: `g_page == 0` = 菜单页; `g_page ∈ [1, PAGE_COUNT-1]` = 详情页
- 每页注册到 `pages[]` (`0.4/user/UI/ui.c`):
  ```c
  typedef struct {
      const char *name;          // 页脚功能名 (缺省页脚 "P<n>/<N> <name>" 用)
      void      (*render)(void); // 该页内容渲染
      void      (*on_key)(void); // K1~K4 处理 (内部直接读 key())
      void      (*footer_hook)(void); // NULL → 缺省页脚; 否则页自管 ROW 9 (TB6612 页用)
  } page_t;
  ```
- **当前 pages[] 顺序** (PAGE_COUNT=10):
 - `[0]` MenuPage 菜单页 (g_page=0 入口, K1/K2 切选中, K5 进入)
 - `[1..5]` Task1..5Page 题目页 1..5 (placeholder, "to be added")
 - `[6]` MotorXPage PD42S1 **X 轴** (UART2) 硬件调试
 - `[7]` MotorYPage PD42S1 **Y 轴** (UART3) 硬件调试
 - `[8]` HuiduTBPage 灰度 + TB6612 编码电机
 - `[9]` MPU9250Page 9 轴 IMU
- **菜单项表 kMenuNames[9]**: `TASK1 / TASK2 / TASK3 / TASK4 / TASK5 / MOTOR-X / MOTOR-Y / HUITB / MPU9250` (`g_menu_sel=1..9`, 上电默认 1)
- **K5 路由** (main.c 集中处理):
 - **详情页** K5 → 返回菜单 (`g_page = 0`) + 按页停轴 (`g_page==6→SM_Stop(SM_X)`, `g_page==7→SM_Stop(SM_Y)`) + `TB6612_Stop(A/B)`
 - **菜单页** K5 → 进入选中项 (`g_page = g_menu_sel`)
- **K1/K2 路由**:
 - **菜单页** K1 → 选中上移 (1→9 循环); K2 → 选中下移 (1→9 循环); 详见 `0.4/main.c`
 - **详情页** K1~K4 → 派发 `pages[g_page].on_key()` (菜单页 on_key 是 no-op, 详情页 K1/K2 不会被菜单抢走)
- **菜单页布局** (16 字符/行, 9 项会自动滚动):
 - ROW 0   `   == MENU ==    ` (标题, 白字深蓝底, 严格 15 字符居中填 ' ')
 - ROW 1..8 `>TASK1 / TASK2 / ... / MPU9250` (8 项菜单, 选中项前缀 `>` + YELLOW, 未选 WHITE)
 - ROW 9   `P1/10 MENU` 页脚
- **菜单项自动滚动**: 当选中项 ≥ 6 项时, `first = MAX(0, sel-5)`, 窗口上推让选中项始终可见 (sel=9 → ROW 1..8 全部移到末尾 8 项)
- UI_Render 每帧: `pages[g_page].render()` → 若 `footer_hook` 非空则调之,否则写缺省 `P<cur+1>/<PAGE_COUNT> <name>` → `row_flush`
- 加新详情页: 实现 `render + on_key` (+ 可选 `footer_hook`), 在 `pages[]` 表里加一行, `kMenuNames[]` 加对应字符串, `MENU_ITEM_COUNT` 加 1

**main.c 当前主循环**:

- 上电 `SM_Init()` 两轴使能 + 通信位置模式（**不清零、不落盘**, 零点在上位机已设）
- `while(1)`: K5 down 分流 (菜单页/详情页) → 菜单页 K1/K2 切 `g_menu_sel` → `pages[g_page].on_key()` (K1~K4 派发) → `SM_Tick()` → `Huidu_Task()` → `MPU9250_Task()` → `UI_Render()`
- motor 页 (MotorXPage / MotorYPage): K3 短按 = `SM_ReadPosition` (刷新显示); K3 长按 = `SM_zero(motor, HM)` 触发驱动器回零 (上位机已配零点); K1/K2 按下 = `SM_Move(motor, dir, 100, 60, step_pulses)` (step_pulses 来自 `kMotorStepTenths[s_motor_step_index]`); K1/K2 松开 = `SM_Stop` + `SM_ReadPosition`; K4 循环切换移动精度 (1° → 3° → 5° → 10° → 0.1°)

---



## 7. 应用层 API（速查）

详细用法见 `0.4/Hardware/PD42S1/stepmotor.h` 顶部注释。下面只列"做什么、几个参数"。

### 7.1 运动 API（一次调用 = 一个意图）


| API                                           | 用途                                           |
| --------------------------------------------- | -------------------------------------------- |
| `SM_Init()`                                   | 上电初始化（仅 X/Y 两轴 0xFA 使能 + 0xF3 设 POS_LOOP；不清零、不 SaveParams，上位机已配零点） |
| `SM_Stop(motor)`                              | 立即刹车（STOP_IMM + CLEAR_STATUS 链式, 按 motor 选轴 X/Y 发帧）             |
| `SM_Run(motor, dir, accel, speed)`            | 速度模式（dir=R/L, accel 0-200, speed 0-6000 RPM, 按 motor 选轴）    |
| `SM_Move(motor, dir, accel, speed, pulses)`   | 相对位置脉冲 (按 motor 选轴; 调试页 K1/K2 按当前精度调用此 API)             |
| `SM_MoveTo(motor, dir, accel, speed, pulses)` | 绝对位置脉冲                                       |
| `SM_ResetPosition(motor)`                     | 当前点清零 (0xF8, 按 motor 选轴; **已不在 SM_Init 中调用**)        |
| `SM_Enable(motor, en)`                        | 电机使能/失能 (0xFA, 按 motor 选轴)                            |
| `SM_IsArrived(motor)`                         | 非阻塞到位检查（当前 stub=true）                        |
| `SM_ReadPosition(motor)`                      | 读一次实时位置 (0x2A), 自动重试到应答; 取后写入 `s_axis_pos[motor-1]` |
| `SM_GetPosition(motor)`                       | 读 `s_axis_pos[motor-1]` (motor=X→idx 0, motor=Y→idx 1) 缓存值 |




### 7.2 回零 API


| API                                                   | 用途                                                                                           |
| ----------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `SM_zeroset(motor, origin, timeout_ms, auto_home_on)` | **已停用 (上位机配零点)**: `stepmotor.c` 仍保留入口; 0.4 调试页 K3 短按/长按 不再调此 API |

### 7.6 双轴 PD42S1 收发路由 (`0.4/Hardware/PD42S1/pd42s1.h`)

- **每轴独立状态**: `s_rx_frame[2]`, `s_frame_ready[2]`, `s_rx_buffer[2][]`, `s_rx_index[2]`, `s_tx_buffer[2][]`, `s_tx_buffer_len[2]`, `s_tx_fired[2]`
- `PD42_AXIS_COUNT=2`, `PD42_X_INDEX=0`, `PD42_Y_INDEX=1`, `axis_index(addr) ∈ {0,1}`
- **TX 路由** `PD42S1_SendCommand(addr, ...)`: `addr=1 (SM_X) → x_bujin_INST` (UART2, PB15/PB16); `addr=2 (SM_Y) → y_bujin_INST` (UART3, PB2/PB3)
- **RX 路由** (main.c): `x_bujin_INST_IRQHandler → PD42S1_UART_CallbackFor(SM_X, rx_data)`; `y_bujin_INST_IRQHandler → PD42S1_UART_CallbackFor(SM_Y, rx_data)`
- 调试 API: `PD42S1_TakeFrame / TakeFrameFor / GetFrame / GetFrameFor / GetTxBuffer(addr)` 都带 axis 维度
| `SM_zero(motor, home_mode)`                           | **触发回零**: 单帧 0x92, mode=SINGLE/NEAREST/MULTI, 驱动器自执行                                         |
| `SM_infzero(motor, side, speed, limit_ma)`            | **无限位回零**: 串行 2 帧 0x91 (mode=0/1 左/右无限位) + 0x92; 撞机械结构, 电流 ≥ limit_ma 判堵转停机                  |
| `SM_limithome(motor, side, speed)`                    | **有限位回零**: 串行 2 帧 0x91 (mode=2/3 左/右有限位) + 0x92; 撞对应侧外部限位开关停机                                |


`SM_infzero` / `SM_limithome` 的 dir 由 side 内部推导 (LEFT→CW, RIGHT→CCW), 用户不传。

### 7.3 按键 API（事件型）

`0.4/Hardware/KEY/key.h`，状态机由 `system/clock.c: SysTick_Handler` 每 1ms 推进：

```c
void KEY_Init(void);                                    // 上电一次
void KEY_Tick(void);                                    // SysTick_Handler 已自动调
bool key(uint8_t id, key_type_t type);                  // id=1..5, type=down/up/long_press, 一次性消费
bool key_pressed(uint8_t id);                           // 当前物理电平 (LCD 显示用)
```

**K5 路由**（main.c 处理, 详情页 vs 菜单页分流）:

```c
if (key(5, down)) {
    if (g_page == 0U) {
        /* 菜单页 K5 → 进入选中详情页 */
        g_page = g_menu_sel;
    } else {
        /* 详情页 K5 → 返回菜单 + 切前停电机 */
        SM_Stop(SM_X);
        TB6612_Stop(TB_MOTOR_L);
        TB6612_Stop(TB_MOTOR_R);
        g_page = 0U;
    }
    UI_ForceRedraw();
}

/* 菜单页 K1/K2: 循环切换选中项 (1 ↔ MENU_ITEM_COUNT) */
if (g_page == 0U) {
    if (key(1, down)) { g_menu_sel = (g_menu_sel <= 1U) ? (uint8_t)MENU_ITEM_COUNT : (uint8_t)(g_menu_sel - 1U); }
    if (key(2, down)) { g_menu_sel = (g_menu_sel >= MENU_ITEM_COUNT) ? (uint8_t)1U : (uint8_t)(g_menu_sel + 1U); }
}

pages[g_page].on_key();  /* K1~K4 派发 (菜单页 on_key 是 no-op) */
```

当前 motor 页 (MotorXPage / MotorYPage 共享 `MotorPage_*`) 的 K1~K4 语义 (`0.4/user/UI/ui.c: MotorPage_OnKey`):

```c
const int32_t step_pulses = MotorStepPulses(s_motor_step_index);   // 0.1° → 脉冲 (单位换算见下)
if (key(1, down) && !key_pressed(2))      SM_Move(motor, R, MOTOR_MOVE_ACCEL, MOTOR_MOVE_RPM, step_pulses);
else if (key(2, down) && !key_pressed(1)) SM_Move(motor, L, MOTOR_MOVE_ACCEL, MOTOR_MOVE_RPM, step_pulses);
else if (key(1, up) || key(2, up))        { SM_Stop(motor); SM_ReadPosition(motor); }
else if (key(3, down))                    SM_ReadPosition(motor);                     // 短按: 刷新位置/度数显示
else if (key(3, long_press))              SM_zero(motor, HM);                         // 长按: 驱动器回零 (上位机已配零点)
else if (key(4, down))                    s_motor_step_index = (s_motor_step_index + 1U) % 5U;   // 1°→3°→5°→10°→0.1°
```

**移动精度档** `kMotorStepTenths[] = {10, 30, 50, 100, 1}` (单位 0.1°, 默认 s_motor_step_index=0 → 10×0.1°=1°):
- `step_pulses = (tenths * MOTOR_PULSES_PER_REV + 1800) / 3600` (四舍五入, MOTOR_PULSES_PER_REV=51200)
- 1° ≈ 142 脉冲; 0.1° ≈ 14 脉冲

**MotorX / MotorY 页面布局** (16 字符/行, 9 行内容 + 页脚):
- ROW 0   `M:X/Y k:1 1 1 1` (标识轴 + 键位)
- ROW 1   `POS:+12345 pul` 当前位置 [脉冲, int32, 有符号]
- ROW 2   `DEG:+123.45   ` 当前位置 [度, 0.05°/bit → 2 位小数]
- ROW 3   `TX:C5 01 A8 00` 本帧 TX 头 4 字节 hex
- ROW 4   `RX:C5 01 2A 01` 本帧 RX 头 4 字节 hex (无应答时显示 `RX:---- --- ---`)
- ROW 5..7 留空 (`LCD_Fill` 留白)
- ROW 8   `STEP:1.0°     ` 当前档号 (1.0° / 3.0° / 5.0° / 10.0° / 0.1°)
- ROW 9   缺省页脚 `P<n>/10 MOTOR-X` 或 `MOTOR-Y` (TB6612 页 `footer_hook` 接管 ROW 9)

### 7.4 TB6612 API (`0.4/Hardware/TB6612/tb6612.h`)

2 路有刷直流电机 (电机 A: AIN1=PB6 / AIN2=PB7 / PWMA=PB13; 电机 B: BIN1=PB23 / BIN2=PB27 / PWMB=PA25)。编码器 A=PA28/PA31, 编码器 B=PB4/PB5 (用 `Encoder_GetCountA/B()` 读位置)。

PWM: TIMG12 / CCP0+CCP1, 周期 2000 @ 40MHz = 20kHz, 占空比 = compare/2000。

| API | 用途 |
| ----------------------------------- | ------------------------------------------------------------ |
| `TB6612_Init()`                     | 上电: 两路刹车 (IN1=IN2=1) + PWM=0 + 档=[20,20] + selected=A |
| `TB6612_Stop(motor)`                | 立即刹车 (IN1=IN2=1) + PWM=0                                  |
| `TB6612_Run(motor, dir)`            | 设方向 (FWD=01/REV=10) + 应用当前档 compare                    |
| `TB6612_NextLevel(motor)`           | PWM 档 20→40→60→20 轮转 (作用于 selected 那个)               |
| `TB6612_GetLevel/GetPct(motor)`     | 读当前档 (0/1/2 → 20/40/60)                                   |
| `TB6612_GetSelected()`              | 当前选中电机 (0=A, 1=B) — K4 长按 toggle                       |
| `TB6612_ToggleSelected()`           | 切换 selected                                                  |

**真值表**:
| IN1 | IN2 | 输出   | 用途        |
| --- | --- | ------ | ----------- |
| 0   | 0   | 滑行   | (未使用)     |
| 0   | 1   | 反转   | `TB_DIR_REV` |
| 1   | 0   | 正转   | `TB_DIR_FWD` |
| 1   | 1   | 刹车   | Init/Stop    |

**TB6612 页按键语义** (`0.4/user/UI/ui.c: TB6612Page_OnKey`):
| 按键       | 动作                                                            |
| ---------- | -------------------------------------------------------------- |
| K2 down/up | 两电机同时 `Run(FWD)` / `Stop`                                   |
| K3 down/up | 两电机同时 `Run(REV)` / `Stop`                                   |
| K4 短按    | `NextLevel(selected)` — 切当前选中电机的 PWM 档 (互不影响两路)        |
| K4 长按    | `ToggleSelected()` — 切换"短按 K4 影响哪一路"                       |
| K1         | no-op (留空)                                                     |
| K5         | 全局切页, `main.c` 副作用 `TB6612_Stop(A) + TB6612_Stop(B)`           |

**为什么要 selected**: K4 短按"分别给不同编码器切换 pwm" (用户 02:30 原话) — 两路 PWM 档独立维护, 长按 K4 在两路间切换 selected。要测 "A 在 20% / B 在 60%" 先短按 K4 把 A 设 20% → 长按 K4 切到 B → 短按 K4 把 B 设 60% → K2 两路一起转。

**TB6612 页布局** (16 字符/行硬约束):
| 行 | y | 内容                                              |
| ---- | ----- | ----------------------------------------------- |
| ROW 0 | 0     | `k:1 1 1 1 1`                                    |
| ROW 1..6 | 16..96 | 清空 (后续可扩展)                              |
| ROW 7 | 112   | `SEL:>A 60 B 20` (15 字符) — selected 加 `>`     |
| ROW 8 | 128   | `E1:+12345 E2:-6789` (16 字符) — 编码器累计值    |
| ROW 9 | 144   | 自定义页脚 `PWM:60% P3/3ENC` (15 字符)            |

**ROW 9 由 `footer_hook` 接管**: TB6612 页注册时 `footer_hook=TB6612Page_Footer`, `UI_Render` 末尾检测到非 NULL 就调它 (里面 `row_put(9, ...)`), 其他页走缺省 `P<n>/<N> <name>`。

### 7.5 MPU9250 API (`0.4/Hardware/MPU9250/mpu9250.h`)

9 轴 IMU (加速度 + 陀螺 + 磁力计), I2C0 controller, **不依赖 NVIC 中断** (driverlib 走纯阻塞 + BUSY 位等 STOP)。

- **MPU6500**: I2C 地址 0x68, WHO_AM_I 期望 0x71 (真品) 或 0x70 (无磁力计假片)
- **AK8963**: I2C 地址 0x0C, WHO_AM_I 期望 0x48, 需先开 INT_PIN_CFG |= 0x02 (BYPASS_EN) + USER_CTRL &= ~0x20

**API** (`mpu9250.h`):

| API | 用途 |
| --- | --- |
| `MPU9250_Init()` | 上电: 配 MPU6500 + 开 bypass + 配 AK8963 16-bit/100Hz + 读工厂校准. 返回 false=I2C 总线没接 |
| `MPU9250_Task()` | 每帧调, 内部 100ms 节流 (10Hz); 读 9 轴 + 算姿态 |
| `MPU9250_GetData()` | 返回 `MPU9250_Data *` 指针, 含 ax/ay/az(g), gx/gy/gz(dps), mx/my/mz(uT), yaw/pitch/roll(deg), temp_c, who_mpu, who_mag, ok, **bias_gx/y/z + calib_state** |
| `MPU9250_ResetGyroCalib()` | K1 down 触发, 重新进入 CAL_RUNNING, 累积新 bias |

**量纲换算** (FS 默认 ±2g / ±250dps / AK 16-bit):

```
acc [g]    = raw / 16384
gyro [dps] = raw / 131
mag  [uT]  = raw * (0.15 * adj),  adj = (ASA - 128) / 256 + 1
temp [°C]  = raw / 340 + 21
```

**MPU9250 页布局** (第 4 页, 16 字符/行硬约束, 用户 2026-07-14 重构):

| 行 | y | 内容 |
| --- | --- | --- |
| ROW 0 | 0 | `k:1 1 1 1 1` |
| ROW 1 | 16 | `X     Y     Z   ` XYZ 标签独立行 |
| ROW 2 | 32 | `+1.20 +0.34 -0.98` 加速度 XYZ [g] (5.2f × 3) |
| ROW 3 | 48 | `+1.24 +4.47 +0.12` 陀螺 XYZ [dps] (校准后; 校准中显示 `cal...`) |
| ROW 4 | 64 | `+12.3 -4.5 +6.7` 磁力计 XYZ [uT] (hard-iron 后; 失败显示 `no mag`) |
| ROW 5 | 80 | `w+060.0+000.0 360.0` **(用户 05:36 新增) 累计欧拉 [°], 上电为 0** |
| ROW 6 | 96 | `Y     P    R` 姿态标签 |
| ROW 7 | 112 | `+45.3 -12.4 +5.7` yaw/pitch/roll 绝对姿态 [deg] (磁力校准, 抗零漂) |
| ROW 8 | 128 | `T:+27.3 CAL=2` 内部温度 + 校准状态 (从原 ROW 7 下移) |
| ROW 9 | 144 | 页脚 `P4/4 MPU9250` |

**euler_x/y/z 抗漂机制** (用户 05:58 反馈: "上电就 4,15,357" → 加了 2 个门控):
- **门控 1**: `s_calib == CAL_OK` — 校准期间不累加 (校准期 g 是原始零漂 1.3 dps, 1 秒能累积 4°)
- **门控 2**: `!s_motionless` — 静止冻结 (|g|<0.5 dps 持续 200ms 视为静止)
  - 静止时不累加 (不清零, 锁定当前值)
  - 旋转时立刻恢复累加
- **效果**: 静止时 ROW 5 完全锁定, 旋转时实时累加
- 串口 1Hz 头行加 `EUL x=... y=... z=... ST=0/1`, 看静止是否冻住

**按键语义** (用户 2026-07-14 决定):
- K1 down → `MPU9250_ResetGyroCalib()` 重新采集陀螺零漂 (板子必须静止 1 秒)
- K2..K4 → no-op (后续 hard-iron / yaw reset 预留位)
- K5 → main.c 切页

**陀螺零漂校准** (用户 2026-07-14 决定: 上电自动 + slow decay):
- 上电后 1 秒 (200 帧 @ 5ms/tick) 累积 gx/gy/gz 平均 → bias_gx/y/z
- CAL_OK 后 60 秒才启用 slow decay, 每 30 秒 bias ← bias × 0.95 + current × 0.05
- UI: ROW 3 校准中显示 `cal...`, 完成后显示数值; ROW 8 `CAL=2`; ROW 5 `w` 用 bias 减过的 gx/gy/gz 积分

**Hard-iron** (用户 2026-07-14 决定: 不做运行时校准, 看串口数据手填):
- `s_hard_iron_{x,y,z}` 默认 0, 应用: `mag_cal = mag_raw * adj - hard_iron`
- 用户看串口 1Hz 输出 `RAW mx/my/mz` + `CAL mx'/my'/mz'` 对比, 把对称中点 (mx_max+mx_min)/2 等手填常量
- 后续比赛调试阶段按实际环境调整

**串口调试** (用户 2026-07-14 决定: 1Hz, 校准前+后+姿态):
- UART_0 (PA10/PA11), 1Hz 节流, 直接调 `DL_UART_Main_transmitData` (不依赖 `UART_Host`)
- 格式: `T:ms CAL=x BIAS=...,...,...` / `RAW ax=... ay=... ...` / `CAL ax=... ...` / `ATT yaw=... pit=... rol=... T=...`
- 接收: 串口助手, baud 看 syscfg (注释 9600, syscfg 默认 115200)

**`MPU9250_Task` 内部状态机** (现已是 ISR 推进): 状态机由 MPU9250_TIM (TIMG6, 5ms ISR) 推进, 每帧推进 1 个 I2C 事务. UI 通过 `MPU9250_GetData()` 读最近一帧快照.

**失败处理** (用户 2026-07-13 决定: 自适应等待, 不卡死):
- I2C 总线没接 / MPU9250 不存在 → `MPU9250_Init` 返回 false, `ok=false` → ROW 2..7 显示 `  no dev`
- MPU9250 真品但 AK8963 失败 (假芯片 / 焊坏) → `who_mag=0`, ROW 4..5 显示 `no mag`, ROW 7 显示 `mag:0 NO`
- 真品 9 轴全通 → `who_mpu=0x71`, `who_mag=0x48`, ROW 7 显示 `71 48 B:+1.23`

**后续可扩展** (在 `mpu9250.c` `MPU9250_Task` 末尾注释里预留位置):
- 加 complementary / kalman filter 融合 yaw 长时间精度
- yaw reset (按 K4 把当前方向存为 0 偏移)
- Hard-iron 运行时校准 (K2 启动 8 字旋转 8 秒)


---



## 8. PD42S1 协议

正点原子自定义 SMD 协议，**8 位校验和**（不是 CRC16）。

- 帧格式: `[C5][ADDR][FUNC][DATA...][CHECKSUM][5C]`
- CHECKSUM = `sum(C5..DATA最后一字节) & 0xFF`，**包含帧头 C5**
- 读位置 (0x2A) 应答: `[C5][01][2A][ERR][POS_B3][POS_B2][POS_B1][POS_B0][CHECKSUM][5C]`，共 10 字节
  - ERR=0x01 表示成功，位置在 `data[1..4]` (int32 big-endian，51200 = 一圈)
- 电机使能 (0xFA): DATA=0 使能, DATA=1 失能
- 协议约束: 相邻命令间隔 ≥ 10ms（由 `stepmotor.c` 的节流器自动保证）

所有协议帧收发都集中在 `pd42s1.c`，应用层**不**直接动 `g_tx_buffer` 或 UART。

---



## 9. 工作约定



### 9.1 修改代码前的流程

1. **读** `LOG.md` 看最近的修改记录和已知问题
2. 确认改动是否符合当前架构
3. 改完代码后，**必须更新** `LOG.md`（追加新条目，不删除旧条目）



### 9.2 必须更新 LOG.md 的内容

- 添加 / 删除 / 修改任何 `.c` / `.h` 文件
- 修改 `empty.syscfg`（时钟、外设、引脚）
- 解决了一个 bug
- 添加了新功能
- 硬件 / 烧录链路改动



### 9.3 不需要更新 LOG.md 的

- 格式化 / 重命名变量 / 改注释
- 修编译警告（除非有特别说明）



### 9.4 目录约定

- `0.4/system/` 系统层 / `0.4/user/` 用户层 / `0.4/Hardware/<模块名>/` 驱动层
- `LOG.md` 不进 git（私人日志）；`AGENTS.md` 进 git（共享）

---



## 10. 已知问题 / 注意事项

- **SysConfig 不调用** `NVIC_EnableIRQ`：实测 syscfg 只 `NVIC_SetPriority`，**不会 enable 中断**。`SYSCFG_DL_init` 之后必须手写 `NVIC_EnableIRQ(x_bujin_INST_INT_IRQN); NVIC_EnableIRQ(y_bujin_INST_INT_IRQN); NVIC_EnableIRQ(BIANMA2_TIM_INST_INT_IRQN); NVIC_EnableIRQ(MPU9250_TIM_INST_INT_IRQN);` 才能进 ISR。
- **Y 轴 RX 已接管**：`y_bujin_INST_IRQHandler → PD42S1_UART_CallbackFor(SM_Y, rx_data)`，与 X 轴共用校验/拼帧逻辑 (`s_frame_ready[1]`、`s_axis_pos[1]`)。上位机在 0xFA/0xF3 阶段必须给 Y 轴驱动器同样发一帧 init，否则会一直 `RX:----`。
- **PD42S1 RX 校验**：协议是 8 位校验和 `sum(HEAD..CHK前一字节) & 0xFF`（**包含帧头**）。`pd42s1.c:checksum8()` 和 `chk_data_len = frame_len - 2` 必须保持一致。
- **未实现 BSL 应急恢复**：CPU 再次锁死只能靠 JLink AIRCR 软复位。
- **步进电机底层接口**（`stepmotor.c` 中 `SM_DIR_REVERSE` 保险宏、`SM_Run/SM_Move/MoveTo` 的 dir 翻转）由底层 stepmotor.h 文档控制，应用层不直接动。

---



## 11. 调试常用命令

```bash
# 进入工程目录
cd /Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI

# 烧录（首次或 CPU 锁死时）
/Applications/SEGGER/JLink_V850/JLinkExe -commanderscript /tmp/burn_mspm0.jlink

# 查看 git 状态
git status

# 查看最近的修改日志
tail -100 LOG.md
```

