# AGENTS.md - 给 AI 助手的项目说明

> **新对话窗口请先读本文件 + `LOG.md`。本文件只保留协作必需的稳态信息，过程记录见 `LOG.md`。**

---

## 1. 项目背景

- **赛事**：2026 TI 电赛（嵌入式赛道）
- **平台**：MSPM0G3507 (LQFP-64)，TI MSPM0 SDK 2.10.00.04
- **工程版本**：`0.1/`
- **IDE**：CCS (Code Composer Studio)，SysConfig 图形化配置
- **主机**：macOS 15 (darwin 25.5.0)
- **仓库路径**：`/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI`

---

## 2. LCD 渲染约束（最容易忘）

**1.8寸 TFT LCD, 竖屏 128×160, `USE_HORIZONTAL=0`, 字体 `LCD_8X16` (8 像素宽 × 16 像素高)。**

- **每行最多 16 个字符**（128 / 8 = 16）
- **最多 10 行**（160 / 16 = 10）
- 行高 16 px, 行间距 2 px → **每行占 18 px**

写 UI 时所有 `snprintf` / `LCD_ShowString` 必须按 16 字符上限规划，**不许超界**（超出会显示到下一行或被截断, 且无运行时检查）。

当前布局参考 `0.1/user/UI/ui.c`：6 行（y=0/18/36/54/72/90），下方 (y≥108) 留空。

---

## 3. 硬件清单（已验证）

| 模块 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| 1.8寸 TFT LCD | SPI1 (Motorola 3-wire) | PB9=SCLK, PB8=PICO, PB10=RES, PB11=DC, PB14=CS, PB26=BLK | SPI 20MHz，依赖 80MHz 主频 |
| PD42S1 X轴步进 | UART2 | PB15=TX, PB16=RX | 115200 baud，**NVIC 必须手动 `NVIC_EnableIRQ`** |
| PD42S1 Y轴步进 | UART3 | PB2=TX, PB3=RX | 115200 baud，同上 |
| 灰度 ADC | ADC12 | A27 (PB25) | DMA 模式 |
| 按键 KEY1 | GPIO | PB00 | 低电平有效 (速度正转) |
| 按键 KEY2 | GPIO | PB01 | 低电平有效 (速度反转) |
| 按键 KEY3 | GPIO | PA22 | 低电平有效 (读位置 0x2A) |
| 按键 KEY4 | GPIO | PB24 | 低电平有效 (触发 SM_zero 回零) |
| 按键 KEY5 | GPIO | PB21 | 低电平有效 (无限位回零 SM_infzero) |
| LED | GPIO | PB22 | 状态指示 |

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
0.1/
├── main.c                # 入口：init + while(1) 按键→意图→UI_Render
├── empty.syscfg          # SysConfig 配置源（**不要手改，时钟和外设都在这里**）
├── system/               # 系统层
│   └── clock.c / clock.h # SysTick-based ms 延时 + KEY_Tick 推进
├── user/                 # 用户层 (经常改的代码)
│   └── UI/               # LCD 渲染 (UI_Render)
└── Hardware/             # 外设驱动层
    ├── KEY/              # 按键事件型 API
    ├── LED/              # LED 控制
    ├── LCD_Hardware_SPI/ # 1.8寸 TFT LCD 驱动（移植版, 含字库）
    └── PD42S1/           # PD42S1 闭环步进电机
        ├── pd42s1.c / pd42s1.h     # 协议底层
        └── stepmotor.c / stepmotor.h  # 高层应用 API (一次调用 = 一个意图)
└── Debug/                # CCS 编译产物 (不要手动 commit)
```

**main.c 当前主循环**：
- 上电 `SM_Init()` 使能电机 + 工作模式 + 位置清零
- `while(1)` 按键 → 运动意图；松开触发刹车；`SM_Tick()` 推进节流；`UI_Render()` 刷 LCD

---

## 7. 应用层 API（速查）

详细用法见 `0.1/Hardware/PD42S1/stepmotor.h` 顶部注释。下面只列"做什么、几个参数"。

### 7.1 运动 API（一次调用 = 一个意图）

| API | 用途 |
|-----|------|
| `SM_Init()`              | 上电初始化（使能 + 模式 + 清零） |
| `SM_Stop(motor)`         | 立即刹车（STOP_IMM + CLEAR_STATUS 链式） |
| `SM_Run(motor, dir, accel, speed)` | 速度模式（dir=R/L, accel 0-200, speed 0-6000 RPM） |
| `SM_Move(motor, dir, accel, speed, pulses)` | 相对位置脉冲 |
| `SM_MoveTo(motor, dir, accel, speed, pulses)` | 绝对位置脉冲 |
| `SM_ResetPosition(motor)` | 当前点清零 (0xF8) |
| `SM_Enable(motor, en)`    | 电机使能/失能 (0xFA) |
| `SM_IsArrived(motor)`     | 非阻塞到位检查（当前 stub=true） |
| `SM_ReadPosition(motor)` | 读一次实时位置 (0x2A), 自动重试到应答 |

### 7.2 回零 API

| API | 用途 |
|-----|------|
| `SM_zeroset(motor, origin, timeout_ms, auto_home_on)` | **设当前点为限位原点**: 串行 5 帧 0xF8 + 0x90/0x98 + 0x95 + 0x97 + 0x04 (SaveParams 落盘), 节流器 10ms 间隔自动发完 |
| `SM_zero(motor, home_mode)` | **触发回零**: 单帧 0x92, mode=SINGLE/NEAREST/MULTI, 驱动器自执行 |
| `SM_infzero(motor, side, speed, limit_ma)` | **无限位回零**: 串行 2 帧 0x91 (mode=0/1 左/右无限位) + 0x92; 撞机械结构, 电流 ≥ limit_ma 判堵转停机 |
| `SM_limithome(motor, side, speed)` | **有限位回零**: 串行 2 帧 0x91 (mode=2/3 左/右有限位) + 0x92; 撞对应侧外部限位开关停机 |

`SM_infzero` / `SM_limithome` 的 dir 由 side 内部推导 (LEFT→CW, RIGHT→CCW), 用户不传。

### 7.3 按键 API（事件型）

`0.1/Hardware/KEY/key.h`，状态机由 `system/clock.c: SysTick_Handler` 每 1ms 推进：

```c
void KEY_Init(void);                                    // 上电一次
void KEY_Tick(void);                                    // SysTick_Handler 已自动调
bool key(uint8_t id, key_type_t type);                  // id=1..5, type=down/up/long_press, 一次性消费
bool key_pressed(uint8_t id);                           // 当前物理电平 (LCD 显示用)
```

主循环示例（速度模式）：

```c
if (key(1, down) && !key_pressed(2))      SM_Run(SM_X, R, ACCEL, RPM);
else if (key(2, down) && !key_pressed(1)) SM_Run(SM_X, L, ACCEL, RPM);
else if (key(1, up) || key(2, up))        SM_Stop(SM_X);
else if (key(3, down))                    SM_ReadPosition(SM_X);
```

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
1. **读 `LOG.md`** 看最近的修改记录和已知问题
2. 确认改动是否符合当前架构
3. 改完代码后，**必须更新 `LOG.md`**（追加新条目，不删除旧条目）

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
- `0.1/system/` 系统层 / `0.1/user/` 用户层 / `0.1/Hardware/<模块名>/` 驱动层
- `LOG.md` 不进 git（私人日志）；`AGENTS.md` 进 git（共享）

---

## 10. 已知问题 / 注意事项

- **SysConfig 不调用 `NVIC_EnableIRQ`**：实测 syscfg 只 `NVIC_SetPriority`，**不会 enable 中断**。`SYSCFG_DL_init` 之后必须手写 `NVIC_EnableIRQ(x_bujin_INST_INT_IRQN); NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);` 才能进 ISR。
- **Y 轴 callback 是空的**：`y_bujin_INST_IRQHandler` 中 `(void)rx_data;` 直接丢弃——Y 轴驱动器暂未驱动。
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