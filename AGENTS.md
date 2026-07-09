# AGENTS.md - 给 AI 助手的项目说明

> **本文件会随每次重要改动更新。新对话窗口请先读本文件 + `LOG.md`。**

---

## 1. 项目背景

- **赛事**：2026 TI 电赛（嵌入式赛道）
- **平台**：MSPM0G3507 (LQFP-64)，TI MSPM0 SDK 2.10.00.04
- **当前工程版本**：`0.1/`
- **IDE**：CCS (Code Composer Studio)，SysConfig 图形化配置
- **主机**：macOS 15 (darwin 25.5.0)
- **仓库路径**：`/Volumes/Moving/work/01_code/01_Competition/01_ELE_com/2026-TI-Em/2026-TI`

---

## 2. 硬件清单（已验证）

| 模块 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| 1.8寸 TFT LCD | SPI1 (Motorola 3-wire) | PB9=SCLK, PB8=PICO, PB10=RES, PB11=DC, PB14=CS, PB26=BLK | SPI 20MHz，依赖 80MHz 主频 |
| PD42S1 X轴步进 | UART2 | PB15=TX, PB16=RX | 115200 baud |
| PD42S1 Y轴步进 | UART3 | PB2=TX, PB3=RX | 115200 baud |
| 灰度 ADC | ADC12 | A27 (PB25) | DMA 模式 |
| 按键 KEY1 | GPIO | PB00 | 低电平有效 |
| 按键 KEY2 | GPIO | PB01 | 低电平有效 |
| LED | GPIO | PB22 | 状态指示 |

**已删除**：PA18（BOOT 引脚）作为普通 GPIO 的配置——已确认不影响 SWD 调试。

---

## 3. 时钟配置（empty.syscfg）

- `EXHFMUX = XTAL` （外部高频晶振）
- `EXLFMUX = XTAL` （外部低频晶振）
- `PLL_PDIV = 2`
- `PLL_QDIV = 5`
- `UDIV = 2`
- `HSCLKMUX = SYSPLL2X` → **主频 80MHz**

---

## 4. 烧录链路（JLink 老固件救命脚本）

**重要**：本项目使用 **JLink 老克隆器**（S/N: 20090928, Hardware V7.00, Firmware 2012）。CCS 和 JLink 默认烧录在以下情况会失败：

- 上一次烧录的代码有 bug（PLL 错配、GPIO 复用冲突、WDT 死锁等）
- CPU 进入"看起来在跑，但 JLink halt 失败"的状态

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

## 5. 代码架构

```
0.1/
├── main.c                # 主循环，按键演示电机控制
├── clock.c / clock.h     # SysTick-based ms 延时（LCD 初始化需要精确延时）
├── empty.syscfg          # SysConfig 配置源（**不要手改，时钟和外设都在这里**）
├── Hardware/
│   ├── LCD_Hardware_SPI/ # 1.8寸 TFT LCD 驱动（移植版）
│   │   ├── LCD.c / LCD.h
│   │   └── LCD_Data.c / LCD_Data.h    # 字库和图片
│   └── PD42S1/           # PD42S1 闭环步进电机 SMD 协议
│       ├── pd42s1.c / pd42s1.h
└── Debug/                # CCS 编译产物
    └── 0.1.out           # 烧录目标文件
```

**main.c 中的 demo 函数**：
- `demo_relative_move()`：X 轴相对位置控制（KEY1 触发）
- `demo_absolute_move()`：XY 轴绝对位置控制（KEY2 触发）
- `demo_speed_mode()`：速度模式（未启用）
- `demo_reciprocal_move()`：往返运动（未启用）
- `demo_motor_enable()`：使能测试（未启用）

---

## 6. 已实现的功能（截至最新一次更新）

- ✅ 系统时钟 80MHz（SYSPLL2X）
- ✅ LCD 1.8寸 TFT SPI 显示（蓝色背景，3 行文字 + 1 红点）
- ✅ PD42S1 X/Y 轴 UART 通信初始化
- ✅ PD42S1 速度模式 / 相对位置 / 绝对位置指令封装
- ✅ 按键 KEY1/KEY2 触发演示
- ✅ LED 心跳闪烁
- ✅ AIRCR 软复位烧录链路（解决 JLink 老固件 halt 失败）

---

## 7. 工作约定

### 7.1 修改代码前的流程
1. **读 LOG.md** 看最近的修改记录和已知问题
2. 确认改动是否符合当前架构
3. 改完代码后，**必须更新 LOG.md**（追加新条目，不删除旧条目）

### 7.2 修改哪些内容需要更新 LOG.md
- ✅ 添加 / 删除 / 修改任何 `.c` / `.h` 文件
- ✅ 修改 `empty.syscfg`（时钟、外设、引脚）
- ✅ 解决了一个 bug
- ✅ 添加了新功能
- ✅ 硬件 / 烧录链路改动
- ✅ 更新了三方驱动 / SysConfig 配置

### 7.3 不需要更新 LOG.md 的
- 格式化 / 重命名变量
- 改注释
- 修编译警告（除非有特别说明）

---

## 8. 目录和文件约定

- **源码**：放在 `0.1/` 下或 `0.1/Hardware/<模块名>/`
- **日志**：根目录 `LOG.md`（**不进 git**，本机私有）
- **AGENTS**：根目录 `AGENTS.md`（**进 git**，所有协作者共享）
- **CCS 元数据**（`.ccsproject`、`.cproject`、`.settings/`）可以进 git
- **Debug 产物**（`.out`、`.o`、`.d`、`.map`）**不要**手动 commit，CCS 会自动维护

---

## 9. 已知问题（待跟进）

- **PD42S1 应答处理未启用**：UART2 中断已注册 `PD42S1_UART_Callback`，但 `PD42S1_WaitResponse` 和 `PD42S1_GetFrame` 的使用流程还没在 demo 里串通。电机指令是"fire-and-forget"。
- **Y 轴 callback 是空的**：`y_bujin_INST_IRQHandler` 中 `(void)rx_data;` 直接丢弃——Y 轴驱动器暂未驱动。
- **未实现 BSL 应急恢复**：一旦再次锁死，只能靠 JLink AIRCR 软复位这条路。

---

## 10. 调试常用命令

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

---

**最后更新**：见 `LOG.md` 末尾的"最后更新"字段