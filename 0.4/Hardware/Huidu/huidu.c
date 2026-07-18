/* ============================================================================
 * @file    huidu.c
 * @brief   灰度传感器驱动 (0.2 灰度板, 3 种模式可软切换)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     uint16_t w[8] = {3000,3000,...};    // 标定白值 (8 路 ADC)
 *     uint16_t b[8] = {500, 500,...};     // 标定黑值
 *     Huidu_Init(w, b);                  // 重配引脚 + 算归一化系数
 *
 *   主循环每帧调一次:
 *     Huidu_Task();                      // 按当前模式扫一轮 + 二值化 + 归一化
 *     uint8_t  d = Huidu_GetDigital();   // 8 位二值化 (bit7=CH8, bit0=CH1)
 *     uint16_t raw[8];
 *     Huidu_GetAnalog(raw);              // 8 路 ADC 原始值 (MUX_ADC 模式有意义)
 *
 *   模式切换 (运行时动态, 重配引脚方向):
 *     Huidu_SetMode(HUIDU_MODE_MUX_ADC);     // 默认: 8 路 MUX+ADC
 *     Huidu_SetMode(HUIDU_MODE_FIRST5_GPIO); // 5 路 GPIO
 *     Huidu_SetMode(HUIDU_MODE_ALL8_GPIO);   // 8 路 GPIO
 *     Huidu_NextMode();                       // 轮转 MUX_ADC → 5GPIO → 8GPIO → MUX_ADC
 *
 * ============================================================================
 * 硬件 (0.2 灰度板 - 按用户提供的实物引脚核对)
 *   CN8 10P 灰度排线 (板子丝印 → MCU 引脚):
 *     1: +5V
 *     2: A27 → PA27   = ADC 输入 (syscfg huidu 通道)
 *     3: B25 → PB25   = MUX EN (低电平有效, 0.2 板反相)
 *     4: B19 → PB19   = MUX AD2 地址线 (最高位)
 *     5: A24 → PA24   = MUX AD1 地址线
 *     6: A26 → PA26   = MUX AD0 地址线
 *     7: A14 → PA14
 *     8: A07 → PA07
 *     9: B12 → PB12
 *    10: GND
 *
 *   注意: PB23 (PINCM51) 是 TB6612 BIN1 电机驱动脚, **不是** AD2 (0.1 syscfg
 *   注释写错为 PB23, 实际硬件 B19 排线脚对应 PB19 = PINCM45)
 * ============================================================================
 * 3 种模式 (运行时软切换, 调用 Huidu_SetMode 重新初始化引脚方向)
 *   HUIDU_MODE_MUX_ADC    : 8 路传感器走 ADC 分时复用 (默认, 全功能)
 *   HUIDU_MODE_FIRST5_GPIO: 只读 GPIO, 物理 1/2/3/4/5 共 5 路 (PA24/PA21/PB23/PB20/PB25)
 *   HUIDU_MODE_ALL8_GPIO  : 读全部 GPIO, 物理 1..8 共 8 路
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/Huidu/huidu.h"
#include <string.h>

/* ============================================================================
 * 引脚硬编 (不依赖 syscfg 宏)
 * ============================================================================ */

/* ADC 输入: SysConfig 已配 huidu ADC 通道 */
#define HUIDU_ADC_INST          huidu_INST
#define HUIDU_ADC_RESULT_REG    huidu_ADCMEM_1_A27

/* 8 路灰度引脚 (按用户给的实物顺序 1..8)
 *   i=0→CH1=A27(PA27 ADC), i=1→CH2=B25(PB25 EN),
 *   i=2→CH3=B19(PB19 AD2), i=3→CH4=A24(PA24 AD1),
 *   i=4→CH5=A26(PA26 AD0), i=5→CH6=A14(PA14),
 *   i=6→CH7=A07(PA07), i=7→CH8=B12(PB12)
 *
 *   PA27 是 huidu ADC (syscfg 已配 PINCM60 为 ADC), 这里仍按 GPIO 列出:
 *   GPIO 模式下直接读 PA27 的数字电平 (灰度板 A27 是数字输出)
 */
typedef struct {
    GPIO_Regs* port;
    uint32_t   pin;
    uint32_t   iomux;          /* IOMUX_PINCMxx */
    uint32_t   pf;             /* IOMUX_PINCMxx_PF_GPIO?_DIOxx */
} huidu_gpio_pin_t;

static const huidu_gpio_pin_t kGrayGpio[8] = {
    /* CH1 ADC  */ { GPIOA, DL_GPIO_PIN_27, IOMUX_PINCM60, IOMUX_PINCM60_PF_GPIOA_DIO27 }, /* PA27 (A27) */
    /* CH2 EN   */ { GPIOB, DL_GPIO_PIN_25, IOMUX_PINCM56, IOMUX_PINCM56_PF_GPIOB_DIO25 }, /* PB25 (B25) */
    /* CH3 AD2  */ { GPIOB, DL_GPIO_PIN_19, IOMUX_PINCM45, IOMUX_PINCM45_PF_GPIOB_DIO19 }, /* PB19 (B19) */
    /* CH4 AD1  */ { GPIOA, DL_GPIO_PIN_24, IOMUX_PINCM54, IOMUX_PINCM54_PF_GPIOA_DIO24 }, /* PA24 (A24) */
    /* CH5 AD0  */ { GPIOA, DL_GPIO_PIN_26, IOMUX_PINCM59, IOMUX_PINCM59_PF_GPIOA_DIO26 }, /* PA26 (A26) */
    /* CH6      */ { GPIOA, DL_GPIO_PIN_14, IOMUX_PINCM36, IOMUX_PINCM36_PF_GPIOA_DIO14 }, /* PA14 (A14) */
    /* CH7      */ { GPIOA, DL_GPIO_PIN_7,  IOMUX_PINCM14, IOMUX_PINCM14_PF_GPIOA_DIO07 }, /* PA07 (A07) */
    /* CH8      */ { GPIOB, DL_GPIO_PIN_12, IOMUX_PINCM29, IOMUX_PINCM29_PF_GPIOB_DIO12 }, /* PB12 (B12) */
};

#define MUX_EN_PORT     (kGrayGpio[1].port)  /* PB25 - CH2 B25 */
#define MUX_EN_PIN      (kGrayGpio[1].pin)
#define MUX_AD0_PORT    (kGrayGpio[4].port)  /* PA26 - CH5 A26 */
#define MUX_AD0_PIN     (kGrayGpio[4].pin)
#define MUX_AD1_PORT    (kGrayGpio[3].port)  /* PA24 - CH4 A24 */
#define MUX_AD1_PIN     (kGrayGpio[3].pin)
#define MUX_AD2_PORT    (kGrayGpio[2].port)  /* PB19 - CH3 B19 (不是 PB23, 那是 BIN1) */
#define MUX_AD2_PIN     (kGrayGpio[2].pin)

/* 方向反转: 关掉, 让 i 直接对应 CH i+1 (CH1→Analog_value[0]→bit0, CH8→Analog_value[7]→bit7)
 *   ROW 0 显示按 bit7..bit0 (高位在前), 但用户视角的 CH1 在最右 */
#define HUIDU_DIRECTION_REVERSE   0

/* 全局变量 */
huidu_mode_t     Huidu_Mode = HUIDU_MODE_MUX_ADC;
Huidu_Sensor_t   g_huidu_sensor = {0};

/* ============================================================================
 * ADC 读取 (阻塞同步, 80MHz 主频下约 10us)
 * ============================================================================ */
static uint16_t adc_read_value(void) {
    /* 启动一次转换, 阻塞等到 ADC 进入 IDLE (转换完成) 再读结果
     *
     * 历史坑: 之前是 startConversion + 等 10us NOP 直接读结果寄存器, 没有检查
     *   转换完成标志. 当 MUX 切地址后立刻 start, ADC 还在采上一次的尾音, 导致
     *   读到旧值 → 8 路全部 ≈ 700 (上一次黑的值), 偶尔正常是时序漂移时对了.
     *
     * 80MHz 主频下 12 位 ADC 转换约 13 cycles sample + 12 cycles conv ≈ 25 cycles
     *   = 0.3us; 但带 bus clock 同步 + MEM 写入延迟实际 ~1-2us. 加重试兜底.
     *
     * MSPM0 SDK ADC12 API 没有"isConversionComplete", 用 STATUS 寄存器轮询
     *   BUSY 字段 (DL_ADC12_STATUS_CONVERSION_ACTIVE = 转换中, IDLE = 完成).
     */
    DL_ADC12_startConversion(HUIDU_ADC_INST);
    uint32_t timeout = 1000U;  /* ~12us @ 80MHz NOP, 远大于 ADC 转换时间 */
    while ((DL_ADC12_getStatus(HUIDU_ADC_INST) & ADC12_STATUS_BUSY_ACTIVE) != 0U) {
        if (--timeout == 0U) break;
        __NOP();
    }
    return DL_ADC12_getMemResult(HUIDU_ADC_INST, HUIDU_ADC_RESULT_REG);
}

static void delay_us_about(uint32_t us) {
    volatile uint32_t n = us * 80;
    while (n--) { __NOP(); }
}

/* ============================================================================
 * MUX 地址线 / EN 控制
 * ============================================================================ */
static void mux_enable(bool en) {
    /* 0.2 板实测: MUX_EN 低电平有效 (高电平时 MUX 输出被强制关闭, ADC 读到悬空/0)
     *   上电 EN 默认是高 (MUX 不工作), 需要 clearPins (拉低) 才能选通
     *   高电平反而全部关闭 → 早期误以为"插上使能不工作" */
    if (en) DL_GPIO_clearPins(MUX_EN_PORT, MUX_EN_PIN);
    else     DL_GPIO_setPins(MUX_EN_PORT, MUX_EN_PIN);
}

static void mux_set_address(uint8_t code) {
    code &= 0x07;
    /* 低电平选通 */
    if (code & 0x01) DL_GPIO_setPins(MUX_AD0_PORT, MUX_AD0_PIN);
    else              DL_GPIO_clearPins(MUX_AD0_PORT, MUX_AD0_PIN);
    if (code & 0x02) DL_GPIO_setPins(MUX_AD1_PORT, MUX_AD1_PIN);
    else              DL_GPIO_clearPins(MUX_AD1_PORT, MUX_AD1_PIN);
    if (code & 0x04) DL_GPIO_setPins(MUX_AD2_PORT, MUX_AD2_PIN);
    else              DL_GPIO_clearPins(MUX_AD2_PORT, MUX_AD2_PIN);
}

/* ============================================================================
 * 引脚方向配置 (按当前模式)
 *
 * 关键: MSPM0G 的 GPIO 输出/输入必须先调 initDigitalOutput/initDigitalInput,
 *       这才会把 PINCM 切到 GPIO 模式; 只调 enableOutput 只置 DOE 位, 引脚
 *       仍停留在 ANALOG 模式, 信号到不了引脚 (这是 0.2 灰度 ADC 8 路同值 bug 的根因)
 * ============================================================================ */
static void reconfig_pins_for_current_mode(void) {
    /* 默认: 8 路全部配成 INPUT (高阻)
     *
     * 注意 PA27 (PINCM60) 是 huidu ADC 输入, syscfg 启动时已配 UNCONNECTED
     * (PF=0, ADC 模拟输入). 在 GPIO 模式下要切到 GPIOA_DIO27 (PF=1) 才能
     * 当数字输入读; 切回 MUX_ADC 时再还原成 UNCONNECTED (PF=0) 让 ADC 工作. */
    for (uint8_t i = 0; i < 8; i++) {
        if (i == 0) {
            /* PA27 默认就是 ADC, 这里先按 GPIO 配 (GPIO 模式需要) */
            DL_GPIO_initDigitalInput(kGrayGpio[i].iomux);
            DL_GPIO_initDigitalInputFeatures(kGrayGpio[i].iomux,
                DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
        } else {
            DL_GPIO_initDigitalInput(kGrayGpio[i].iomux);
            DL_GPIO_initDigitalInputFeatures(kGrayGpio[i].iomux,
                /* 灰度板 PNP 输出: 检测到黑线 → 输出高 (1), 白线 → 输出低 (0)
                 *   实测 (用户在 2026-07-13 0:14 确认) 黑线 = 1
                 *   PULL_UP 内部上拉同时作信号高 + 默认电平
                 *   如果观察反了 (白线 = 1), 改 INVERSION_ENABLE */
                DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
        }
        DL_GPIO_disableOutput(kGrayGpio[i].port, kGrayGpio[i].pin);
    }

    if (Huidu_Mode == HUIDU_MODE_MUX_ADC) {
        /* 4 个 MUX 控制引脚 (CH2=EN/PB25, CH3=AD2/PB19, CH4=AD1/PA24, CH5=AD0/PA26) */
        static const uint8_t kMuxOutIdx[4] = {1, 2, 3, 4};
        for (uint8_t k = 0; k < 4; k++) {
            uint8_t i = kMuxOutIdx[k];
            DL_GPIO_initDigitalOutput(kGrayGpio[i].iomux);
            DL_GPIO_initDigitalOutputFeatures(kGrayGpio[i].iomux,
                DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
            DL_GPIO_enableOutput(kGrayGpio[i].port, kGrayGpio[i].pin);
            DL_GPIO_clearPins(kGrayGpio[i].port, kGrayGpio[i].pin);  /* 默认地址 = 0 */
        }
        /* PA27 还原成 ADC 输入 (PF=UNCONNECTED=0) — 上面的循环可能误把它切到 GPIO */
        IOMUX->SECCFG.PINCM[IOMUX_PINCM60] =
            IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM60_PF_UNCONNECTED;
        mux_set_address(0);
        mux_enable(true);   /* 0.2 板 EN 低有效 */
    } else {
        /* GPIO 模式: EN 拉高 (MUX 关), 地址线清零, PA27 已经是 GPIO 输入 */
        mux_enable(false);
        mux_set_address(0);
    }
}

/* ============================================================================
 * 初始化 (含阈值/系数计算)
 * ============================================================================ */
void Huidu_Init(const uint16_t white[8], const uint16_t black[8]) {
    memset(&g_huidu_sensor, 0, sizeof(g_huidu_sensor));

    reconfig_pins_for_current_mode();

    /* 12 位 ADC */
    g_huidu_sensor.bits = 4096.0;

    for (uint8_t i = 0; i < 8; i++) {
        uint16_t w = white[i];
        uint16_t b = black[i];
        if (b >= w) {
            uint16_t t = w;
            w = b;
            b = t;
        }
        g_huidu_sensor.Calibrated_white[i] = w;
        g_huidu_sensor.Calibrated_black[i] = b;
        g_huidu_sensor.Gray_white[i] = (uint16_t)((w * 2U + b) / 3U);
        g_huidu_sensor.Gray_black[i] = (uint16_t)((w + b * 2U) / 3U);
        if ((w == 0 && b == 0) || (w == b)) {
            g_huidu_sensor.Normal_factor[i] = 0.0;
            continue;
        }
        double diff = (double)w - (double)b;
        g_huidu_sensor.Normal_factor[i] = g_huidu_sensor.bits / diff;
    }
    g_huidu_sensor.ok = 1;
}

/* ============================================================================
 * 模式切换 (运行时动态切, 重配引脚方向)
 * ============================================================================ */
void Huidu_SetMode(huidu_mode_t mode) {
    Huidu_Mode = mode;
    reconfig_pins_for_current_mode();
}

huidu_mode_t Huidu_GetMode(void) {
    return Huidu_Mode;
}

void Huidu_NextMode(void) {
    huidu_mode_t next = (huidu_mode_t)((Huidu_Mode + 1U) % 3U);
    Huidu_SetMode(next);
}

/* ============================================================================
 * 二值化 + 归一化
 * ============================================================================ */
/* 二值化阈值方向: Gray_white > Gray_black
 *   v > Gray_white  → 判定为白(0)
 *   v < Gray_black  → 判定为黑(1)
 *   中间滞回区保留上次状态
 *
 * 注意: 阈值方向与 GPIO 模式一致 (黑=bit1). 标定值由 Huidu_Init 决定. */
static void convert_to_digital(void) {
    uint8_t digital = g_huidu_sensor.Digital;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit = (uint8_t)(1U << i);
        uint16_t v = g_huidu_sensor.Analog_value[i];
        if (v > g_huidu_sensor.Gray_white[i]) {
            digital &= (uint8_t)~bit;
        } else if (v < g_huidu_sensor.Gray_black[i]) {
            digital |= bit;
        }
    }
    g_huidu_sensor.Digital = digital;
}

static void normalize(void) {
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t v = g_huidu_sensor.Analog_value[i];
        uint16_t b = g_huidu_sensor.Calibrated_black[i];
        uint16_t n;
        if (v < b || b == 0) {
            n = 0;
        } else {
            double tmp = (double)(v - b) * g_huidu_sensor.Normal_factor[i];
            n = (uint16_t)(tmp + 0.5);
            if (n > (uint16_t)g_huidu_sensor.bits) {
                n = (uint16_t)g_huidu_sensor.bits;
            }
        }
        g_huidu_sensor.Normal_value[i] = n;
    }
}

/* ============================================================================
 * 采集 (按当前模式)
 * ============================================================================ */
void Huidu_Task(void) {
    if (Huidu_Mode == HUIDU_MODE_MUX_ADC) {
        /* MUX_ADC: 顺序选 8 路, 每次读 ADC, 物理 i → 逻辑 [7-i] */
        for (uint8_t ch = 0; ch < 8; ch++) {
            mux_set_address(ch);
            delay_us_about(10);
            uint32_t acc = 0;
            for (uint8_t s = 0; s < 8; s++) {
                acc += adc_read_value();
            }
            uint16_t v = (uint16_t)(acc / 8U);
#if HUIDU_DIRECTION_REVERSE
            g_huidu_sensor.Analog_value[7 - ch] = v;
#else
            g_huidu_sensor.Analog_value[ch] = v;
#endif
        }
    } else if (Huidu_Mode == HUIDU_MODE_FIRST5_GPIO) {
        /* 5 路 GPIO: 物理 1, 2, 3, 4, 5 → 逻辑 0..4 (无反转, PCB 顺序即物理顺序) */
        for (uint8_t i = 0; i < 5; i++) {
            uint32_t pin = DL_GPIO_readPins(kGrayGpio[i].port, kGrayGpio[i].pin);
            g_huidu_sensor.Analog_value[i] = pin ? 4096U : 0U;
        }
        /* 物理 6, 7, 8 通道填 0 (未用) */
        g_huidu_sensor.Analog_value[5] = 0;
        g_huidu_sensor.Analog_value[6] = 0;
        g_huidu_sensor.Analog_value[7] = 0;
    } else {  /* HUIDU_MODE_ALL8_GPIO */
        for (uint8_t i = 0; i < 8; i++) {
            uint32_t pin = DL_GPIO_readPins(kGrayGpio[i].port, kGrayGpio[i].pin);
            g_huidu_sensor.Analog_value[i] = pin ? 4096U : 0U;
        }
    }

    convert_to_digital();
    normalize();
}

/* ============================================================================
 * 读 API
 * ============================================================================ */
uint8_t Huidu_GetDigital(void) {
    return g_huidu_sensor.Digital;
}

void Huidu_GetAnalog(uint16_t out[8]) {
    memcpy(out, g_huidu_sensor.Analog_value, sizeof(uint16_t) * 8);
}

void Huidu_GetNormal(uint16_t out[8]) {
    memcpy(out, g_huidu_sensor.Normal_value, sizeof(uint16_t) * 8);
}

void Huidu_SwitchAddress(uint8_t code) {
    mux_set_address(code);
}