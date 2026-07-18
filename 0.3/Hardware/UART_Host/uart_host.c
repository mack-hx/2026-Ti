/* ============================================================================
 * @file    uart_host.c
 * @brief   上位机 (UART_0) + K230 (UART1) 通用 UART 驱动 (状态机收包 + 各种发送)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     UART_Host_Init();                  // enable NVIC + 清接收缓冲
 *
 *   主循环查接收:
 *     if (UART0_RxFlag) {                // 上位机收了一帧
 *         process_packet(UART0_RxPacket);
 *         UART0_RxFlag = 0;              // 一次性消费
 *     }
 *     if (UART1_RxFlag) {                // K230 收了一帧
 *         ...
 *         UART1_RxFlag = 0;
 *     }
 *
 *   主循环发送:
 *     UART_SendChar(UART_CH0, 'A');              // 单字节
 *     UART_SendString(UART_CH0, "hello\r\n");    // 字符串
 *     UART_SendData(UART_CH0, buf, len);         // 数据块
 *     UART_SendNumber(UART_CH0, 1234, 4);        // 4 位十进制数字
 *     UART_Printf(UART_CH1, "pos=%d\r\n", pos);  // printf 重定向
 *
 *   ISR 入口 (main.c 里挂, NVIC_EnableIRQ 已在 UART_Host_Init 里完成):
 *     void UART_0_INST_IRQHandler(void) {
 *         if (DL_UART_Main_getEnabledInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX)) {
 *             UART0_RxCallback(DL_UART_Main_receiveData(UART_0_INST));
 *             DL_UART_Main_clearInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
 *         }
 *     }
 *     void K230_INST_IRQHandler(void) {
 *         if (DL_UART_Main_getEnabledInterruptStatus(K230_INST, DL_UART_MAIN_INTERRUPT_RX)) {
 *             UART1_RxCallback(DL_UART_Main_receiveData(K230_INST));
 *             DL_UART_Main_clearInterruptStatus(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
 *         }
 *     }
 *
 * ============================================================================
 * 硬件映射 (来自 empty.syscfg)
 *   上位机 (UART_CH0): UART_0_INST (UART0, PA10/PA11, 9600 baud)
 *   K230   (UART_CH1): K230_INST   (UART1, PA8/PA9,   9600 baud)
 *
 *   SysConfig 已配置:
 *     - 波特率、字长、停止位、校验
 *     - RX 相关中断 (UART_0: RX+其它全开; K230: RX_TIMEOUT_ERROR)
 *   本文件额外做:
 *     - 显式 NVIC_EnableIRQ (SysConfig 已知只 SetPriority 不 Enable, 见 AGENTS.md §10)
 *     - 状态机收包 (与 v1.21 完全等价)
 *     - printf 重定向 (snprintf + 阻塞发送)
 *
 * ============================================================================
 * 数据包协议 (与 v1.21 一致, 状态机收包)
 *   - 不强制 '@' 开头, 收到任意字节就开始记录
 *   - 帧尾: \r\n 或 \n 单独; 兼容文本工具的 \n / \r 转义
 *   - 收到完整一帧后 UARTx_RxFlag=1, 主循环消费后清零
 *
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/UART_Host/uart_host.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================================================================
 * 接收缓冲 (与 v1.21 命名一致, 上位机 = UART0_RxPacket, K230 = UART1_RxPacket)
 * ============================================================================ */
char             UART0_RxPacket[UART_RX_BUF_SIZE] = {0};
volatile uint8_t UART0_RxFlag = 0;

char             UART1_RxPacket[UART_RX_BUF_SIZE] = {0};
volatile uint8_t UART1_RxFlag = 0;

/* ============================================================================
 * 内部: 阻塞发送单字节 (轮询 TX 完成)
 * ============================================================================ */
static inline void send_byte(uart_ch_t ch, uint8_t data) {
    if (ch == UART_CH0) {
        DL_UART_Main_transmitData(UART_0_INST, data);
        while (DL_UART_Main_isBusy(UART_0_INST)) {}
    } else {
        DL_UART_Main_transmitData(K230_INST, data);
        while (DL_UART_Main_isBusy(K230_INST)) {}
    }
}

/* 内部: 计算 X 的 Y 次方 (SendNumber 用) */
static uint32_t pow10_u32(uint32_t x, uint32_t y) {
    uint32_t r = 1;
    while (y--) { r *= x; }
    return r;
}

/* ============================================================================
 * API: 初始化
 * ============================================================================
 * SysConfig 已经初始化了 UART 外设, 这里只需要:
 *   1) 确保 NVIC 已 enable (SysConfig 已知漏 enable, 见 AGENTS.md §10)
 *   2) 清接收缓冲/标志
 * ============================================================================ */
void UART_Host_Init(void) {
    /* SysConfig 已经 SetPriority, 这里只显式 enable NVIC.
     * 注: PD42S1 的 x_bujin/y_bujin 也是同样的处理, 见 main.c. */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);

    memset((void *)UART0_RxPacket, 0, UART_RX_BUF_SIZE);
    UART0_RxFlag = 0;
    memset((void *)UART1_RxPacket, 0, UART_RX_BUF_SIZE);
    UART1_RxFlag = 0;
}

/* ============================================================================
 * API: 发送
 * ============================================================================ */
void UART_SendChar(uart_ch_t ch, uint8_t data) {
    send_byte(ch, data);
}

void UART_SendString(uart_ch_t ch, const char *str) {
    while (*str != '\0') {
        send_byte(ch, (uint8_t)(*str++));
    }
}

void UART_SendData(uart_ch_t ch, const uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        send_byte(ch, data[i]);
    }
}

void UART_SendNumber(uart_ch_t ch, uint32_t Number, uint8_t Length) {
    for (uint8_t i = 0; i < Length; i++) {
        uint32_t div = pow10_u32(10, Length - i - 1);
        send_byte(ch, (uint8_t)('0' + (Number / div) % 10));
    }
}

/* ============================================================================
 * API: 阻塞接收单字符 (调试用, 主循环应该查 RxFlag 而不是阻塞接收)
 * ============================================================================ */
uint8_t UART_ReceiveChar(uart_ch_t ch) {
    if (ch == UART_CH0) {
        while (!DL_UART_Main_getEnabledInterruptStatus(
                   UART_0_INST, DL_UART_MAIN_INTERRUPT_RX)) {}
        uint8_t d = DL_UART_Main_receiveData(UART_0_INST);
        DL_UART_Main_clearInterruptStatus(UART_0_INST,
                                          DL_UART_MAIN_INTERRUPT_RX);
        return d;
    } else {
        while (!DL_UART_Main_getEnabledInterruptStatus(
                   K230_INST, DL_UART_MAIN_INTERRUPT_RX)) {}
        uint8_t d = DL_UART_Main_receiveData(K230_INST);
        DL_UART_Main_clearInterruptStatus(K230_INST,
                                          DL_UART_MAIN_INTERRUPT_RX);
        return d;
    }
}

/* ============================================================================
 * API: printf 重定向 (栈上 snprintf 100 字节 → 阻塞发送)
 * ============================================================================ */
void UART_Printf(uart_ch_t ch, const char *format, ...) {
    char buf[UART_RX_BUF_SIZE];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    UART_SendString(ch, buf);
}

/* ============================================================================
 * 中断回调: 状态机收包 (与 v1.21 完全等价)
 *
 * 状态说明:
 *   0: 空闲 (等待开始记录)
 *   1: 正在接收数据
 *   2: 收到 \r, 等待 \n 确认 (CRLF)
 *
 * 转义兼容:
 *   - 文本工具可能直接发 "\n" 字节序列 (反斜杠 + n), 这里识别为帧尾
 *   - 文本工具可能直接发 "\r" 字节序列, 同上
 * ============================================================================
 */
static void rx_state_machine(uint8_t *state, uint8_t *idx,
                             char *buf, volatile uint8_t *flag,
                             uint8_t rx_data, uint8_t *bs) {
    const uint8_t max = (uint8_t)(UART_RX_BUF_SIZE - 1);

    if (*flag) {
        return;   /* 上次帧还没消费, 丢弃本次数据 */
    }

    if (*state == 0) {
        *state = 1;
        *idx   = 0;
        if (rx_data != '@' && *idx < max) {
            buf[(*idx)++] = (char)rx_data;
        }
        return;
    }

    /* state == 1: 数据 / 帧尾处理 */
    if (*bs) {
        *bs = 0;
        if (rx_data == 'n' || rx_data == 'r') {
            *state = 0;
            buf[*idx] = '\0';
            *flag = 1;
            return;
        }
        if (*idx < max) { buf[(*idx)++] = '\\'; }
    }

    if (rx_data == '\\') {
        *bs = 1;
    } else if (rx_data == '\n') {
        *state = 0;
        buf[*idx] = '\0';
        *flag = 1;
    } else if (rx_data == '\r') {
        *state = 2;
    } else {
        if (*idx < max) { buf[(*idx)++] = (char)rx_data; }
        if (*idx >= max) {
            *state = 0;
            buf[*idx] = '\0';
            *flag = 1;
        }
    }

    /* state == 2: \r 后等 \n */
    if (*state == 2) {
        if (rx_data == '\n') {
            *state = 0;
            buf[*idx] = '\0';
            *flag = 1;
        } else if (rx_data == '\r' || rx_data == '\n') {
            *state = 0;
            buf[*idx] = '\0';
            *flag = 1;
        } else {
            /* 之前 \r 当普通字符补回, 当前字节进入数据 */
            if (*idx < max) { buf[(*idx)++] = '\r'; }
            *state = 1;
            if (*idx < max) { buf[(*idx)++] = (char)rx_data; }
            if (*idx >= max) {
                *state = 0;
                buf[*idx] = '\0';
                *flag = 1;
            }
        }
    }
}

void UART0_RxCallback(uint8_t rx_data) {
    static uint8_t state = 0, idx = 0, bs = 0;
    rx_state_machine(&state, &idx, UART0_RxPacket, &UART0_RxFlag,
                     rx_data, &bs);
}

void UART1_RxCallback(uint8_t rx_data) {
    static uint8_t state = 0, idx = 0, bs = 0;
    rx_state_machine(&state, &idx, UART1_RxPacket, &UART1_RxFlag,
                     rx_data, &bs);
}