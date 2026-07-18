/* ============================================================================
 * @file    uart_host.h
 * @brief   上位机 / K230 通用 UART 驱动 (状态机收包 + 阻塞发送 + printf)
 *
 *   应用层用法 (主循环):
 *     1. main() 上电调一次 UART_Host_Init();              ← enable NVIC + 清缓冲
 *     2. main.c 挂 UART0/1_INST_IRQHandler → UART0/1_RxCallback(rx_data);
 *     3. 主循环查 UARTx_RxFlag (一次性消费);
 *     4. 发送调 UART_SendChar / SendString / SendData / SendNumber / Printf.
 *
 * ============================================================================
 */
#ifndef __UART_HOST_H__
#define __UART_HOST_H__

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 接收包全局变量 (供主循环查询, 与 v1.21 命名一致)
 * ============================================================================
 * UART0_RxPacket: 上位机接收缓冲 (来自 UART_0, PA10/PA11)
 * UART1_RxPacket: K230   接收缓冲 (来自 K230,   PA8/PA9)
 * ============================================================================ */
#define UART_RX_BUF_SIZE   100
extern char             UART0_RxPacket[UART_RX_BUF_SIZE];
extern volatile uint8_t UART0_RxFlag;

extern char             UART1_RxPacket[UART_RX_BUF_SIZE];
extern volatile uint8_t UART1_RxFlag;

/* ============================================================================
 * UART 通道枚举
 * ============================================================================ */
typedef enum {
    UART_CH0 = 0,   /* 上位机 (UART_0, PA10/PA11) */
    UART_CH1 = 1,   /* K230    (K230,   PA8/PA9)   */
} uart_ch_t;

/* ============================================================================
 * API
 * ============================================================================ */

/* 初始化两个 UART (一次性 enable NVIC + 清状态). main() 启动时调一次. */
void UART_Host_Init(void);

/* 单字符 / 字符串 / 数据块发送 */
void UART_SendChar(uart_ch_t ch, uint8_t data);
void UART_SendString(uart_ch_t ch, const char *str);
void UART_SendData(uart_ch_t ch, const uint8_t *data, uint16_t length);
void UART_SendNumber(uart_ch_t ch, uint32_t Number, uint8_t Length);

/* 阻塞接收单字符 (调试用, 不要在中断或主循环里持续调用) */
uint8_t UART_ReceiveChar(uart_ch_t ch);

/* printf 重定向到指定通道 (snprintf 100 字节栈缓冲) */
void UART_Printf(uart_ch_t ch, const char *format, ...);

/* 中断回调 - 由 UART0/1_IRQHandler 调用 */
void UART0_RxCallback(uint8_t rx_data);   /* 上位机 */
void UART1_RxCallback(uint8_t rx_data);   /* K230 */

#endif /* __UART_HOST_H__ */