/**
 * @file    esp_at.h
 * @brief   ESP8266（D1 mini）AT 驱动 —— **只做 TCP 管道**（方案 A，决策 D34）
 *
 * 定位（2026-10-22）
 * --------------------------------------------------------------------------
 * 这里**不实现 MQTT**，也不解析 `+IPD`。ESP 只负责三件事：
 *   ① 连 Wi-Fi  ② 建一条到 IoTDA 的 TCP 连接  ③ 进「透传」后当一根网线。
 * MQTT 报文由 `mqtt_pkt.c` 在 MCU 侧拼好，直接往这根管道里写 ——
 * 好处是**发出去的字节就是 `tools/iotda_probe.py` 打印的那串字节**，可以逐字节对拍。
 *
 * 🔴 为什么用透传（`CIPMODE=1`）而不是 `CIPSEND=<len>`
 * --------------------------------------------------------------------------
 * 普通模式下每发一包都要 `AT+CIPSEND=<len>` + 等 `>`，收包还要解析
 * `+IPD,<len>:<data>`（长度是文本、数据可能被截断、还夹着 AT 应答）。
 * 透传模式下**收发都是裸字节**，上面这些全不存在 —— 对 MQTT 这种"定时小包"最合适。
 *
 * 代价：透传里**任何**从 MCU 出去的字节都会进 TCP。所以
 *   · 进透传后**不要**再发 AT 指令；
 *   · 退出必须用 `+++`，且前后各留 1 s 静默（见 `esp_exit_transparent()`）。
 *
 * 硬件前提（接线见 `RUNBOOK R9.7.2`）
 * --------------------------------------------------------------------------
 *   PB10 = USART3_TX → ESP 的 **RX**
 *   PB11 = USART3_RX ← ESP 的 **TX**
 *   共地；ESP 供电 3V3（🔴 USB 与 3V3 只能接一个）
 *   ⚠️ D1 mini 板载 CH340 的 TXD 焊在同一条 RX 线上 ⇒ **联调时必须拔掉 USB**
 *
 * 用法
 * --------------------------------------------------------------------------
 *   esp_init(&huart3);
 *   if (esp_ping() == ESP_OK && esp_join(WIFI_SSID, WIFI_PASS) == ESP_OK &&
 *       esp_tcp_open(IOTDA_HOST, 1883) == ESP_OK &&
 *       esp_enter_transparent() == ESP_OK) {
 *       esp_transparent_send(pkt.buf, pkt.len);      // 这就是 CONNECT
 *       n = esp_transparent_recv(rx, sizeof rx);     // 这就是 CONNACK
 *   }
 */
#ifndef ESP_AT_H
#define ESP_AT_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 返回码。非 0 一律带具体原因，方便串口上直接看出卡在哪一步 */
typedef enum {
    ESP_OK          = 0,
    ESP_ERR_PARAM   = -1,
    ESP_ERR_TIMEOUT = -2,   /* 等不到 OK */
    ESP_ERR_FAIL    = -3,   /* 收到 ERROR / FAIL / busy */
    ESP_ERR_STATE   = -4    /* 调用顺序不对（如没进透传就 send） */
} esp_status_t;

/** 绑定 UART 句柄（本项目用 `&huart3`），并清空内部状态 */
void esp_init(UART_HandleTypeDef *huart);

/** `AT` → `OK`。用来确认模块活着、波特率对（出厂多为 115200） */
esp_status_t esp_ping(void);

/** `ATE0` → `OK`。关回显，否则每条指令都会被原样吐回来，增加解析噪声 */
esp_status_t esp_echo_off(void);

/** 软复位。改 Wi-Fi 参数或模块行为异常时用，之后要重新 ping/join（约 1~2 s） */
esp_status_t esp_reset(void);

/**
 * @brief `AT+CWJAP="ssid","pwd"` —— 加入 AP
 * @note  超时给足 **25 s**（路由器 + DHCP 慢的时候真的会到 10 s 以上）。
 *        🔴 透传模式下**不是** AT 指令，调用它会打乱字节流 ⇒ 会被 `ESP_ERR_STATE` 挡掉。
 */
esp_status_t esp_join(const char *ssid, const char *password);

/** `AT+CIPSTART="TCP",<host>,<port>` —— 建到 IoTDA 的 TCP（端口 **1883**） */
esp_status_t esp_tcp_open(const char *host, uint16_t port);

/**
 * @brief 进透传：`AT+CIPMODE=1` → `AT+CIPSEND` → 等 `>`
 * @note  成功返回后，**任何** `esp_transparent_send()` 的字节都直达 TCP。
 */
esp_status_t esp_enter_transparent(void);

/**
 * @brief 透传模式下发字节（原样写 UART，不追加任何 CRLF）
 * @return 实际写出的字节数；<0 表示状态错或长度非法
 */
int esp_transparent_send(const uint8_t *buf, uint16_t len);

/**
 * @brief 透传模式下收字节（非阻塞，最多等 `timeout_ms`）
 * @return >0 收到的字节数；0 = 这段时间没有数据（**不是错误**）；<0 状态错
 */
int esp_transparent_recv(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms);

/**
 * @brief 退出透传：静默 1 s → `+++`（不带 CRLF）→ 静默 1 s → `AT` 验活
 * @note  `+++` 前后必须静默，否则模块不会退出。这是透传唯一的"暗号"。
 */
esp_status_t esp_exit_transparent(void);

/** 是否处于透传模式（调试用） */
int esp_is_transparent(void);

/**
 * @brief **开机逃逸**：不管 MCU 自己以为在什么模式，硬发一次 `+++`
 *
 * 🔴 为什么必须有：STM32 与 ESP8266 是**两颗独立的芯片**，只有 STM32 会被
 *    ST-Link / 看门狗 / 手动 RESET 复位，模块的电源一直是通的。上一次运行
 *    如果进过透传（`CIPMODE=1`），模块就**留在透传里**；新起的 App 一发
 *    `AT\r\n`，这 4 个字节会直接进 TCP 流，模块不做任何解释
 *    ⇒ 表现为「AT 超时 / ESP 无应答」，而换回任何旧固件都一样失败
 *    （2026-10-25 实测踩到：v13 这个原本跑通的镜像也报同一句）。
 *
 * 所以启动握手失败时先发暗号把模块拽回指令模式，再重试 `AT`。
 * 反复多发一次是无害的：已经在指令模式时 `+++` 只是个看不懂的命令。
 *
 * @note 阻塞约 2.2 s（前后各 1.1 s 静默，模块只认这个时序）。
 */
void esp_escape_transparent(void);

/**
 * @brief USART3 接收中断实现 —— 由 `USART3_IRQHandler()` 在 HAL 之前调用
 *
 * 2026-10-25 起接收改成「1 字节中断 + 256 B 环形缓冲」：
 * 轮询 RXNE 在 115200 下会丢字节（推理 170 µs 期间就有 2 字节进来），
 * 4 字节的 PUBACK 丢一个就整包认不出。
 * 🔴 内部直接读 SR/DR，**不用 HAL_UART_Receive_IT**（那个会和阻塞发送抢 HAL 锁，
 *    拿不到锁就不再置位 RXNEIE，表现为"AT 无应答"）。
 */
void esp_uart3_isr(void);

/** 出错（ORE/FE/NE）后重新装接收，否则再也不进中断 */
void esp_rx_restart(void);

#endif /* ESP_AT_H */
