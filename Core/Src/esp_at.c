/**
 * @file    esp_at.c
 * @brief   ESP8266 AT 驱动（TCP 管道 / 透传模式）
 *
 * 设计取舍（2026-10-22，写在没有硬件的时候）
 * --------------------------------------------------------------------------
 * ① **全部阻塞 + 轮询**，不用中断、不用 DMA：
 *    本模块只在 MQTT 的连接/上报时刻被调用（30 s 一次），不在 1 kHz 采样路径上；
 *    轮询最省心、最好读、出错时串口日志最直白。115200 下 1 字节 ≈ 87 µs，
 *    1 ms 超时的单字节非阻塞读足够。
 *
 * ② **每一行 AT 应答都 printf 出来**（默认开 `ESP_AT_DEBUG`）：
 *    10.23 联调时唯一的现场信息就是串口，先把原始应答打出来，
 *    比事后猜"到底是 ERROR 还是超时"便宜得多。发布时可把宏置 0。
 *
 * ③ **不用 `+IPD` 解析**：透传模式下数据是裸的，见 `esp_at.h` 的说明。
 *
 * ④ **超时值都写死成宏**，因为它们的依据是现场（DHCP 慢、模块忙）：
 *      AT 应答      1 000 ms
 *      CWJAP       25 000 ms   ← 一定要给足
 *      CIPSTART    10 000 ms
 *      CIPSEND 等 > 5 000 ms
 */
#include "esp_at.h"
#include <string.h>
#include <stdio.h>

#ifndef ESP_AT_DEBUG
#define ESP_AT_DEBUG 1
#endif

/* ---- 超时（ms） ---- */
#define T_AT          1000u
#define T_CWJAP      25000u
#define T_CIPSTART   10000u
#define T_CIPSEND     5000u
#define T_RST         3000u
#define T_SILENCE     1100u   /* 透传进出的静默要求是 1 s，留 10% 余量 */

/* ---- 行缓冲 ---- */
#define LINE_MAX      160

static UART_HandleTypeDef *s_uart = NULL;
static int      s_transparent = 0;
static char     s_line[LINE_MAX];
static uint16_t s_line_len = 0;

#if ESP_AT_DEBUG
#define DBG(...)  printf(__VA_ARGS__)
#else
#define DBG(...)  do { } while (0)
#endif

/* ------------------------------------------------------------------ */
/* 底层读写                                                            */
/* ------------------------------------------------------------------ */

/** 非阻塞读 1 字节。返回 1 = 读到，0 = 没有 */
static int uart_getc(uint8_t *c)
{
    if (s_uart == NULL) {
        return 0;
    }
    return (HAL_UART_Receive(s_uart, c, 1u, 1u) == HAL_OK) ? 1 : 0;
}

static void uart_put(const uint8_t *buf, uint16_t len)
{
    if (s_uart == NULL || buf == NULL || len == 0u) {
        return;
    }
    (void)HAL_UART_Transmit(s_uart, (uint8_t *)buf, len, 1000u);
}

static void uart_puts(const char *s)
{
    uart_put((const uint8_t *)s, (uint16_t)strlen(s));
}

/* ------------------------------------------------------------------ */
/* 行累积 + 关键字匹配                                                  */
/* ------------------------------------------------------------------ */

static void line_reset(void)
{
    s_line_len = 0u;
    s_line[0] = '\0';
}

/** 累积一个字符。遇到 \n 就打印整行并清空（只负责调试输出，不做匹配） */
static void line_feed(char c)
{
    if (c == '\r') {
        return;
    }
    if (c == '\n') {
        if (s_line_len > 0u) {
            DBG("    [ESP] %s\n", s_line);
        }
        line_reset();
        return;
    }
    if (s_line_len < (uint16_t)(LINE_MAX - 1u)) {
        s_line[s_line_len++] = c;
        s_line[s_line_len] = '\0';
    }
}

/** 当前累积的内容里是否包含 token（大小写敏感，AT 固件全大写） */
static int line_has(const char *token)
{
    return (strstr(s_line, token) != NULL) ? 1 : 0;
}

typedef enum { WAIT_FOUND = 0, WAIT_TIMEOUT = 1, WAIT_FAIL = 2 } wait_res_t;

/**
 * @brief 等到 token 出现，或超时，或出现 ERROR/FAIL
 * @param token 期望的字符串（如 "OK" / ">" / "ready"）；NULL = 只等 OK/ERROR
 *
 * 🔴 **匹配必须在"换行清缓冲"之前做**。第一版写成"先 line_feed 再匹配"，
 *    而 `OK\r\n` 的 `\n` 一到就把缓冲清空了 ⇒ `line_has("OK")` 永远为假，
 *    表现为**每条 AT 都超时**（而模块明明回了 OK）。这类 bug 在没有真机时
 *    靠读代码才发现，所以把顺序写死在这里。
 */
static wait_res_t wait_for(const char *token, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t  c;

    line_reset();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (!uart_getc(&c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            line_feed((char)c);         /* 只负责打印 + 清空 */
            continue;
        }
        /* 先入缓冲，再判断 */
        if (s_line_len < (uint16_t)(LINE_MAX - 1u)) {
            s_line[s_line_len++] = (char)c;
            s_line[s_line_len] = '\0';
        }
        /* ERROR / FAIL 是"确定失败"，不必等超时 */
        if (line_has("ERROR") || line_has("FAIL") || line_has("busy p")) {
            DBG("    [ESP] <- 失败关键字：%s\n", s_line);
            return WAIT_FAIL;
        }
        if (token != NULL) {
            if (line_has(token)) {
                return WAIT_FOUND;
            }
        } else if (line_has("OK")) {
            return WAIT_FOUND;
        }
    }
    DBG("    [ESP] <- 超时 %lu ms（等 \"%s\"）\n",
        (unsigned long)timeout_ms, token ? token : "OK");
    return WAIT_TIMEOUT;
}

/* ------------------------------------------------------------------ */
/* 发指令                                                              */
/* ------------------------------------------------------------------ */
static esp_status_t at_cmd(const char *cmd, const char *token, uint32_t timeout_ms)
{
    if (s_uart == NULL) {
        return ESP_ERR_PARAM;
    }
    if (s_transparent) {
        /* 透传里发 AT 会直接进 TCP，必须挡住 */
        DBG("    [ESP] ✗ 透传模式下拒绝发送 AT：%s\n", cmd);
        return ESP_ERR_STATE;
    }
    DBG("    [ESP] -> %s\n", cmd);
    uart_puts(cmd);
    uart_puts("\r\n");

    switch (wait_for(token, timeout_ms)) {
    case WAIT_FOUND:   return ESP_OK;
    case WAIT_FAIL:    return ESP_ERR_FAIL;
    default:           return ESP_ERR_TIMEOUT;
    }
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
void esp_init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_transparent = 0;
    line_reset();
}

int esp_is_transparent(void)
{
    return s_transparent;
}

esp_status_t esp_ping(void)
{
    return at_cmd("AT", NULL, T_AT);
}

esp_status_t esp_echo_off(void)
{
    return at_cmd("ATE0", NULL, T_AT);
}

esp_status_t esp_reset(void)
{
    esp_status_t st;

    if (s_transparent) {
        return ESP_ERR_STATE;       /* 透传里复位会打乱字节流，先 exit */
    }
    st = at_cmd("AT+RST", "ready", T_RST);
    if (st == ESP_OK) {
        return ESP_OK;
    }
    /* 有的固件只吐 "OK" 不吐 "ready"，用 AT 兜底确认；兜底也失败才算失败 */
    HAL_Delay(200u);
    return esp_ping();
}

esp_status_t esp_join(const char *ssid, const char *password)
{
    char cmd[128];
    esp_status_t st;

    if (ssid == NULL || password == NULL) {
        return ESP_ERR_PARAM;
    }
    if (s_transparent) {
        return ESP_ERR_STATE;
    }
    if ((uint32_t)strlen(ssid) + (uint32_t)strlen(password) + 16u > sizeof cmd) {
        return ESP_ERR_PARAM;
    }
    snprintf(cmd, sizeof cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    /* CWJAP 成功回 "WIFI GOT IP" + "OK"；失败回 "+CWJAP:<code>" + "FAIL" */
    st = at_cmd(cmd, "OK", T_CWJAP);
    if (st == ESP_OK) {
        DBG("    [ESP] Wi-Fi 已连接\n");
    }
    return st;
}

esp_status_t esp_tcp_open(const char *host, uint16_t port)
{
    char cmd[128];
    esp_status_t st;

    if (host == NULL) {
        return ESP_ERR_PARAM;
    }
    if (s_transparent) {
        return ESP_ERR_STATE;
    }
    if ((uint32_t)strlen(host) + 32u > sizeof cmd) {
        return ESP_ERR_PARAM;
    }
    snprintf(cmd, sizeof cmd, "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
    /* 成功回 "CONNECT" + "OK" */
    st = at_cmd(cmd, "OK", T_CIPSTART);
    if (st == ESP_OK) {
        DBG("    [ESP] TCP 已连上 %s:%u\n", host, (unsigned)port);
    }
    return st;
}

esp_status_t esp_enter_transparent(void)
{
    uint32_t t0;
    uint8_t  c;
    uint16_t drained = 0u;

    if (s_transparent) {
        return ESP_OK;              /* 已经在里面了，幂等 */
    }
    /* ① 开透传模式 */
    if (at_cmd("AT+CIPMODE=1", NULL, T_AT) != ESP_OK) {
        return ESP_ERR_FAIL;
    }
    /* ② CIPSEND 无参数 ⇒ 进透传，模块回一行提示 + '>' */
    if (at_cmd("AT+CIPSEND", ">", T_CIPSEND) != ESP_OK) {
        return ESP_ERR_FAIL;
    }
    s_transparent = 1;

    /* ③ 🔴 抽掉残留在 UART/FIFO 里的 "OK\r\n" 之类
     *    否则它们会被第一次 esp_transparent_recv() 当成 MQTT 字节收进来，
     *    表现为"CONNACK 前面多了几个字节"，解析失败但看起来像平台的问题。 */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 30u) {
        if (uart_getc(&c)) {
            drained++;
        }
    }
    DBG("    [ESP] 已进入透传模式（清掉残留 %u 字节，此后字节直达 TCP）\n",
        (unsigned)drained);
    return ESP_OK;
}

int esp_transparent_send(const uint8_t *buf, uint16_t len)
{
    if (!s_transparent || buf == NULL || len == 0u) {
        return ESP_ERR_STATE;
    }
    uart_put(buf, len);
    return (int)len;
}

int esp_transparent_recv(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms)
{
    uint32_t t0;
    uint16_t n = 0u;
    uint8_t  c;

    if (!s_transparent || buf == NULL || max_len == 0u) {
        return ESP_ERR_STATE;
    }
    t0 = HAL_GetTick();
    while (n < max_len) {
        if (uart_getc(&c)) {
            buf[n++] = c;
            t0 = HAL_GetTick();     /* 有数据就重置超时，尽量把一整个报文收完 */
            continue;
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            break;
        }
    }
    return (int)n;
}

esp_status_t esp_exit_transparent(void)
{
    uint8_t plus[3] = { '+', '+', '+' };

    if (!s_transparent) {
        return ESP_OK;
    }
    /* ①②③ 必须是"静默 1 s → +++ → 静默 1 s"，中间不能有任何其他字节 */
    HAL_Delay(T_SILENCE);
    uart_put(plus, 3u);
    HAL_Delay(T_SILENCE);
    s_transparent = 0;
    DBG("    [ESP] 已退出透传\n");

    /* ④ 用一条 AT 确认真的回到了指令模式（有些固件 +++ 后什么都不回） */
    line_reset();
    return esp_ping();
}
