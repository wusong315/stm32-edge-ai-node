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

/* ---- 接收环形缓冲（USART3 中断写入、主循环读出） -------------------
 * 🔴 为什么必须从"轮询 RXNE"改成中断：115200 下 87 µs 就到下一个字节，
 *    主循环里任何一次超过 87 µs 的阻塞（单次推理实测 ≈170 µs、日志搬运）
 *    都会让 DR 被新字节覆盖（ORE）⇒ 收到的字节直接消失。
 *    2026-10-25 上板实测：上报 8 次、puback 恒为 0，而华为云影子有数据
 *    ⇒ 上行通、下行(PUBACK 只 4 字节)在轮询间隙被丢光。
 *    改成 1 字节中断 + 256 B 环形缓冲后，主循环再忙也只是晚点取。 */
#define ESP_RX_RING   256u
static volatile uint8_t  s_ring[ESP_RX_RING];
static volatile uint16_t s_ring_head = 0u;   /* ISR 写 */
static volatile uint16_t s_ring_tail = 0u;   /* 主循环读 */
static volatile uint32_t s_last_rx_tick = 0u;/* 最近一次收到字节的时刻（ISR 维护） */

#if ESP_AT_DEBUG
#define DBG(...)  printf(__VA_ARGS__)
#else
#define DBG(...)  do { } while (0)
#endif

/* ------------------------------------------------------------------ */
/* 底层读写                                                            */
/* ------------------------------------------------------------------ */

/** 从环形缓冲取 1 字节。返回 1 = 有，0 = 空（**永不阻塞**） */
static int uart_getc(uint8_t *c)
{
    if (s_uart == NULL || c == NULL) {
        return 0;
    }
    if (s_ring_tail == s_ring_head) {
        return 0;
    }
    *c = s_ring[s_ring_tail];
    s_ring_tail = (uint16_t)((s_ring_tail + 1u) % ESP_RX_RING);
    return 1;
}

/** 环形缓冲里是否有字节 —— 几条指令，纳秒级 */
static int uart_rx_ready(void)
{
    return (s_ring_tail != s_ring_head) ? 1 : 0;
}

/**
 * @brief USART3 接收中断的**真正实现** —— 直接读寄存器，不经过 HAL
 *
 * 🔴 为什么不能用 `HAL_UART_Receive_IT()`：它会 `__HAL_LOCK(huart)`。
 *    而 `uart_put()` 走的是阻塞式 `HAL_UART_Transmit()`，整个发送期间都持有同一把锁。
 *    模块的应答正好在发送过程中开始到达（115200 下 87 µs/字节），中断里装机必然
 *    拿到 `HAL_BUSY`、**不重新置位 RXNEIE** ⇒ 从此再也不进中断 ⇒ 表现为
 *    "AT 无应答"。2026-10-25 实测踩过这个坑（v14 镜像一轮就复现）。
 *
 * 所以这里自己读 SR/DR：一次读操作同时清掉 RXNE 与 ORE，绕开 HAL 的状态机与锁。
 * 调用点：`stm32f4xx_it.c` 的 `USART3_IRQHandler()` 里 **HAL_UART_IRQHandler 之前**。
 */
void esp_uart3_isr(void)
{
    USART_TypeDef *u;
    uint32_t sr;
    uint8_t  b;
    uint16_t nx;

    if (s_uart == NULL) {
        return;
    }
    u  = s_uart->Instance;
    sr = u->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) == 0u) {
        return;
    }
    b = (uint8_t)u->DR;                     /* 读 DR 清 RXNE/ORE（溢出也一起清） */
    if ((sr & USART_SR_RXNE) == 0u) {
        return;                             /* 只有错误标志、没有新数据 */
    }
    nx = (uint16_t)((s_ring_head + 1u) % ESP_RX_RING);
    if (nx != s_ring_tail) {                /* 满了就丢这个字节（丢新，不覆盖未解析的旧数据） */
        s_ring[s_ring_head] = b;
        s_ring_head = nx;
        s_last_rx_tick = HAL_GetTick();     /* 记"最后一次收到"的时刻，供静默判定 */
    }
}

/** 重新打开接收中断（出错后调用；寄存器级，不碰 HAL 的锁） */
void esp_rx_restart(void)
{
    if (s_uart != NULL) {
        __HAL_UART_ENABLE_IT(s_uart, UART_IT_RXNE);
    }
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
    s_ring_head = 0u;
    s_ring_tail = 0u;
    /* 🔴 把"最后一次收到字节"锚在初始化时刻，而不是留 0。
     *    留 0 的话 `esp_silence_ms()` 会走哨兵分支返回"很久很久"，
     *    于是状态机一上来就以为线路早就静默好了、立刻发 `+++` ——
     *    而模块可能正忙着吐上一次的残留，3 个 `+` 会被当数据吞掉，
     *    白白浪费一整轮重连（2026-10-25 实测：多花 12 s）。 */
    s_last_rx_tick = HAL_GetTick();
    /* 只开 RXNEIE（寄存器级）：溢出(ORE)也会经由同一位触发中断，
     * 由 esp_uart3_isr() 读 DR 一并清掉。不动 HAL 的 RX 状态机，
     * 因此不会和阻塞发送抢锁。 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
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
        /* 🔴 先看标志位再读：`uart_getc()` 在没数据时要等满 1 ms，
         *    而被主循环每轮调用一次的话，1 kHz 采样直接崩（见 uart_rx_ready）。 */
        if (!uart_rx_ready()) {
            if ((HAL_GetTick() - t0) >= timeout_ms) {
                break;              /* timeout_ms == 0 时这里第一次就跳出 = 真非阻塞 */
            }
            continue;
        }
        if (uart_getc(&c)) {
            buf[n++] = c;
            t0 = HAL_GetTick();     /* 有数据就重置超时，尽量把一整个报文收完 */
        }
    }
    return (int)n;
}

/**
 * @brief 等到"**真正的**静默"：连续 T_SILENCE 毫秒没收到任何字节
 *
 * 🔴 为什么不能盲等：`+++` 只在"前后各 1 s 线路上没有数据"时才算暗号。
 *    光 `HAL_Delay(1100)` 只保证**我们不发**，不保证**模块不说** ——
 *    模块可能正在把 MCU 复位前排队的东西吐出来（残留 PUBACK、`CLOSED` 等），
 *    这时那 3 个 `+` 会被当成数据一起塞进 TCP，白费一次机会。
 *    2026-10-25 实测就有一次要逃逸两次才成，正是这个原因。
 *
 * @return 1 = 拿到静默；0 = 超时（超时也照样发 `+++`，多一次机会不亏）
 */
static int wait_silence(uint32_t max_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t last = t0;
    uint8_t  c;

    while ((HAL_GetTick() - t0) < max_ms) {
        if (uart_getc(&c)) {
            last = HAL_GetTick();       /* 又来字节了，静默计时重来 */
            continue;
        }
        if ((HAL_GetTick() - last) >= T_SILENCE) {
            return 1;
        }
    }
    return 0;
}

void esp_escape_transparent(void)
{
    static const uint8_t plus[3] = { '+', '+', '+' };

    if (s_uart == NULL) {
        return;
    }
    /* 🔴 先把本地标志清掉：这里的前提就是"MCU 侧并不知道模块的真实状态"
     *    （刚复位过、模块还停在 CIPMODE=1），所以不能靠 s_transparent 判断。 */
    s_transparent = 0;

    /* ① 等到线路真的安静（顺手把残留字节吃掉，免得被当成 AT 应答） */
    (void)wait_silence(3000u);

    /* ② 暗号本身 */
    uart_put(plus, 3u);

    /* ③ 之后也必须静默满 1 s，否则模块不认 */
    HAL_Delay(T_SILENCE);

    /* ④ 清掉模块可能回的 OK/ERROR/CLOSED，免得污染紧接着的 AT 判定 */
    line_reset();
    {
        uint8_t c;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 50u) {
            (void)uart_getc(&c);
        }
    }
}

esp_status_t esp_exit_transparent(void)
{
    if (!s_transparent) {
        return ESP_OK;
    }
    esp_escape_transparent();

    /* 用一条 AT 确认真的回到了指令模式（有些固件 +++ 后什么都不回） */
    return esp_ping();
}

/* ------------------------------------------------------------------ */
/* 非阻塞（异步）AT —— 运行期重连用                                    */
/* ------------------------------------------------------------------ */

#define ESP_CMD_MAX 160

static char        s_cmd[ESP_CMD_MAX];
static const char *s_async_tok = NULL;
static uint32_t    s_async_dl  = 0u;
static uint32_t    s_async_tmo = 0u;
static uint8_t     s_async_run = 0u;

uint32_t esp_silence_ms(void)
{
    if (s_last_rx_tick == 0u) {
        return 0x7FFFFFFFu;      /* 开机以来一个字节都没收到过 ⇒ 当作很久 */
    }
    return HAL_GetTick() - s_last_rx_tick;
}

void esp_rx_flush(void)
{
    /* 单写者（ISR 只动 head）⇒ 直接把 tail 追上去即可，不用关中断 */
    s_ring_tail = s_ring_head;
    line_reset();
}

void esp_set_transparent(int on)
{
    s_transparent = on ? 1 : 0;
}

void esp_send_plus3(void)
{
    static const uint8_t plus[3] = { '+', '+', '+' };
    if (s_uart == NULL) {
        return;
    }
    uart_put(plus, 3u);
}

/**
 * @brief 发出命令并开始计时。**立刻返回**，结果靠 `esp_async_poll()` 取。
 * @return 0 = 已发出；<0 = 状态错（没 init / 正在透传）
 */
static int async_start(const char *cmd, const char *token, uint32_t timeout_ms)
{
    if (s_uart == NULL || cmd == NULL) {
        return -1;
    }
    if (s_transparent) {
        /* 透传里发 AT 会直接进 TCP，必须挡住（与 at_cmd 一致） */
        DBG("    [ESP] ✗ 透传模式下拒绝发送 AT：%s\n", cmd);
        return -1;
    }
    esp_rx_flush();               /* 清掉上一条的残留应答 */
    s_async_tok = token;
    s_async_tmo = timeout_ms;
    s_async_dl  = HAL_GetTick() + timeout_ms;
    s_async_run = 1u;
    DBG("    [ESP] -> %s\n", cmd);
    uart_puts(cmd);
    uart_puts("\r\n");
    return 0;
}

int esp_async_poll(void)
{
    uint8_t  c;
    uint16_t guard = 0u;

    if (s_async_run == 0u) {
        return ESP_ASYNC_IDLE;
    }

    /* 🔴 匹配必须在"换行清缓冲"之前做 —— 与 `wait_for()` 同一个坑：
     *    `OK\r\n` 的 `\n` 一到 line_feed 就把缓冲清了，
     *    先清后匹配的话 `line_has("OK")` 永远为假，表现为"每条 AT 都超时"。 */
    while ((guard++ < (uint16_t)ESP_RX_RING) && uart_getc(&c)) {
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line_feed('\n');       /* 只负责打印 + 清空 */
            continue;
        }
        if (s_line_len < (uint16_t)(LINE_MAX - 1u)) {
            s_line[s_line_len++] = (char)c;
            s_line[s_line_len] = '\0';
        }
        if (line_has("ERROR") || line_has("FAIL") || line_has("busy p")) {
            s_async_run = 0u;
            DBG("    [ESP] <- 失败关键字：%s\n", s_line);
            return ESP_ASYNC_FAIL;
        }
        if (s_async_tok != NULL) {
            if (line_has(s_async_tok)) {
                s_async_run = 0u;
                return ESP_ASYNC_OK;
            }
        } else if (line_has("OK")) {
            s_async_run = 0u;
            return ESP_ASYNC_OK;
        }
    }

    if ((int32_t)(HAL_GetTick() - s_async_dl) >= 0) {
        s_async_run = 0u;
        DBG("    [ESP] <- 超时 %lu ms（等 \"%s\"）\n",
            (unsigned long)s_async_tmo, s_async_tok ? s_async_tok : "OK");
        return ESP_ASYNC_TIMEOUT;
    }
    return ESP_ASYNC_RUNNING;
}

int esp_async_ping(void)
{
    return async_start("AT", "OK", T_AT);
}

int esp_async_echo_off(void)
{
    return async_start("ATE0", "OK", T_AT);
}

int esp_async_join(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        return -1;
    }
    if ((uint32_t)strlen(ssid) + (uint32_t)strlen(password) + 20u > sizeof s_cmd) {
        return -1;
    }
    (void)snprintf(s_cmd, sizeof s_cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    /* 成功回 "WIFI GOT IP" + "OK"；失败回 "+CWJAP:<code>" + "FAIL" */
    return async_start(s_cmd, "OK", T_CWJAP);
}

int esp_async_tcp_open(const char *host, uint16_t port)
{
    if (host == NULL) {
        return -1;
    }
    if ((uint32_t)strlen(host) + 32u > sizeof s_cmd) {
        return -1;
    }
    (void)snprintf(s_cmd, sizeof s_cmd,
                   "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
    return async_start(s_cmd, "OK", T_CIPSTART);
}

int esp_async_cipmode(void)
{
    return async_start("AT+CIPMODE=1", "OK", T_AT);
}

int esp_async_cipsend(void)
{
    /* 无参数 ⇒ 进透传，模块回一行提示 + '>' */
    return async_start("AT+CIPSEND", ">", T_CIPSEND);
}
