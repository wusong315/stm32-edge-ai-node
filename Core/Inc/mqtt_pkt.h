/**
 * @file    mqtt_pkt.h
 * @brief   MQTT 3.1.1 报文编码器 + IoTDA 鉴权（纯 C、无 HAL 依赖、无 malloc）
 *
 * 定位
 * --------------------------------------------------------------------------
 * 这个模块**只负责"把字节拼出来"**，不碰 socket、不碰 AT 指令、不管状态机。
 * 分开的理由（2026-10-22）：
 *   · 编码是唯一能**完全离线证明正确**的部分 —— PC 上 gcc 编译后，
 *     与 `tools/iotda_probe.py` 的输出**逐字节比对**（`_tmp/hosttest/`）；
 *   · 10.23 上板后如果连不上，可以立刻把"报文拼错"排除掉，
 *     只剩 AT 驱动与接线两个变量 —— 这正是这个项目一直在用的"二分法"。
 *
 * 已对拍（2026-10-22）
 * --------------------------------------------------------------------------
 *   CONNECT  164 B，PUBLISH(qos0) 181 B，与 Python 端字节完全相同。
 *
 * 用法
 * --------------------------------------------------------------------------
 *   mqtt_pkt_t pkt;
 *   char cid[80], pw[65];
 *   mqtt_auth_build(DEVICE_ID, DEVICE_SECRET, AUTH_TS, AUTH_SIG, cid, sizeof cid, pw, sizeof pw);
 *   mqtt_encode_connect(&pkt, cid, DEVICE_ID, pw, 120);
 *   tcp_send(pkt.buf, pkt.len);
 */
#ifndef MQTT_PKT_H
#define MQTT_PKT_H

#include <stdint.h>
#include <stddef.h>

/** 报文缓冲上限。本项目 CONNECT 164 B / PUBLISH 181 B，留足余量。 */
#define MQTT_PKT_MAX  384

/** 编码结果。len == 0 表示失败（缓冲不够或参数非法），此时 buf 内容不可用。 */
typedef struct {
    uint8_t  buf[MQTT_PKT_MAX];
    uint16_t len;
} mqtt_pkt_t;

/**
 * @brief 按 IoTDA 规则生成 ClientId 与 Password
 * @param device_id  形如 `6ad76a1d..._stm32-01`（= 产品ID_NodeId）
 * @param secret     设备密钥（注册设备时自填的那串）
 * @param ts         `YYYYMMDDHH`，**必须 10 位**；`sig_type=0` 时不校验内容（实测）
 * @param sig_type   0 = 平台不校验时间戳 / 1 = 严格校验。本项目用 0。
 * @param cid_out    `{device_id}_0_{sig_type}_{ts}`
 * @param pw_out     `lower_hex(HMAC-SHA256(key=ts, msg=secret))`，65 字节
 * @return 0 成功；-1 参数非法或缓冲不够
 * @note  🔴 key 是 `ts`、msg 是 `secret`。反过来算是 0x04（已实测，别再猜）。
 */
int mqtt_auth_build(const char *device_id, const char *secret, const char *ts,
                    uint8_t sig_type,
                    char *cid_out, size_t cid_sz,
                    char *pw_out, size_t pw_sz);

/**
 * @brief CONNECT 报文
 * @param keepalive 秒。🔴 IoTDA 只接受 **30..1200**，填 10 会 `rc=0x02`（实测）。
 * @note 固定使用：协议 MQTT/4、clean session=1、带 username + password（flags = 0xC2）
 */
uint16_t mqtt_encode_connect(mqtt_pkt_t *out,
                             const char *client_id,
                             const char *username,
                             const char *password,
                             uint16_t keepalive);

/**
 * @brief PUBLISH 报文
 * @param qos     0 = 不等回执；**1 = 平台回 PUBACK（推荐，可确认"真收到了"）**
 * @param pkt_id  qos=1 时用；qos=0 时忽略
 */
uint16_t mqtt_encode_publish(mqtt_pkt_t *out,
                             const char *topic,
                             const uint8_t *payload, uint16_t payload_len,
                             uint8_t qos, uint16_t pkt_id);

/** PINGREQ（`C0 00`）—— 每隔约 60 s 发一次，否则 1.5×keepalive 被踢 */
uint16_t mqtt_encode_pingreq(mqtt_pkt_t *out);

/** DISCONNECT（`E0 00`） */
uint16_t mqtt_encode_disconnect(mqtt_pkt_t *out);

/**
 * @brief 把收到的报文类型提出来（只看第一个字节高 4 位）
 * @return 2=CONNACK / 4=PUBACK / 13=PINGRESP / 9=SUBACK / 0=不认识
 */
uint8_t mqtt_packet_type(const uint8_t *buf, uint16_t len);

/**
 * @brief 取 CONNACK 的返回码（第 4 字节）
 * @return 0=成功 / 2=clientId 错或 keepalive 非法 / 4=用户名密码错 / 5=未授权；-1=不是 CONNACK 或长度不足
 */
int mqtt_connack_rc(const uint8_t *buf, uint16_t len);

#endif /* MQTT_PKT_H */
