/**
 * @file    mqtt_pkt.c
 * @brief   MQTT 3.1.1 报文编码器 + IoTDA 鉴权
 *
 * 对拍基准（2026-10-22，与本文件的 CONNECT/PUBLISH 逐字节相同）
 * --------------------------------------------------------------------------
 * ClientId = 6ad76a1dcbb0cf6bb98f1aee_stm32-01_0_0_2026102205
 * Password = f8a2a682d66486d5406e5ababad3091592ad373e5d544fab89c0e6f7dd62acdd
 *
 * CONNECT 164 B:
 *   10 a1 01 0004 4d515454 04 c2 0078 0030<c31字符> 0021<33字符> 0040<64字符>
 *
 * PUBLISH(qos0) 181 B:
 *   30 b2 01 0043$oc/devices/<id>/sys/properties/report {"services":[...]}
 */
#include "mqtt_pkt.h"
#include "hmac_sha256.h"
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------- */
/* 内部工具                                                          */
/* ---------------------------------------------------------------- */

/** 写 2 字节大端（MQTT 里所有长度为大端） */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

/** 写 MQTT UTF-8 字符串：2 字节长度 + 内容。返回写入字节数，0 = 超长/失败 */
static uint16_t put_str(uint8_t *p, uint16_t cap, const char *s)
{
    size_t n = strlen(s);
    if (n > 0xFFFFu || (size_t)2 + n > cap) {
        return 0u;
    }
    put_u16(p, (uint16_t)n);
    memcpy(p + 2, s, n);
    return (uint16_t)(2u + n);
}

/**
 * @brief 写 MQTT "Remaining Length" 变长编码
 * @return 写入字节数（1..4）
 * @note  🔴 这是最容易算错的一段：编码的是**报文体长度**，不含首字节也不含本字段自身。
 *          我们的第一版 selftest 就栽在这（把 19 算成 10）。
 */
static uint8_t put_varint(uint8_t *p, uint32_t v)
{
    uint8_t n = 0;
    do {
        uint8_t b = (uint8_t)(v & 0x7Fu);
        v >>= 7;
        if (v > 0u) {
            b |= 0x80u;
        }
        p[n++] = b;
    } while (v > 0u && n < 4u);
    return n;
}

/* ---------------------------------------------------------------- */
/* 鉴权                                                              */
/* ---------------------------------------------------------------- */
int mqtt_auth_build(const char *device_id, const char *secret, const char *ts,
                    uint8_t sig_type,
                    char *cid_out, size_t cid_sz,
                    char *pw_out, size_t pw_sz)
{
    uint8_t digest[32];
    int n;

    if (device_id == NULL || secret == NULL || ts == NULL ||
        cid_out == NULL || pw_out == NULL) {
        return -1;
    }
    /* ts 必须 10 位（YYYYMMDDHH）。短串能过格式校验，但服务端解析未验证过 —— 不冒险 */
    if (strlen(ts) != 10u) {
        return -1;
    }

    /* ClientId = {device_id}_0_{sig_type}_{ts} */
    n = snprintf(cid_out, cid_sz, "%s_0_%u_%s", device_id, (unsigned)sig_type, ts);
    if (n < 0 || (size_t)n >= cid_sz) {
        return -1;
    }

    /* Password = lower_hex(HMAC-SHA256(key = ts, msg = secret)) */
    if (pw_sz < 65u) {
        return -1;
    }
    hmac_sha256((const uint8_t *)ts, strlen(ts),
                (const uint8_t *)secret, strlen(secret), digest);
    sha256_to_hex_lower(digest, pw_out);
    return 0;
}

/* ---------------------------------------------------------------- */
/* CONNECT                                                           */
/* ---------------------------------------------------------------- */
uint16_t mqtt_encode_connect(mqtt_pkt_t *out,
                             const char *client_id,
                             const char *username,
                             const char *password,
                             uint16_t keepalive)
{
    uint8_t  body[MQTT_PKT_MAX];
    uint16_t blen = 0u, n;
    uint8_t  hdr[5];
    uint8_t  hlen;

    if (out == NULL || client_id == NULL || username == NULL || password == NULL) {
        out->len = 0u;
        return 0u;
    }

    /* 可变头：协议名 "MQTT" + 协议级别 4 */
    n = put_str(body + blen, (uint16_t)(sizeof body - blen), "MQTT");
    if (n == 0u) { out->len = 0u; return 0u; }
    blen = (uint16_t)(blen + n);
    body[blen++] = 0x04u;

    /* 连接标志：username(7) | password(6) | clean session(1) = 0xC2 */
    body[blen++] = 0xC2u;

    /* keepalive（秒，2 字节大端） */
    put_u16(body + blen, keepalive);
    blen = (uint16_t)(blen + 2u);

    /* 载荷：ClientId / Username / Password（顺序固定，不可换） */
    n = put_str(body + blen, (uint16_t)(sizeof body - blen), client_id);
    if (n == 0u) { out->len = 0u; return 0u; }
    blen = (uint16_t)(blen + n);

    n = put_str(body + blen, (uint16_t)(sizeof body - blen), username);
    if (n == 0u) { out->len = 0u; return 0u; }
    blen = (uint16_t)(blen + n);

    n = put_str(body + blen, (uint16_t)(sizeof body - blen), password);
    if (n == 0u) { out->len = 0u; return 0u; }
    blen = (uint16_t)(blen + n);

    /* 固定头：0x10 + Remaining Length */
    hdr[0] = 0x10u;
    hlen = put_varint(hdr + 1, (uint32_t)blen);

    if ((uint16_t)(1u + hlen + blen) > MQTT_PKT_MAX) {
        out->len = 0u;
        return 0u;
    }
    out->buf[0] = hdr[0];
    memcpy(out->buf + 1, hdr + 1, hlen);
    memcpy(out->buf + 1 + hlen, body, blen);
    out->len = (uint16_t)(1u + hlen + blen);
    return out->len;
}

/* ---------------------------------------------------------------- */
/* PUBLISH                                                           */
/* ---------------------------------------------------------------- */
uint16_t mqtt_encode_publish(mqtt_pkt_t *out,
                             const char *topic,
                             const uint8_t *payload, uint16_t payload_len,
                             uint8_t qos, uint16_t pkt_id)
{
    uint8_t  body[MQTT_PKT_MAX];
    uint16_t blen = 0u, n;
    uint8_t  hdr[5];
    uint8_t  hlen;
    uint8_t  first;

    if (out == NULL || topic == NULL || (payload == NULL && payload_len > 0u) ||
        qos > 1u) {
        out->len = 0u;
        return 0u;
    }

    n = put_str(body + blen, (uint16_t)(sizeof body - blen), topic);
    if (n == 0u) { out->len = 0u; return 0u; }
    blen = (uint16_t)(blen + n);

    if (qos == 1u) {
        put_u16(body + blen, pkt_id);
        blen = (uint16_t)(blen + 2u);
    }

    if ((uint32_t)blen + payload_len > (uint32_t)(sizeof body)) {
        out->len = 0u;
        return 0u;
    }
    if (payload_len > 0u) {
        memcpy(body + blen, payload, payload_len);
        blen = (uint16_t)(blen + payload_len);
    }

    /* 固定头首字节：PUBLISH(3) << 4 | qos << 1 | retain(0) */
    first = (uint8_t)(0x30u | (uint8_t)(qos << 1));
    hdr[0] = first;
    hlen = put_varint(hdr + 1, (uint32_t)blen);

    if ((uint16_t)(1u + hlen + blen) > MQTT_PKT_MAX) {
        out->len = 0u;
        return 0u;
    }
    out->buf[0] = hdr[0];
    memcpy(out->buf + 1, hdr + 1, hlen);
    memcpy(out->buf + 1 + hlen, body, blen);
    out->len = (uint16_t)(1u + hlen + blen);
    return out->len;
}

/* ---------------------------------------------------------------- */
/* 简单报文                                                          */
/* ---------------------------------------------------------------- */
uint16_t mqtt_encode_pingreq(mqtt_pkt_t *out)
{
    if (out == NULL) {
        return 0u;
    }
    out->buf[0] = 0xC0u;
    out->buf[1] = 0x00u;
    out->len = 2u;
    return 2u;
}

uint16_t mqtt_encode_disconnect(mqtt_pkt_t *out)
{
    if (out == NULL) {
        return 0u;
    }
    out->buf[0] = 0xE0u;
    out->buf[1] = 0x00u;
    out->len = 2u;
    return 2u;
}

/* ---------------------------------------------------------------- */
/* 收包解析                                                          */
/* ---------------------------------------------------------------- */
uint8_t mqtt_packet_type(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < 2u) {
        return 0u;
    }
    return (uint8_t)(buf[0] >> 4);
}

int mqtt_connack_rc(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < 4u || (buf[0] >> 4) != 2u) {
        return -1;
    }
    return (int)buf[3];
}
