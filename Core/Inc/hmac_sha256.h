/**
 * @file    hmac_sha256.h
 * @brief   SHA-256 + HMAC-SHA256（纯 C、无 HAL 依赖、无 malloc）
 *
 * 为什么自己写而不引库（2026-10-22，T10 上云）
 * --------------------------------------------------------------------------
 * 需求只有一件事：算 IoTDA 的 MQTT 密码 ——
 *      password = lower_hex( HMAC-SHA256(key = ts, msg = device_secret) )
 * 引 mbedTLS 会带进几十 KB 和一套配置系统，而我们**已经决定不用 TLS**（明文 1883）。
 * 所以写一个 ~200 行的独立实现，好处是：
 *   ① 可以在 PC 上用 gcc 编译、和 Python 的 hmac 逐字节对拍（`_tmp/hosttest/`）；
 *   ② 上板后行为与 PC 上验证过的完全一致（同一份源码）。
 *
 * 接口约定
 * --------------------------------------------------------------------------
 * - 只依赖 <stdint.h> / <stddef.h> / <string.h>
 * - 不打印、不分配内存、可重入（无全局状态）
 * - `out` 必须是 32 字节
 */
#ifndef HMAC_SHA256_H
#define HMAC_SHA256_H

#include <stdint.h>
#include <stddef.h>

/** SHA-256 摘要（FIPS 180-4）。out[32]。 */
void sha256(const uint8_t *data, size_t len, uint8_t out[32]);

/** HMAC-SHA256（RFC 2104）。out[32]。key 任意长度（>64 会先做一次 SHA256）。 */
void hmac_sha256(const uint8_t *key, size_t key_len,
                 const uint8_t *msg, size_t msg_len,
                 uint8_t out[32]);

/**
 * @brief 把 32 字节摘要写成 64 个小写十六进制字符 + '\0'（`out` 至少 65 字节）
 * @note  IoTDA 的 Password 要求**小写** hex，大写会被拒。
 */
void sha256_to_hex_lower(const uint8_t digest[32], char out[65]);

#endif /* HMAC_SHA256_H */
