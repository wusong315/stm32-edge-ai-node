/**
 * @file    hmac_sha256.c
 * @brief   SHA-256 + HMAC-SHA256 实现（FIPS 180-4 / RFC 2104）
 *
 * 验证（2026-10-22）：`_tmp/hosttest/` 用 mingw gcc 编译后跑固定向量，
 *   · SHA-256("")                      = e3b0c442…b855
 *   · SHA-256("abc")                   = ba7816bf…15ad
 *   · HMAC RFC 4231 case 1 / case 2 / case 6 / case 7 全部匹配
 *   · HMAC(key="2026102205", msg=本项目的 secret) 与 Python hmac 逐字符相同
 */
#include "hmac_sha256.h"

/* ------------------------------------------------------------------ */
/* SHA-256 常量                                                        */
/* ------------------------------------------------------------------ */
static const uint32_t K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
    0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
    0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
    0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32u - (n))))

/** 处理一个 64 字节块，就地更新状态 h[8]。 */
static void sha256_block(uint32_t h[8], const uint8_t blk[64])
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, hh, t1, t2;
    int i;

    /* 大端读入 16 个字 */
    for (i = 0; i < 16; i++) {
        w[i] = ((uint32_t)blk[i * 4 + 0] << 24)
             | ((uint32_t)blk[i * 4 + 1] << 16)
             | ((uint32_t)blk[i * 4 + 2] << 8)
             | ((uint32_t)blk[i * 4 + 3]);
    }
    /* 扩展到 64 个字 */
    for (i = 16; i < 64; i++) {
        uint32_t s0 = ROTR(w[i - 15], 7) ^ ROTR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROTR(w[i - 2], 17) ^ ROTR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    a = h[0]; b = h[1]; c = h[2]; d = h[3];
    e = h[4]; f = h[5]; g = h[6]; hh = h[7];

    for (i = 0; i < 64; i++) {
        uint32_t S1 = ROTR(e, 6) ^ ROTR(e, 11) ^ ROTR(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t S0 = ROTR(a, 2) ^ ROTR(a, 13) ^ ROTR(a, 22);
        uint32_t mj = (a & b) ^ (a & c) ^ (b & c);

        t1 = hh + S1 + ch + K[i] + w[i];
        t2 = S0 + mj;

        hh = g; g = f; f = e;
        e  = d + t1;
        d  = c; c = b; b = a;
        a  = t1 + t2;
    }

    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

void sha256(const uint8_t *data, size_t len, uint8_t out[32])
{
    uint32_t h[8] = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u
    };
    uint8_t  blk[64];
    uint64_t bits = (uint64_t)len * 8u;
    size_t   i = 0, rem;
    int      k;

    /* 整块 */
    while (len - i >= 64u) {
        sha256_block(h, data + i);
        i += 64u;
    }

    /* 尾巴：数据 + 0x80 + 补零 + 64 位长度（大端） */
    rem = len - i;
    for (k = 0; k < 64; k++) {
        blk[k] = 0u;
    }
    for (k = 0; k < (int)rem; k++) {
        blk[k] = data[i + (size_t)k];
    }
    blk[rem] = 0x80u;

    if (rem >= 56u) {
        sha256_block(h, blk);               /* 本块放不下长度，再压一块 */
        for (k = 0; k < 64; k++) {
            blk[k] = 0u;
        }
    }
    for (k = 0; k < 8; k++) {
        blk[63 - k] = (uint8_t)((bits >> (8u * (uint32_t)k)) & 0xFFu);
    }
    sha256_block(h, blk);

    /* 大端输出 */
    for (k = 0; k < 8; k++) {
        out[k * 4 + 0] = (uint8_t)((h[k] >> 24) & 0xFFu);
        out[k * 4 + 1] = (uint8_t)((h[k] >> 16) & 0xFFu);
        out[k * 4 + 2] = (uint8_t)((h[k] >> 8) & 0xFFu);
        out[k * 4 + 3] = (uint8_t)((h[k]) & 0xFFu);
    }
}

void hmac_sha256(const uint8_t *key, size_t key_len,
                 const uint8_t *msg, size_t msg_len,
                 uint8_t out[32])
{
    uint8_t k0[64];
    uint8_t ipad[64];
    uint8_t opad[64];
    uint8_t inner[32];
    uint8_t scratch[64 + 32];   /* 不动态分配；inner 最长 = 32 字节，放得下 */
    int     i;

    for (i = 0; i < 64; i++) {
        k0[i] = 0u;
    }
    if (key_len > 64u) {
        sha256(key, key_len, k0);   /* 长密钥先摘要，剩下 32 字节仍为 0 */
    } else if (key_len > 0u) {
        for (i = 0; i < (int)key_len; i++) {
            k0[i] = key[i];
        }
    }

    for (i = 0; i < 64; i++) {
        ipad[i] = (uint8_t)(k0[i] ^ 0x36u);
        opad[i] = (uint8_t)(k0[i] ^ 0x5Cu);
    }

    /* inner = SHA256(ipad || msg) —— 分两段喂，避免大缓冲 */
    {
        /* 先把 ipad 放进 scratch，再追加 msg（msg 可能很长，分段处理） */
        size_t total = 64u + msg_len;
        uint32_t h[8] = {
            0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
            0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u
        };
        uint8_t  blk[64];
        size_t   consumed = 0u, n;
        uint64_t bits = (uint64_t)total * 8u;
        int      k;

        /* 逐 64 字节压块：前 64 字节来自 ipad，其余来自 msg */
        while (total - consumed >= 64u) {
            for (k = 0; k < 64; k++) {
                size_t idx = consumed + (size_t)k;
                blk[k] = (idx < 64u) ? ipad[idx] : msg[idx - 64u];
            }
            sha256_block(h, blk);
            consumed += 64u;
        }

        n = total - consumed;
        for (k = 0; k < 64; k++) {
            blk[k] = 0u;
        }
        for (k = 0; k < (int)n; k++) {
            size_t idx = consumed + (size_t)k;
            blk[k] = (idx < 64u) ? ipad[idx] : msg[idx - 64u];
        }
        blk[n] = 0x80u;
        if (n >= 56u) {
            sha256_block(h, blk);
            for (k = 0; k < 64; k++) {
                blk[k] = 0u;
            }
        }
        for (k = 0; k < 8; k++) {
            blk[63 - k] = (uint8_t)((bits >> (8u * (uint32_t)k)) & 0xFFu);
        }
        sha256_block(h, blk);

        for (k = 0; k < 8; k++) {
            inner[k * 4 + 0] = (uint8_t)((h[k] >> 24) & 0xFFu);
            inner[k * 4 + 1] = (uint8_t)((h[k] >> 16) & 0xFFu);
            inner[k * 4 + 2] = (uint8_t)((h[k] >> 8) & 0xFFu);
            inner[k * 4 + 3] = (uint8_t)((h[k]) & 0xFFu);
        }
    }

    /* outer = SHA256(opad || inner) */
    for (i = 0; i < 64; i++) {
        scratch[i] = opad[i];
    }
    for (i = 0; i < 32; i++) {
        scratch[64 + i] = inner[i];
    }
    sha256(scratch, 96u, out);
}

void sha256_to_hex_lower(const uint8_t digest[32], char out[65])
{
    static const char HEX[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 32; i++) {
        out[i * 2 + 0] = HEX[(digest[i] >> 4) & 0x0Fu];
        out[i * 2 + 1] = HEX[digest[i] & 0x0Fu];
    }
    out[64] = '\0';
}
