/* 自动生成 —— 由 tools/calib_dev.py 产出，请勿手改。
 * 生成时间：2026-10-15T17:18:18
 * 模型：a_true = (R_meas - b) / k    （R 单位 LSB，±2g → 16384 LSB/g）
 *
 * 🔴 固件当前**尚未**消费这些常数（Core/ 下无标定接口）；
 *    等接口就绪（OTA 参数区已预留 24 B）再接。此前可用于离线修正数据集。
 *
 * 使用示例：
 *     float ax = (raw_ax - MPU6050_CALIB_B_X) / MPU6050_CALIB_K_X;
 */
#ifndef MPU6050_CALIB_H
#define MPU6050_CALIB_H

#define MPU6050_CALIB_VALID        (1U)

#define MPU6050_CALIB_K_X         (1.001149f)
#define MPU6050_CALIB_K_Y         (1.004934f)
#define MPU6050_CALIB_K_Z         (1.025700f)

#define MPU6050_CALIB_B_X         (803.97f)   /* 0.049070 g */
#define MPU6050_CALIB_B_Y         (-607.63f)   /* -0.037087 g */
#define MPU6050_CALIB_B_Z         (-561.60f)   /* -0.034277 g */

#endif /* MPU6050_CALIB_H */
