# STM32 边缘 AI 振动故障诊断节点

> **中文** · 基于 STM32F407 的边缘 AI 振动故障诊断节点 —— 1 kHz 采样 → 片上 AI 推理 → A/B 双区 OTA → MQTT 上云
>
> **English** · An edge-AI vibration fault-detection node on STM32F407 — 1 kHz IMU sampling, on-device NanoEdge AI inference, A/B dual-slot OTA and MQTT cloud reporting.

⚠️ **项目进行中（第 1 周）**：当前仓库内是**工程骨架与外设配置**，应用代码尚未开始。进度见下表。

---

## 这是什么

把「电机振动是否异常」的判断**放在设备本地完成**，而不是把原始数据传到云端再算。

工业现场的泵、风机、传送带需要早期故障预警，但往往**没有稳定网络**，也不适合把 1 kHz 的振动原始数据持续上传。本项目的做法是：

1. **本地采集** —— MPU6050（I²C）以 **1 kHz** 采三轴加速度
2. **本地推理** —— NanoEdge AI 生成的模型**直接跑在 MCU 上**（Cortex-M4F @ 168 MHz），输出「正常 / 异常」
3. **本地决策** —— 异常时点亮 LED 并上报；**断网也能独立工作**
4. **按需上云** —— 联网时只上传**结论与少量特征**，不传原始数据
5. **远程升级** —— A/B 双区 OTA，坏包拒绝、断电不变砖

## 当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 环境 | STM32CubeMX / STM32CubeIDE / 工具链 / 时钟树（168 MHz） | ✅ 完成 |
| 骨架 | 工程结构、Flash 链接脚本、`.gitignore` | ✅ 完成 |
| 外设 | I2C1 400 kHz、USART1 115200、TIM2 1 kHz、GPIO | ✅ 完成 |
| 驱动 | MPU6050 寄存器驱动（`WHO_AM_I` 校验） | ⬜ 待硬件 |
| 采集 | TIM2 中断 1 kHz 采样 + 串口输出 + PC 端波形验证 | ⬜ 待硬件 |
| 数据集 | 正常 / 异常各 50 段 CSV | ⬜ 待硬件 |
| 模型 | NanoEdge AI 训练与片上集成 | ⬜ 计划中 |
| OTA | A/B 双区升级与回滚验证 | ⬜ 计划中 |
| 上云 | ESP8266 + MQTT + HMAC 签名上报 | ⬜ 计划中 |

## 硬件

| 部件 | 型号 | 用途 |
|---|---|---|
| 主控 | STM32F407VGT6（LQFP100，1 MB Flash） | 采样 + 推理 + 控制 |
| 传感器 | MPU6050（I²C） | 三轴加速度 / 陀螺仪 |
| 通信 | ESP8266（AT 透传） | MQTT 上云 |
| 振动源 | 130 直流电机 + IRF520 MOSFET | 模拟正常 / 异常工况 |
| 供电 | 独立电池组 | 电机与主控隔离供电，避免干扰 |

## 系统架构

```
              ┌─────────────┐
   MPU6050 ──▶│  I2C1       │
  (1 kHz)     │  400 kHz    │        报告结论 / 特征
              └──────┬──────┘
                     │
              ┌──────▼──────┐    ┌──────────────────┐    ┌──────────────┐
              │ STM32F407   │    │ NanoEdge AI 模型 │    │ 硬件判决     │
              │ TIM2 中断   │───▶│ (片上推理)       │───▶│            │
              │ 1 kHz 采样  │    │ Cortex-M4F       │    └──────┬───────┘
              └──────┬──────┘    └──────────────────┘           │
                     │                                          │
              ┌──────▼──────┐                          ┌────────▼────────┐
              │ USART1      │                          │ LED / 本地告警  │
              │ 115200 调试 │                          │  (PA1)          │
              └──────┬──────┘                          └─────────────────┘
                     │
              ┌──────▼──────┐        ┌──────────────┐
              │ ESP8266     │───────▶│ 云端 IoT     │
              │ AT + MQTT   │        │ (结论上云)   │
              └─────────────┘        └──────────────┘
```

## 时钟与关键配置

| 项 | 值 |
|---|---|
| HSE | 8 MHz |
| PLL | M = 8 / N = 336 / P = 2 |
| SYSCLK = HCLK | **168 MHz** |
| APB1 / APB2 | 42 MHz / 84 MHz |
| FLASH_LATENCY | 5 WS |

| 外设 | 引脚 | 参数 |
|---|---|---|
| I2C1 | PB6 = SCL / PB7 = SDA | Fast Mode, 400 kHz |
| USART1 | PA9 = TX / PA10 = RX | 115200, 8-N-1 |
| TIM2 | 内部时钟 + NVIC 中断 | `PSC = 83` / `ARR = 999` → **1 kHz** |
| GPIO | PA1（User Label: `LED`） | 推挽输出，低电平点亮 |

> ⚠️ **TIM2 分频陷阱**：F407 在 **APB1 预分频 ≠ 1** 时，**定时器时钟 = 2 × PCLK1 = 84 MHz**（不是 42 MHz）。
> 按 84 MHz 计算才得到正确的 1 kHz；若误用 42 MHz 会得到 500 Hz，采样率直接差一半。

## 目录结构

```
stm32-edge-ai-node/
├── Core/
│   ├── Inc/            # 头文件（main.h、外设初始化声明）
│   ├── Src/            # 源文件（main.c、外设初始化实现）
│   └── Startup/        # 启动文件（startup_stm32f407vgtx.s）
├── Drivers/
│   ├── CMSIS/          # ARM CMSIS 内核与设备头文件
│   └── STM32F4xx_HAL_Driver/   # ST HAL 库
├── .settings/          # STM32CubeIDE 工程设置
├── stm32-edge-ai-node.ioc      # CubeMX 配置文件（改配置从这里改）
├── STM32F407VGTX_FLASH.ld      # Flash 链接脚本（1024K）
├── STM32F407VGTX_RAM.ld        # RAM 链接脚本
├── .cproject / .project / .mxproject   # CubeIDE 工程文件
└── .gitignore
```

## 构建与烧录

1. 用 **STM32CubeMX** 打开 `stm32-edge-ai-node.ioc` —— 需要改外设/时钟配置时从这里改
2. 用 **STM32CubeIDE** 导入：`File → Open Projects from File System…` → 选本工程目录
3. `Project → Build All` 编译 → 通过 **ST-Link** 烧录

> ⚠️ **工程路径必须为纯英文** —— CubeMX / CubeIDE / GCC / OpenOCD 对含非 ASCII 字符的路径支持不佳。

## 参考资料

- [stm32-hotspot/stm32ai-nanoedge-datalogger](https://github.com/stm32-hotspot/stm32ai-nanoedge-datalogger) —— ST 官方 NanoEdge AI 数据记录器参考实现
- [NanoEdge AI Studio](https://www.st.com/en/development-tools/nanoedgeaistudio.html) —— 自动生成适合 MCU 的 AI 库

---

## 开发日志

本项目按 6 周计划推进，每个里程碑对应一次 commit，可追溯完整开发过程。

| 日期 | 里程碑 | commit |
|---|---|---|
| 2026-10-08 | 工程骨架 + 时钟树（168 MHz） | — |
| 2026-10-08 | 四个外设配置（I2C1 / USART1 / TIM2 / GPIO） | `f54f7fe` |
