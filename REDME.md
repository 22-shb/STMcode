# STMcode - STM32 开发项目

这是一个存放 STM32 相关开发代码的仓库。

## 🛠️ 当前项目内容
- **温度控制系统**：基于 STM32F103，使用 NTC 热敏电阻 + ADC + DMA 实现。
- **电机控制 (FOC)**：针对 2804 电机与 AS5600 编码器的闭环控制（开发中）。

## 📂 目录说明
- `Core/Src`: 核心逻辑代码 (.c)
- `Core/Inc`: 头文件 (.h)
- `MDK-ARM`: Keil MDK 工程文件
- `*.ioc`: STM32CubeMX 配置文件

## 🚀 开发环境
- IDE: Keil uVision5 / VS Code
- 硬件: STM32F103 / STM32F407
- 工具: STM32CubeMX, GitHub Desktop