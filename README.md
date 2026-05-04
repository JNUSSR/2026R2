# JNUSSR R2 Robot Firmware

暨南大学 SSR 战队 ROBOCON 2026 赛季 R2 机器人固件仓库。

本项目面向 STM32F427 平台，采用 STM32CubeMX + CMake + FreeRTOS 的工程化流程，提供机器人主控固件、驱动层与任务层的完整实现。

使用 VS Code 时建议清理旧路径缓存，并重新配置 CMake Presets。

## 关键特性

- MCU: STM32F427 系列
- RTOS: FreeRTOS（Middlewares/Third_Party/FreeRTOS）
- HAL: STM32F4xx HAL Driver
- 构建系统: CMake + Ninja（兼容 GCC Arm Embedded 工具链）
- 分层结构: Core / Drivers / Middlewares / User / Protocol / Task

## 目录说明

- Core: CubeMX 生成的核心启动与外设代码
- Drivers: CMSIS 与 HAL 驱动
- Middlewares: FreeRTOS 等中间件
- User: 算法/驱动/模块/任务
- Protocol: 协议与工具（含 MAVLink 相关）
- Task: 机器人业务任务
- cmake: 工具链与 CubeMX CMake 适配脚本

## 构建说明（CMake）

1. 安装依赖
- CMake >= 3.22
- Ninja
- gcc-arm-none-eabi 工具链

2. 配置与构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

默认目标名为 `test_feedback`，链接脚本为 `STM32F427XX_FLASH.ld`。

## 开发建议

- 新业务代码优先放在 User/Task 目录并按现有分层维护
- 尽量避免直接改动 CubeMX 自动生成代码区块
- 日常编译使用 CMake 构建，下载与在线调试可使用 Keil

## 参考

- 框架思路参考: https://github.com/yssickjgd/robowalker_train.git

## 许可证

如无额外声明，按仓库后续约定执行。
