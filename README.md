# 电力参数采集与显示系统

本项目基于 TI MSP 微控制器，结合 FreeRTOS、SEGGER RTT 和 U8G2 图形库，设计并实现了一个可实时采集电压、电流并进行分析与显示的嵌入式系统。系统支持自动增益控制、谐波分析和图形化显示。

## 🧠 项目功能

- 实时采集电压、电流数据（通过 ADC）
- 自动增益调整（5挡增益）
- 交流有效值、有功功率、功率因数、谐波含量等电力参数计算
- 数据经校准处理后显示在屏幕上
- 通过 RTT 输出调试信息，便于调试与数据分析

## 📁 项目结构

```plaintext
.
├── main.c           // 系统入口，初始化 FreeRTOS 和主任务
├── dsp.c            // DSP 处理任务，包括采样、自动增益、算法分析
├── gui.c            // 图形界面处理，SPI 屏幕驱动 + 事件显示
├── button.c         // 按键处理模块
├── dsp.h / gui.h    // 相关头文件
├── edc24balg0731c.h // 算法模块（包括功率分析、THD 分析等）
└── ti_msp_dl_config.* // 由 TI 工具自动生成的配置文件
```

## 📦 依赖项

- **TI DriverLib / SYSCFG_DL_init**: TI 提供的底层外设控制库
- **FreeRTOS**: 实时多任务调度系统
- **SEGGER RTT**: 实时调试输出
- **u8g2**: OLED 显示图形库
- **RTOS Queue & Task APIs**: 用于任务间通信与同步

## 🔧 主要模块说明

### 1. `main.c`

- 初始化系统时钟、外设配置
- 启动默认任务 `StartDefaultTask`
- 默认任务主要负责 LED 闪烁、按钮处理、空闲堆栈输出等

### 2. `dsp.c`

- 实时 ADC 采样与 DMA / FIFO 读取
- 自动调整 PGA 增益（根据信号强弱）
- 调用算法库计算：
  - 有效电压、电流
  - 实时功率
  - 功率因数与谐波分析（THD）

### 3. `gui.c`

- 管理 OLED 图形显示
- SPI 发送驱动
- 使用 U8G2 进行绘图与数据展示
- 利用 FreeRTOS 消息队列与 DSP 模块通信

## 🚀 快速开始

### 硬件要求

- 支持 TI MSP DL 库的 MCU（如 MSPM0 系列）
- OLED 显示屏（SPI 接口）
- 模拟信号输入接口（用于电压电流采样）
- 接线并配置好 OPA 增益模块（支持 5挡）

### 编译环境

- [TI Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO) 或其他支持 MSP 的 IDE
- TI DriverLib
- SEGGER RTT 库
- FreeRTOS

### 编译 & 下载

1. 使用 TI SYSCONFIG 配置外设（SPI, ADC, GPIO, Timer）
2. 编译工程
3. 下载程序到板子
4. 通过 RTT Viewer 查看日志
5. 观察 OLED 屏幕上的电力参数动态变化

## 📝 示例输出

通过 SEGGER RTT 观察调试输出：

```
BOOT! 10:45:12
dsp
NO_CAL G3 U=2.45 I=105.63 P=0.26 
CAL G3 2.96V 93.44mA 0.28W 98.5% 1.2%
```

## 💡 后续功能建议

- 支持蓝牙/WiFi 实时上传数据
- 增加电压、电流波形显示
- 长期数据记录功能

## 📃 License

本项目为教学与研究用途，如需商用请联系原作者授权。

---

如果你希望加上中文注释版的 README 或添加具体图片/波形图演示，也可以告诉我，我可以帮你扩展更详细的文档内容。
