# VL53L8CX ToF 传感器 C++ 库

> **Language/语言**: [English](README.md) | [简体中文](README-zh-CN.md)

一个用于 VL53L8CX 飞行时间（ToF）多区域测距传感器的 C++ 封装库，适用于STM32 HAL。该库在官方 VL53L8CX ULD驱动 C API 之上提供了面向对象的接口。

## 特性

- **面向对象 API**：干净的 C++ 接口，具有适当的错误处理
- **双通信支持**：I2C 和 SPI 接口（可在编译时配置）
- **多区域测距**：支持 4x4 和 8x8 分辨率模式
- **全面的getter**：距离、信号强度、环境光、反射率等
- **运动检测**：内置运动指示器功能

## 硬件要求

- STM32 MCU
- VL53L8CX ToF
- I2C 或 SPI 接口连接，注意I2C需要上拉电阻

## 安装

将以下文件复制到您的 STM32 项目中：

```
vl53l8cx.h
vl53l8cx.cpp
vl53l8cx_uld_platform.h
vl53l8cx_uld_platform.c
VL53L8CX_ULD_API/（整个目录）
```

### 2. 配置您的 IDE

将包含路径添加到您的项目设置中：
- `vl53l8cx.h` 和 `vl53l8cx_uld_platform.h` 的路径
- `VL53L8CX_ULD_API/inc/` 的路径

### 3. 启用通信接口

在CubeMX项目配置中，确保至少启用了 I2C 或 SPI 接口。

## 快速开始

### 基本 I2C 示例

```cpp
#include "i2c.h"
#include "vl53l8cx.h"

VL53L8CX tof(&hi2c1, VL53L8CX_DEFAULT_I2C_ADDRESS);

int cpp_main() {
    
    // 初始化传感器
    auto status = sensor.begin(); // 此处begin内部会向传感器下载ULD的固件，I2C@400kHz耗时大约8秒
    if (status != VL53L8CX::Status::OK) {
        // 处理初始化错误
    } else {
        tof.refreshSettings();  // 刷新设置
    }
    
    // 开始连续测距
    status = sensor.startRanging();
    if (status != VL53L8CX::Status::OK) {
        // 处理错误
        return;
    }
    
    while (1) {
        if (sensor.dataReady()) {
            // 读取测距数据
            status = sensor.readRangingData();
            if (status == VL53L8CX::Status::OK) {
                // 从区域 0，目标 0 获取距离
                bool ok;
                int16_t distance = sensor.getDistanceMm(0, 0, ok);
                if (ok) {
                    // 处理距离测量
                    // ...
                }
            }
        }
        HAL_Delay(10);

        // 完成后停止测距
        // sensor.stopRanging();
    }
    

}
```

### 基本 SPI 示例

```cpp
#include "vl53l8cx.h"
#include "spi.h"

// 使用 SPI 接口创建传感器实例
VL53L8CX sensor(&hspi1, TOF_CS_GPIO_Port, TOF_CS_Pin);

void setup_tof_spi() {
    // 初始化并使用，类似于 I2C 示例
    sensor.begin();
    // ... 其余代码
}

// 注意，SPI 接口我还未测试过
```

## API 参考

### class `VL53L8CX`

#### 构造函数
- `VL53L8CX(I2C_HandleTypeDef *hi2c, uint8_t i2c_address = VL53L8CX_DEFAULT_I2C_ADDRESS)` - I2C 构造函数
- `VL53L8CX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIOx, uint8_t CS_Pin)` - SPI 构造函数

#### 初始化
- `Status begin()` - 初始化传感器
- `bool isAlive()` - 检查传感器是否响应，可作为基础检查
- `Status setIICAddress(uint16_t i2c_address)` - 更改 I2C 地址（仅 I2C）

#### 测距控制
- `Status startRanging()` - 开始连续测距
- `Status stopRanging()` - 停止测距
- `bool dataReady()` - 检查是否有新数据可用

#### 数据访问
- `Status getRangingData(VL53L8CX_ResultsData *p_results)` - 获取测距数据到缓冲区
- `Status readRangingData()` - 更新内部结果缓存
- `const VL53L8CX_ResultsData& results() const` - 获取结果数据的引用，此引用中包含所有数据

#### 单值获取器
- `int16_t getDistanceMm(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint16_t getRangeSigmaMm(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint32_t getSignalPerSpad(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint8_t getReflectancePercent(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint8_t getTargetStatus(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint32_t getAmbientPerSpad(uint8_t zone, bool &ok) const`
- `uint8_t getNbTargetDetected(uint8_t zone, bool &ok) const`
- `uint32_t getNbSpadsEnabled(uint8_t zone, bool &ok) const`

#### 运动指示器（如果启用）
- `uint32_t getMotion(uint8_t motion_idx, bool &ok) const`
- `uint32_t getMotionGlobalIndicator1(bool &ok) const`
- `uint32_t getMotionGlobalIndicator2(bool &ok) const`
- `uint8_t getMotionStatus(bool &ok) const`
- `uint8_t getMotionNbOfDetectedAggregates(bool &ok) const`

#### 配置
- `Status refreshSettings()` - 从传感器刷新设置
- `VL53L8CX_Configuration *getConfig()` - 获取配置指针
- `uint8_t getResolution() const` - 获取当前分辨率（4x4 或 8x8）
- `uint8_t getRangingFrequencyHz() const` - 获取测距频率（Hz）
- `uint32_t getIntegrationTimeMs() const` - 获取积分时间（ms）
- `uint8_t getSharpenerPercent() const` - 获取锐化器百分比
- `uint8_t getTargetOrder() const` - 获取目标顺序
- `bool isSyncPinEnabled() const` - 检查同步引脚是否启用
- `uint32_t getVHVRepeatCount() const` - 获取 VHV 重复计数

#### DCI 操作
- `Status dciReadData(uint8_t *data, uint32_t index, uint16_t data_size)`
- `Status dciWriteData(uint8_t *data, uint32_t index, uint16_t data_size)`
- `Status dciReplaceData(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos)`

### 状态码

`VL53L8CX::Status` 枚举提供以下状态码：

- `OK` - 操作成功
- `TIMEOUT` - 通信超时
- `CORRUPTED_FRAME` - 数据帧损坏
- `LASER_SAFETY` - 激光安全错误
- `XTALK_FAILED` - 串扰校准失败
- `FW_CHECKSUM_FAIL` - 固件校验和错误
- `MCU_ERROR` - MCU 内部错误
- `INVALID_PARAM` - 无效参数
- `ERROR` - 一般错误

## 配置选项

### 平台配置（`vl53l8cx_uld_platform.h`）

- `VL53L8CX_NB_TARGET_PER_ZONE` - 每个区域的目标数量（1-4，默认：1）
- `VL53L8CX_DISABLE_*` 宏 - 禁用特定输出以减少 I2C 流量
- `VL53L8CX_USE_RAW_FORMAT` - 使用固件格式而不是用户格式（提高精度）

### 默认禁用的输出

为了最小化 I2C 数据传输，默认禁用以下输出：
- `VL53L8CX_DISABLE_TARGET_STATUS`
- `VL53L8CX_DISABLE_MOTION_INDICATOR`

如果需要，请在 `vl53l8cx_uld_platform.h` 中启用它们。

## 文件结构

```
VL53L8CX-ToF-Lib/
├── vl53l8cx.h              # C++ 封装类头文件
├── vl53l8cx.cpp            # C++ 封装类实现
├── vl53l8cx_uld_platform.h # 平台抽象层头文件
├── vl53l8cx_uld_platform.c # 平台抽象层实现
├── VL53L8CX_ULD_API/
│   ├── inc/
│   │   ├── vl53l8cx_api.h                 # 主 C API 头文件
│   │   ├── vl53l8cx_buffers.h             # 缓冲区定义
│   │   ├── vl53l8cx_plugin_detection_thresholds.h
│   │   ├── vl53l8cx_plugin_motion_indicator.h
│   │   └── vl53l8cx_plugin_xtalk.h
│   └── src/
│       ├── vl53l8cx_api.c                 # 主 C API 实现
│       ├── vl53l8cx_plugin_detection_thresholds.c
│       ├── vl53l8cx_plugin_motion_indicator.c
│       └── vl53l8cx_plugin_xtalk.c
└── Docs/                   # 参考文档
```

## 版本历史

- **v1.0** (2025.12.09)：初始版本
- **v1.1** (2026.02.23)：修复 I2C 通信超时问题

---

**WilliTourt / 2025.12 V1 Release**
