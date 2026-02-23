# VL53L8CX ToF Sensor C++ Library

A C++ wrapper library for the STMicroelectronics VL53L8CX Time-of-Flight (ToF) multi-zone ranging sensor, compatible with STM32 HAL. This library provides an object-oriented interface on top of the official VL53L8CX Ultra Lite Driver (ULD) C API.

## Features

- **Object-Oriented API**: Clean C++ interface with proper error handling
- **Dual Communication Support**: Both I2C and SPI interfaces (configurable at compile time)
- **Multi-Zone Ranging**: Supports 4x4 and 8x8 resolution modes
- **Comprehensive getters**: Distance, signal strength, ambient light, reflectance, and more
- **Motion Detection**: Built-in motion indicator functionality

## Hardware Requirements

- STM32 MCU
- VL53L8CX ToF sensor
- I2C or SPI interface connections, note that I2C requires pull-up resistors

## Installation

Copy the following files to your STM32 project:

```
vl53l8cx.h
vl53l8cx.cpp
vl53l8cx_uld_platform.h
vl53l8cx_uld_platform.c
VL53L8CX_ULD_API/ (entire directory)
```

### 2. Configure Your IDE

Add the include paths to your project settings:
- Path to `vl53l8cx.h` and `vl53l8cx_uld_platform.h`
- Path to `VL53L8CX_ULD_API/inc/`

### 3. Enable Communication Interface

In your CubeMX project configuration, ensure at least I2C or SPI interface is enabled.

## Quick Start

### Basic I2C Example

```cpp
#include "i2c.h"
#include "vl53l8cx.h"

VL53L8CX tof(&hi2c1, VL53L8CX_DEFAULT_I2C_ADDRESS);

int cpp_main() {
    
    // Initialize sensor
    auto status = sensor.begin(); // The begin() function downloads ULD firmware to the sensor, takes about 8 seconds at I2C@400kHz
    if (status != VL53L8CX::Status::OK) {
        // Handle initialization error
    } else {
        tof.refreshSettings();  // Refresh settings
    }
    
    // Start continuous ranging
    status = sensor.startRanging();
    if (status != VL53L8CX::Status::OK) {
        // Handle error
        return;
    }
    
    while (1) {
        if (sensor.dataReady()) {
            // Read ranging data
            status = sensor.readRangingData();
            if (status == VL53L8CX::Status::OK) {
                // Get distance from zone 0, target 0
                bool ok;
                int16_t distance = sensor.getDistanceMm(0, 0, ok);
                if (ok) {
                    // Process distance measurement
                    // ...
                }
            }
        }
    }
    
    // Stop ranging when done
    sensor.stopRanging();
}
```

### Basic SPI Example

```cpp
#include "vl53l8cx.h"
#include "spi.h"

VL53L8CX sensor(&hspi1, TOF_CS_GPIO_Port, TOF_CS_Pin);

void setup_tof_spi() {
    // Initialize and use similarly to I2C example
    sensor.begin();
    // ... rest of the code
}

// Note: SPI interface has not been tested yet
```

## API Reference

### class `VL53L8CX`

#### Constructors
- `VL53L8CX(I2C_HandleTypeDef *hi2c, uint8_t i2c_address = VL53L8CX_DEFAULT_I2C_ADDRESS)` - I2C constructor
- `VL53L8CX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIOx, uint8_t CS_Pin)` - SPI constructor

#### Initialization
- `Status begin()` - Initialize the sensor
- `bool isAlive()` - Check if sensor is responsive, can be used as basic check
- `Status setIICAddress(uint16_t i2c_address)` - Change I2C address (I2C only)

#### Ranging Control
- `Status startRanging()` - Start continuous ranging
- `Status stopRanging()` - Stop ranging
- `bool dataReady()` - Check if new data is available

#### Data Access
- `Status getRangingData(VL53L8CX_ResultsData *p_results)` - Get ranging data to buffer
- `Status readRangingData()` - Update internal results cache
- `const VL53L8CX_ResultsData& results() const` - Get reference to results data, this reference contains all data

#### Single Value Getters
- `int16_t getDistanceMm(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint16_t getRangeSigmaMm(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint32_t getSignalPerSpad(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint8_t getReflectancePercent(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint8_t getTargetStatus(uint8_t zone, uint8_t target_idx, bool &ok) const`
- `uint32_t getAmbientPerSpad(uint8_t zone, bool &ok) const`
- `uint8_t getNbTargetDetected(uint8_t zone, bool &ok) const`
- `uint32_t getNbSpadsEnabled(uint8_t zone, bool &ok) const`

#### Motion Indicator (if enabled)
- `uint32_t getMotion(uint8_t motion_idx, bool &ok) const`
- `uint32_t getMotionGlobalIndicator1(bool &ok) const`
- `uint32_t getMotionGlobalIndicator2(bool &ok) const`
- `uint8_t getMotionStatus(bool &ok) const`
- `uint8_t getMotionNbOfDetectedAggregates(bool &ok) const`

#### Configuration
- `Status refreshSettings()` - Refresh settings from sensor
- `VL53L8CX_Configuration *getConfig()` - Get configuration pointer
- `uint8_t getResolution() const` - Get current resolution (4x4 or 8x8)
- `uint8_t getRangingFrequencyHz() const` - Get ranging frequency in Hz
- `uint32_t getIntegrationTimeMs() const` - Get integration time in ms
- `uint8_t getSharpenerPercent() const` - Get sharpener percentage
- `uint8_t getTargetOrder() const` - Get target order
- `bool isSyncPinEnabled() const` - Check if sync pin is enabled
- `uint32_t getVHVRepeatCount() const` - Get VHV repeat count

#### DCI Operations
- `Status dciReadData(uint8_t *data, uint32_t index, uint16_t data_size)`
- `Status dciWriteData(uint8_t *data, uint32_t index, uint16_t data_size)`
- `Status dciReplaceData(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos)`

### Status Codes

`VL53L8CX::Status` enum provides the following status codes:

- `OK` - Operation successful
- `TIMEOUT` - Communication timeout
- `CORRUPTED_FRAME` - Corrupted data frame
- `LASER_SAFETY` - Laser safety error
- `XTALK_FAILED` - Crosstalk calibration failed
- `FW_CHECKSUM_FAIL` - Firmware checksum error
- `MCU_ERROR` - MCU internal error
- `INVALID_PARAM` - Invalid parameter
- `ERROR` - General error

## Configuration Options

### Platform Configuration (`vl53l8cx_uld_platform.h`)

- `VL53L8CX_NB_TARGET_PER_ZONE` - Number of targets per zone (1-4, default: 1)
- `VL53L8CX_DISABLE_*` macros - Disable specific outputs to reduce I2C traffic
- `VL53L8CX_USE_RAW_FORMAT` - Use firmware format instead of user format (increased precision)

### Default Disabled Outputs

To minimize I2C data transfer, the following outputs are disabled by default:
- `VL53L8CX_DISABLE_TARGET_STATUS`
- `VL53L8CX_DISABLE_MOTION_INDICATOR`

Enable them in `vl53l8cx_uld_platform.h` if needed.

## File Structure

```
VL53L8CX-ToF-Lib/
├── vl53l8cx.h              # C++ wrapper class header
├── vl53l8cx.cpp            # C++ wrapper class implementation
├── vl53l8cx_uld_platform.h # Platform abstraction layer header
├── vl53l8cx_uld_platform.c # Platform abstraction layer implementation
├── VL53L8CX_ULD_API/
│   ├── inc/
│   │   ├── vl53l8cx_api.h                 # Main C API header
│   │   ├── vl53l8cx_buffers.h             # Buffer definitions
│   │   ├── vl53l8cx_plugin_detection_thresholds.h
│   │   ├── vl53l8cx_plugin_motion_indicator.h
│   │   └── vl53l8cx_plugin_xtalk.h
│   └── src/
│       ├── vl53l8cx_api.c                 # Main C API implementation
│       ├── vl53l8cx_plugin_detection_thresholds.c
│       ├── vl53l8cx_plugin_motion_indicator.c
│       └── vl53l8cx_plugin_xtalk.c
└── Docs/                   # Reference documentation
```

## Version History

- **v1.0** (2025.12.09): Initial release
- **v1.1** (2026.02.23): Fixed I2C communication timeout issue

---

**WilliTourt / 2025.12 V1 Release**