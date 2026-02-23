

#pragma once

extern "C" {

#include "vl53l8cx_uld_platform.h"
#include "vl53l8cx_api.h"

}



class VL53L8CX {
    public:

        enum class Status : uint8_t {
            OK = VL53L8CX_STATUS_OK,
            TIMEOUT = VL53L8CX_STATUS_TIMEOUT_ERROR,
            CORRUPTED_FRAME = VL53L8CX_STATUS_CORRUPTED_FRAME,
            LASER_SAFETY = VL53L8CX_STATUS_LASER_SAFETY,
            XTALK_FAILED = VL53L8CX_STATUS_XTALK_FAILED,
            FW_CHECKSUM_FAIL = VL53L8CX_STATUS_FW_CHECKSUM_FAIL,
            MCU_ERROR = VL53L8CX_MCU_ERROR,
            INVALID_PARAM = VL53L8CX_STATUS_INVALID_PARAM,
            ERROR = VL53L8CX_STATUS_ERROR
        };

        #ifdef HAL_I2C_MODULE_ENABLED
        VL53L8CX(I2C_HandleTypeDef *hi2c, uint8_t i2c_address = VL53L8CX_DEFAULT_I2C_ADDRESS);
        #endif
        #ifdef HAL_SPI_MODULE_ENABLED
        VL53L8CX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIOx, uint8_t CS_Pin);
        #endif

        Status begin();

        bool isAlive();
        #if defined(HAL_I2C_MODULE_ENABLED)
        Status setIICAddress(uint16_t i2c_address);
        #endif

        Status startRanging();
        Status stopRanging();
        bool dataReady();

        /* Communication/read helpers: call underlying C API and update internal cache */
        Status getRangingData(VL53L8CX_ResultsData *p_results); /* fills internal _results and immediately get it */
        Status readRangingData(); /* only updates internal _results */

        inline uint8_t  getResolution() const         { return _cfg.platform.resolution;                }
        inline uint8_t  getRangingFrequencyHz() const { return _cfg.platform.ranging_freq_hz;           }
        inline uint8_t  getRangingMode() const        { return _cfg.platform.ranging_mode;              }
        inline uint32_t getIntegrationTimeMs() const  { return _cfg.platform.integrations_period_ms;    }
        inline uint8_t  getSharpenerPercent() const   { return _cfg.platform.sharpener_percent;         }
        inline uint8_t  getTargetOrder() const        { return _cfg.platform.target_order;              }
        inline bool     isSyncPinEnabled() const      { return (bool)_cfg.platform.is_sync_pin_enabled; }
        inline uint32_t getVHVRepeatCount() const     { return _cfg.platform.VHV_repeat_cnt;            }

        inline const VL53L8CX_ResultsData& results() const { return _results; }

        Status refreshSettings(); /* updates platform fields (resolution, freq, integration time, etc.) */

        Status dciReadData(uint8_t *data, uint32_t index, uint16_t data_size);
        Status dciWriteData(uint8_t *data, uint32_t index, uint16_t data_size);
        Status dciReplaceData(uint8_t *data, uint32_t index, uint16_t data_size,
                              uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);

        VL53L8CX_Configuration *getConfig() { return &_cfg; }

        /* Single-value getters from cached results. Each returns the value and
         * sets `ok` to true when the index is valid, false otherwise. */
        int16_t  getDistanceMm(uint8_t zone, uint8_t target_idx, bool &ok) const;
        uint16_t getRangeSigmaMm(uint8_t zone, uint8_t target_idx, bool &ok) const;
        uint32_t getSignalPerSpad(uint8_t zone, uint8_t target_idx, bool &ok) const;
        uint8_t  getReflectancePercent(uint8_t zone, uint8_t target_idx, bool &ok) const;
        uint8_t  getTargetStatus(uint8_t zone, uint8_t target_idx, bool &ok) const;

        uint32_t getAmbientPerSpad(uint8_t zone, bool &ok) const;
        uint8_t  getNbTargetDetected(uint8_t zone, bool &ok) const;
        uint32_t getNbSpadsEnabled(uint8_t zone, bool &ok) const;

        /* Motion indicator getters (if enabled). `motion_idx` range: 0..31 */
        uint32_t getMotion(uint8_t motion_idx, bool &ok) const;
        uint32_t getMotionGlobalIndicator1(bool &ok) const;
        uint32_t getMotionGlobalIndicator2(bool &ok) const;
        uint8_t  getMotionStatus(bool &ok) const;
        uint8_t  getMotionNbOfDetectedAggregates(bool &ok) const;

    private:
        VL53L8CX_Configuration _cfg;
        VL53L8CX_ResultsData _results;

};
