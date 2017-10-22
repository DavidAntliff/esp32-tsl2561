/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file tsl2561.h
 * @brief Interface definitions for the ESP32-compatible TSL2561 Light to Digital Converter component.
 *
 * This component provides structures and functions that are useful for communicating with the device.
 *
 * Technically, the TSL2561 device is an I2C not SMBus device, however the datasheet makes it clear
 * that most SMBus operations are compatible with this device, so it makes sense to use an SMBus interface
 * to manage communication.
 */

#ifndef TSL2561_H
#define TSL2561_H

#include <stdbool.h>
#include "smbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enum for recognised TSL256x devices
 */
typedef enum
{
    TSL2561_DEVICE_TYPE_INVALID = 0b1111,         ///< Invalid device
    TSL2561_DEVICE_TYPE_TSL2560CS = 0b0000,       ///< TSL2560CS (Chipscale)
    TSL2561_DEVICE_TYPE_TSL2561CS = 0b0001,       ///< TSL2561CS (Chipscale)
    TSL2561_DEVICE_TYPE_TSL2560T_FN_CL = 0b0100,  ///< TSL2560T/FN/CL (TMB-6 or Dual Flat No-Lead-6 or ChipLED-6)
    TSL2561_DEVICE_TYPE_TSL2561T_FN_CL = 0b0101,  ///< TSL2561T/FN/CL (TMB-6 or Dual Flat No-Lead-6 or ChipLED-6)
} tsl2561_device_type_t;

/**
 * @brief Enum for supported integration durations.
 * These durations assume the default internal oscillator frequency of 735 kHz.
 */
typedef enum
{
    TSL2561_INTEGRATION_TIME_13MS = 0x00,   ///< Integrate over 13.7 milliseconds
    TSL2561_INTEGRATION_TIME_101MS = 0x01,  ///< Integrate over 101 milliseconds
    TSL2561_INTEGRATION_TIME_402MS = 0x02,  ///< Integrate over 402 milliseconds
} tsl2561_integration_time_t;

/**
 * @brief Enum for supported gain values.
 */
typedef enum
{
    TSL2561_GAIN_1X = 0x00,
    TSL2561_GAIN_16X = 0x10,
} tsl2561_gain_t;

typedef uint8_t tsl2561_revision_t;    ///< The type of the IC's revision value
typedef uint16_t tsl2561_visible_t;    ///< The type of a visible light measurement value
typedef uint16_t tsl2561_infrared_t;   ///< The type of an infrared light measurement value

/**
 * @brief Structure containing information related to the SMBus protocol.
 */
typedef struct
{
    bool init;                                    ///< True if struct has been initialised, otherwise false
    bool powered;                                 ///< True if the device has been powered up
    smbus_info_t * smbus_info;                    ///< Pointer to associated SMBus info
    tsl2561_device_type_t device_type;            ///< Detected type of device (Chipscale vs T/FN/CL)
    tsl2561_integration_time_t integration_time;  ///< Current integration time for measurements
    tsl2561_gain_t gain;                          ///< Current gain for measurements
} tsl2561_info_t;

/**
 * @brief Construct a new TSL2561 info instance.
 *        New instance should be initialised before calling other functions.
 * @return Pointer to new device info instance, or NULL if it cannot be created.
 */
tsl2561_info_t * tsl2561_malloc(void);

/**
 * @brief Delete an existing TSL2561 info instance.
 * @param[in,out] tsl2561_info Pointer to TSL2561 info instance that will be freed and set to NULL.
 */
void tsl2561_free(tsl2561_info_t ** tsl2561_info);

/**
 * @brief Initialise a TSL2561 info instance with the specified SMBus information.
 * @param[in] tsl2561_info Pointer to TSL2561 info instance.
 * @param[in] smbus_info Pointer to SMBus info instance.
 */
esp_err_t tsl2561_init(tsl2561_info_t * tsl2561_info, smbus_info_t * smbus_info);

/**
 * @brief Retrieve the Device Type ID and Revision number from the device.
 * @param[in] tsl2561_info Pointer to initialised TSL2561 info instance.
 * @param[out] device The retrieved Device Type ID.
 * @param[out] revision The retrieved Device Revision number.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t tsl2561_device_id(const tsl2561_info_t * tsl2561_info, tsl2561_device_type_t * device, tsl2561_revision_t * revision);

/**
 * @brief Set the integration time and gain. These values are set together
 *        as they are programmed via the same register.
 * @param[in] tsl2561_info Pointer to initialised TSL2561 info instance.
 * @param[out] integration_time The integration time to use for the next measurement.
 * @param[out] infrared The gain setting to use for the next measurement.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t tsl2561_set_integration_time_and_gain(tsl2561_info_t * tsl2561_info, tsl2561_integration_time_t integration_time, tsl2561_gain_t gain);

/**
 * @brief Retrieve a visible and infrared light measurement from the device.
 *        This function will sleep until the integration time has passed.
 * @param[in] tsl2561_info Pointer to initialised TSL2561 info instance.
 * @param[out] visible The resultant visible light measurement.
 * @param[out] infrared The resultant infrared light measurement.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t tsl2561_read(tsl2561_info_t * tsl2561_info, tsl2561_visible_t * visible, tsl2561_infrared_t * infrared);

/**
 * @brief Compute the Lux approximation from a visible and infrared light measurement.
 *        The calculation is performed according to the procedure given in the datasheet.
 * @param[in] tsl2561_info Pointer to initialised TSL2561 info instance.
 * @param[in] visible The visible light measurement.
 * @param[in] infrared The infrared light measurement.
 * @return The resulting approximation of the light measurement in Lux.
 */
uint32_t tsl2561_compute_lux(const tsl2561_info_t * tsl2561_info, tsl2561_visible_t visible, tsl2561_infrared_t infrared);

#ifdef __cplusplus
}
#endif

#endif  // TSL2561_H
