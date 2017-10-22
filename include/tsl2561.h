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
    TSL2561_DEVICE_INVALID = 0b1111,
    TSL2561_DEVICE_TSL2560CS = 0b0000,
    TSL2561_DEVICE_TSL2561CS = 0b0001,
    TSL2561_DEVICE_TSL2560T_FN_CL = 0b0100,
    TSL2561_DEVICE_TSL2561T_FN_CL = 0b0101,
} tsl2561_device_t;

typedef uint8_t tsl2561_revision_t;

/**
 * @brief Structure containing information related to the SMBus protocol.
 */
typedef struct
{
    bool init;                     ///< True if struct has been initialised, otherwise false
    smbus_info_t * smbus_info;     ///< Pointer to associated SMBus info
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

esp_err_t tsl2561_device_id(tsl2561_info_t * tsl2561_info, tsl2561_device_t * device, tsl2561_revision_t * revision);



#ifdef __cplusplus
}
#endif

#endif  // TSL2561_H
