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
 * @file tsl2561.c
 *
 * Acknowledgements to Kevin Townsend for the Adafruit TSL2561 driver: https://github.com/adafruit/Adafruit_TSL2561
 * Acknowledgements to https://github.com/lexruee/tsl2561 for a working reference.
 */

#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "tsl2561.h"

static const char * TAG = "tsl2561";

// Register addresses
#define REG_CONTROL         0x00
#define REG_TIMING          0x01
#define REG_THRESHLOWLOW    0x02
#define REG_THRESHLOWHIGH   0x03
#define REG_THRESHHIGHLOW   0x04
#define REG_THRESHHIGHHIGH  0x05
#define REG_INTERRUPT       0x06
#define REG_ID              0x0A
#define REG_DATA0LOW        0x0C
#define REG_DATA0HIGH       0x0D
#define REG_DATA1LOW        0x0E
#define REG_DATA1HIGH       0x0F

// The following values are bitwise ORed with register addresses to create a command value
#define SMB_BLOCK           0x10  // Transaction to use Block Write/Read protocol
#define SMB_WORD            0x20  // Transaction to use Word Write/Read protocol
#define SMB_CLEAR           0x40  // Clear any pending interrupt (self-clearing)
#define SMB_COMMAND         0x80  // Select command register

#define TSL2561_CONTROL_POWER_UP   0x03
#define TSL2561_CONTROL_POWER_DOWN 0x00

// Device defaults:
#define DEFAULT_INTEGRATION_TIME TSL2561_INTEGRATION_TIME_402MS
#define DEFAULT_GAIN             TSL2561_GAIN_1X

#define CH_SCALE       10      // Scale channel values by 2^10
#define CH_SCALE_TINT0 0x7517  // 322/11 * 2^CH_SCALE
#define CH_SCALE_TINT1 0x0FE7  // 322/81 * 2^CH_SCALE

#define RATIO_SCALE    9       // Scale ratio by 2^9
#define LUX_SCALE      14      // Scale by 2^14

// T, FN, and CL Package coefficients
#define TSL2561_K1T 0x0040
#define TSL2561_B1T 0x01F2
#define TSL2561_M1T 0x01BE
#define TSL2561_K2T 0x0080
#define TSL2561_B2T 0x0214
#define TSL2561_M2T 0x02D1
#define TSL2561_K3T 0x00C0
#define TSL2561_B3T 0x023F
#define TSL2561_M3T 0x037B
#define TSL2561_K4T 0x0100
#define TSL2561_B4T 0x0270
#define TSL2561_M4T 0x03FE
#define TSL2561_K5T 0x0138
#define TSL2561_B5T 0x016F
#define TSL2561_M5T 0x01fC
#define TSL2561_K6T 0x019A
#define TSL2561_B6T 0x00D2
#define TSL2561_M6T 0x00FB
#define TSL2561_K7T 0x029A
#define TSL2561_B7T 0x0018
#define TSL2561_M7T 0x0012
#define TSL2561_K8T 0x029A
#define TSL2561_B8T 0x0000
#define TSL2561_M8T 0x0000

// CS Package coefficients
#define TSL2561_K1C 0x0043
#define TSL2561_B1C 0x0204
#define TSL2561_M1C 0x01AD
#define TSL2561_K2C 0x0085
#define TSL2561_B2C 0x0228
#define TSL2561_M2C 0x02C1
#define TSL2561_K3C 0x00C8
#define TSL2561_B3C 0x0253
#define TSL2561_M3C 0x0363
#define TSL2561_K4C 0x010A
#define TSL2561_B4C 0x0282
#define TSL2561_M4C 0x03DF
#define TSL2561_K5C 0x014D
#define TSL2561_B5C 0x0177
#define TSL2561_M5C 0x01DD
#define TSL2561_K6C 0x019A
#define TSL2561_B6C 0x0101
#define TSL2561_M6C 0x0127
#define TSL2561_K7C 0x029A
#define TSL2561_B7C 0x0037
#define TSL2561_M7C 0x002B
#define TSL2561_K8C 0x029A
#define TSL2561_B8C 0x0000
#define TSL2561_M8C 0x0000

static bool _is_init(const tsl2561_info_t * tsl2561_info)
{
    bool ok = false;
    if (tsl2561_info != NULL)
    {
        if (tsl2561_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "tsl2561_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "tsl2561_info is NULL");
    }
    return ok;
}

static bool _check_device_id(tsl2561_device_type_t device)
{
    const char * name = NULL;
    switch (device)
    {
        case TSL2561_DEVICE_TYPE_TSL2560CS: name = "0CS"; break;
        case TSL2561_DEVICE_TYPE_TSL2561CS: name = "1CS"; break;
        case TSL2561_DEVICE_TYPE_TSL2560T_FN_CL: name = "0T/FN/CL"; break;
        case TSL2561_DEVICE_TYPE_TSL2561T_FN_CL: name = "1T/FN/CL"; break;
        default: break;
    }
    if (name)
    {
        ESP_LOGI(TAG, "Device is TSL256%s", name);
    }
    else
    {
        ESP_LOGW(TAG, "Device is not recognised");
    }
    return name != NULL;
}

static esp_err_t _power_up(tsl2561_info_t * tsl2561_info)
{
    esp_err_t err = ESP_FAIL;
    if (tsl2561_info != NULL)
    {
        if (!tsl2561_info->powered)
        {
            if ((err = smbus_write_byte(tsl2561_info->smbus_info, REG_CONTROL | SMB_COMMAND, TSL2561_CONTROL_POWER_UP)) == ESP_OK)
            {
                tsl2561_info->powered = true;
            }
        }
        else
        {
            ESP_LOGW(TAG, "Device already powered");
            err = ESP_OK;  // not an error
        }
    }
    return err;
}

static esp_err_t _power_down(tsl2561_info_t * tsl2561_info)
{
    esp_err_t err = ESP_FAIL;
    if (tsl2561_info != NULL)
    {
        if (tsl2561_info->powered)
        {
            if ((err = smbus_write_byte(tsl2561_info->smbus_info, REG_CONTROL | SMB_COMMAND, TSL2561_CONTROL_POWER_DOWN)) == ESP_OK)
            {
                tsl2561_info->powered = false;
            }
        }
        else
        {
            ESP_LOGW(TAG, "Device not powered");
            err = ESP_OK;  // not an error
        }
    }
    return err;
}

// Assumes device is already powered up
static esp_err_t _set_integration_time_and_gain(tsl2561_info_t * tsl2561_info, tsl2561_integration_time_t integration_time, tsl2561_gain_t gain)
{
    esp_err_t err = ESP_FAIL;
    if (tsl2561_info != NULL && tsl2561_info->powered)
    {
        if ((err = smbus_write_byte(tsl2561_info->smbus_info, REG_TIMING | SMB_COMMAND, integration_time | gain)) == ESP_OK)
        {
            tsl2561_info->integration_time = integration_time;
            tsl2561_info->gain = gain;
        }
    }
    return err;
}

// Public API

tsl2561_info_t * tsl2561_malloc(void)
{
    tsl2561_info_t * tsl2561_info = malloc(sizeof(*tsl2561_info));
    if (tsl2561_info != NULL)
    {
        memset(tsl2561_info, 0, sizeof(*tsl2561_info));
        ESP_LOGD(TAG, "malloc tsl2561_info_t %p", tsl2561_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc tsl2561_info_t failed");
    }
    return tsl2561_info;
}

void tsl2561_free(tsl2561_info_t ** tsl2561_info)
{
    if (tsl2561_info != NULL && (*tsl2561_info != NULL))
    {
        ESP_LOGD(TAG, "free tsl2561_info_t %p", *tsl2561_info);
        free(*tsl2561_info);
        *tsl2561_info = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "free tsl2561_info_t failed");
    }
}

esp_err_t tsl2561_init(tsl2561_info_t * tsl2561_info, smbus_info_t * smbus_info)
{
    esp_err_t err = ESP_FAIL;
    if (tsl2561_info != NULL)
    {
        tsl2561_info->smbus_info = smbus_info;
        tsl2561_info->powered = false;
        tsl2561_info->integration_time = DEFAULT_INTEGRATION_TIME;
        tsl2561_info->gain = DEFAULT_GAIN;
        tsl2561_info->device_type= TSL2561_DEVICE_TYPE_INVALID;

        tsl2561_info->init = true;

        // read the ID register and confirm that it is as expected for this device
        tsl2561_device_type_t device_type = TSL2561_DEVICE_TYPE_INVALID;
        tsl2561_revision_t revision = 0;
        err = tsl2561_device_id(tsl2561_info, &device_type, &revision);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Detected device ID 0x%02x, revision %d on I2C address 0x%02x", device_type, revision, smbus_info->address);
            if (_check_device_id(device_type))
            {
                tsl2561_info->device_type = device_type;
                err = ESP_OK;
            }
            else
            {
                ESP_LOGE(TAG, "Unsupported device detected");
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "tsl2561_info is NULL");
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t tsl2561_device_id(const tsl2561_info_t * tsl2561_info, tsl2561_device_type_t * device, tsl2561_revision_t * revision)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(tsl2561_info) && device && revision)
    {
        uint8_t id = 0;
        err = smbus_read_byte(tsl2561_info->smbus_info, REG_ID | SMB_COMMAND, &id);
        if (err == ESP_OK)
        {
            *device = (tsl2561_device_type_t)((id >> 4) & 0x0f);
            *revision = (tsl2561_revision_t)(id & 0x0f);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read device ID");
        }
    }
    return err;
}

esp_err_t tsl2561_read(tsl2561_info_t * tsl2561_info, tsl2561_visible_t * visible, tsl2561_infrared_t * infrared)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(tsl2561_info) && visible && infrared)
    {
        if ((err = _power_up(tsl2561_info)) == ESP_OK)
        {
            TickType_t delay = 0;
            switch (tsl2561_info->integration_time)
            {
            case TSL2561_INTEGRATION_TIME_13MS:
                // wait at least 15ms according to Adafruit driver
                delay = 15;
                break;
            case TSL2561_INTEGRATION_TIME_101MS:
                // wait at least 120ms according to Adafruit driver
                delay = 120;
                break;
            default:
                ESP_LOGW(TAG, "Invalid integration time: %d", tsl2561_info->integration_time);
                /* fall through */
            case TSL2561_INTEGRATION_TIME_402MS:
                // wait at least 450ms according to Adafruit driver
                delay = 450;
                break;
            }
            vTaskDelay((delay - 1) / portTICK_RATE_MS + 1);

            uint16_t ch0 = 0;
            uint16_t ch1 = 0;
            if ((err = smbus_read_word(tsl2561_info->smbus_info, REG_DATA0LOW | SMB_COMMAND | SMB_WORD, &ch0)) == ESP_OK)
            {
                if ((err = smbus_read_word(tsl2561_info->smbus_info, REG_DATA1LOW | SMB_COMMAND | SMB_WORD, &ch1)) == ESP_OK)
                {
                    if ((err = _power_down(tsl2561_info)) == ESP_OK)
                    {
                        *visible = ch0 - ch1;
                        *infrared = ch1;
                    }
                }
            }
        }
    }
    return err;
}

esp_err_t tsl2561_set_integration_time_and_gain(tsl2561_info_t * tsl2561_info, tsl2561_integration_time_t integration_time, tsl2561_gain_t gain)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(tsl2561_info))
    {
        if ((err = _power_up(tsl2561_info)) == ESP_OK)
        {
            if ((err = _set_integration_time_and_gain(tsl2561_info, integration_time, gain)) == ESP_OK)
            {
                tsl2561_info->integration_time = integration_time;
                tsl2561_info->gain = gain;
            }

            esp_err_t pderr = _power_down(tsl2561_info);
            err = pderr != ESP_OK ? pderr : err;
        }
    }
    return err;
}

uint32_t tsl2561_compute_lux(const tsl2561_info_t * tsl2561_info, tsl2561_visible_t visible, tsl2561_infrared_t infrared)
{
    uint32_t lux = 0;
    if (_is_init(tsl2561_info))
    {
        uint32_t scale = 0;

        // scale channel values
        switch (tsl2561_info->integration_time)
        {
        case TSL2561_INTEGRATION_TIME_13MS:
            scale = CH_SCALE_TINT0;
            break;
        case TSL2561_INTEGRATION_TIME_101MS:
            scale = CH_SCALE_TINT1;
            break;
        default:
            scale = 1 << CH_SCALE;
        }

        // scale 1x measurement up to 16x
        if (tsl2561_info->gain == TSL2561_GAIN_1X)
        {
            scale <<= 4;
        }

        // convert visible/infrared back into channel data
        uint32_t channel0 = ((visible + infrared) * scale) >> CH_SCALE;
        uint32_t channel1 = (infrared * scale) >> CH_SCALE;

        // find the ratio of the channel values (channel1/channel0)
        // protect against divide by zero
        uint32_t ratio1 = 0;
        if (channel0 != 0)
        {
            ratio1 = (channel1 << (RATIO_SCALE + 1)) / channel0;
        }

        // round the ratio value
        uint32_t ratio = (ratio1 + 1) >> 1;

        // is ratio <= eachBreak ?
        int b = 0, m = 0;

        switch (tsl2561_info->device_type)
        {
        case 1:
            if (ratio <= TSL2561_K1C) {
                b = TSL2561_B1C; m = TSL2561_M1C;
            } else if (ratio <= TSL2561_K2C) {
                b = TSL2561_B2C; m = TSL2561_M2C;
            } else if (ratio <= TSL2561_K3C) {
                b = TSL2561_B3C; m = TSL2561_M3C;
            } else if (ratio <= TSL2561_K4C) {
                b = TSL2561_B4C; m = TSL2561_M4C;
            } else if (ratio <= TSL2561_K5T) {
                b = TSL2561_B5C; m = TSL2561_M5C;
            } else if (ratio <= TSL2561_K6T) {
                b = TSL2561_B6C; m = TSL2561_M6C;
            } else if (ratio <= TSL2561_K7T) {
                b = TSL2561_B7C; m = TSL2561_M7C;
            } else if (ratio > TSL2561_K8C) {
                b = TSL2561_B8C; m = TSL2561_M8C;
            }
            break;

        case 0:
        default:
            if (ratio <= TSL2561_K1T) {
                b = TSL2561_B1T; m = TSL2561_M1T;
            } else if (ratio <= TSL2561_K2T) {
                b = TSL2561_B2T; m = TSL2561_M2T;
            } else if (ratio <= TSL2561_K3T) {
                b = TSL2561_B3T; m = TSL2561_M3T;
            } else if (ratio <= TSL2561_K4T) {
                b = TSL2561_B4T; m = TSL2561_M4T;
            } else if (ratio <= TSL2561_K5T) {
                b = TSL2561_B5T; m = TSL2561_M5T;
            } else if (ratio <= TSL2561_K6T) {
                b = TSL2561_B6T; m = TSL2561_M6T;
            } else if (ratio <= TSL2561_K7T) {
                b = TSL2561_B7T; m = TSL2561_M7T;
            } else if (ratio > TSL2561_K8T) {
                b = TSL2561_B8T; m = TSL2561_M8T;
            }
            break;
        }

        uint32_t temp = (channel0 * b) - (channel1 * m);

        // prevent negative lux values
        if ((channel1 * m) > (channel0 * b))
        {
            temp = 0;
        }

        // round lsb
        temp += (1 << (LUX_SCALE-1));

        // strip off fractional portion
        lux = temp >> LUX_SCALE;

    }
    return lux;
}
