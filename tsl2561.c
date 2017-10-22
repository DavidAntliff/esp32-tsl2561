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

bool _check_device_id(tsl2561_device_t device)
{
    const char * name = NULL;
    switch (device)
    {
        case TSL2561_DEVICE_TSL2560CS: name = "0CS"; break;
        case TSL2561_DEVICE_TSL2561CS: name = "1CS"; break;
        case TSL2561_DEVICE_TSL2560T_FN_CL: name = "0T/FN/CL"; break;
        case TSL2561_DEVICE_TSL2561T_FN_CL: name = "1T/FN/CL"; break;
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
        tsl2561_info->init = true;

        // read the ID register and confirm that it is as expected for this device
        tsl2561_device_t device = TSL2561_DEVICE_INVALID;
        tsl2561_revision_t revision = 0;
        err = tsl2561_device_id(tsl2561_info, &device, &revision);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Detected device ID 0x%02x, revision %d on I2C address 0x%02x", device, revision, smbus_info->address);
            if (_check_device_id(device))
            {
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

esp_err_t tsl2561_device_id(tsl2561_info_t * tsl2561_info, tsl2561_device_t * device, tsl2561_revision_t * revision)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(tsl2561_info) && device && revision)
    {
        uint8_t id = 0;
        err = smbus_read_byte(tsl2561_info->smbus_info, REG_ID | SMB_COMMAND, &id);
        if (err == ESP_OK)
        {
            *device = (tsl2561_device_t)((id >> 4) & 0x0f);
            *revision = (tsl2561_revision_t)(id & 0x0f);
        }
        else
        {
            ESP_LOGE(TAG, "Attempt to read ID register failed");
        }
    }
    return err;
}
