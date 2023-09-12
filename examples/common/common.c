/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bmp3.h"
#include "i2c_locker.h"
#include "common.h"

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID 0xD3

/* Variable to store the device address */
static uint8_t dev_addr = 0;

uint8_t GTXBuffer[512], GRXBuffer[2048];

/*!
 * I2C read function 
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *) intf_ptr; 
    uint16_t DevAddress = dev_addr << 1;
    Protected_I2C_Master_Transmit(&hi2c1, DevAddress, &reg_addr, 1, 100);
    Protected_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *) reg_data, len, 100);
    return 0;
}

/*!
 * I2C write function 
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *) intf_ptr;
    uint16_t DevAddress = dev_addr << 1;
    GTXBuffer[0]        = reg_addr;
    memcpy(&GTXBuffer[1], reg_data, len);
    Protected_I2C_Master_Transmit(&hi2c1, DevAddress, GTXBuffer, len + 1, 100);
    return 0;
    }

/*!
 * Delay function */

void bmp3_delay_us(uint32_t period, void *intf_ptr) {
    uint32_t i;
    while (period--) {
        for (i = 0; i < 84; i++) {
            ;
        }
    }
}

void bmp3_check_rslt(const char api_name[], int8_t rslt) {
    switch (rslt) {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name,
                   rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf) {

    int8_t rslt = BMP3_OK;

    if (bmp3 != NULL) {
        /* Bus configuration : I2C */
        if (intf == BMP3_I2C_INTF) {
            printf("I2C Interface\n");
            dev_addr    = BMP3_ADDR_I2C_PRIM;
            bmp3->read  = bmp3_i2c_read;
            bmp3->write = bmp3_i2c_write;
            bmp3->intf  = BMP3_I2C_INTF;
        }
        /* Bus configuration : SPI */
        else if (intf == BMP3_SPI_INTF) {
            printf("SPI Interface\n");
        }

        bmp3->delay_us = bmp3_delay_us;
        bmp3->intf_ptr = &dev_addr;
    } else {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is an exit point.
 * * Reads the chip ID and calibration data of the sensor.
 */
void bmp3_deinit(struct bmp3_dev *dev) {
    bmp3_soft_reset(dev);
    dev->chip_id = 0;
    dev->dummy_byte = 0;
}

struct bmp3_dev *bmp3_get_dev_singleton() {
    static struct bmp3_dev bmp3_dev_singleton;
    if (bmp3_dev_singleton.intf_ptr == 0) {
        bmp3_interface_init(&bmp3_dev_singleton, BMP3_I2C_INTF);
    }
    return &bmp3_dev_singleton;
}