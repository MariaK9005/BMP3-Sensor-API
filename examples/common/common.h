/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bmp3.h"

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bmp3      : Structure instance of bmp3_dev
 *  @param[in] intf     : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */

void bmp3_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmp3_check_rslt(const char api_name[], int8_t rslt);

/*!
 * \ingroup bmp3ApiDeinit
 * \page bmp3_api_bmp3_deinit bmp3_deinit
 * \code
 * int8_t bmp3_deinit(struct bmp3_dev *dev);
 * \endcode
 * @details This API is the entry point.
 * Performs the selection of I2C/SPI read mechanism according to the
 * selected interface and reads the chip-id and calibration data of the sensor.
 *
 *  @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
void bmp3_deinit(struct bmp3_dev *dev);

struct bmp3_dev *get_bmp3_dev_singleton();

#ifdef __cplusplus
}
#endif /*__cplusplus */