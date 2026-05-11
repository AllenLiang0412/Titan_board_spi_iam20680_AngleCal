#ifndef __IAM20680_H
#define __IAM20680_H

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * INCLUDES
 ****************************************************************************/ 
/**
 * @brief Required includes
 */
#include <stdio.h>
#include <stdint.h>

#include <rtthread.h>
#include <rtdevice.h>

/*****************************************************************************
 * MACROS AND DEFINES 
 ****************************************************************************/ 
/**
 * @brief Register addresses
 */ 
#define IAM20680_SELF_TEST_X_GYRO   0x00
#define IAM20680_SELF_TEST_Y_GYRO   0x01
#define IAM20680_SELF_TEST_Z_GYRO   0x02
#define IAM20680_SELF_TEST_X_ACCEL  0x0D
#define IAM20680_SELF_TEST_Y_ACCEL  0x0E
#define IAM20680_SELF_TEST_Z_ACCEL  0x0F
#define IAM20680_XG_OFFS_USRH       0x13
#define IAM20680_XG_OFFS_USRL       0x14
#define IAM20680_YG_OFFS_USRH       0x15
#define IAM20680_YG_OFFS_USRL       0x16
#define IAM20680_ZG_OFFS_USRH       0x17
#define IAM20680_ZG_OFFS_USRL       0x18
#define IAM20680_SMPLRT_DIV         0x19
#define IAM20680_CONFIG             0x1A
#define IAM20680_GYRO_CONFIG        0x1B
#define IAM20680_ACCEL_CONFIG       0x1C
#define IAM20680_ACCEL_CONFIG2      0x1D
#define IAM20680_LP_MODE_CFG        0x1E
#define IAM20680_ACCEL_WOM_THR      0x1F
#define IAM20680_FIFO_EN            0x23
#define IAM20680_FSYNC_INT          0x36
#define IAM20680_INT_PIN_CFG        0x37
#define IAM20680_INT_ENABLE         0x38
#define IAM20680_INT_STATUS         0x3A
#define IAM20680_ACCEL_XOUT_H       0x3B
#define IAM20680_ACCEL_XOUT_L       0x3C
#define IAM20680_ACCEL_YOUT_H       0x3D
#define IAM20680_ACCEL_YOUT_L       0x3E
#define IAM20680_ACCEL_ZOUT_H       0x3F
#define IAM20680_ACCEL_ZOUT_L       0x40
#define IAM20680_TEMP_OUT_H         0x41
#define IAM20680_TEMP_OUT_L         0x42
#define IAM20680_GYRO_XOUT_H        0x43
#define IAM20680_GYRO_XOUT_L        0x44
#define IAM20680_GYRO_YOUT_H        0x45
#define IAM20680_GYRO_YOUT_L        0x46
#define IAM20680_GYRO_ZOUT_H        0x47
#define IAM20680_GYRO_ZOUT_L        0x48
#define IAM20680_SIGNAL_PATH_RESET  0x68
#define IAM20680_ACCEL_INTEL_CTRL   0x69
#define IAM20680_USER_CTRL          0x6A
#define IAM20680_PWR_MGMT_1         0x6B
#define IAM20680_PWR_MGMT_2         0x6C
#define IAM20680_FIFO_COUNTH        0x72
#define IAM20680_FIFO_COUNTL        0x73
#define IAM20680_FIFO_R_W           0x74
#define IAM20680_WHO_AM_I           0x75
#define IAM20680_XA_OFFSET_H        0x77
#define IAM20680_XA_OFFSET_L        0x78
#define IAM20680_YA_OFFSET_H        0x7A
#define IAM20680_YA_OFFSET_L        0x7B
#define IAM20680_ZA_OFFSET_H        0x7D
#define IAM20680_ZA_OFFSET_L        0x7E

/**\name Status */
#define IAM20680_OK     0x00 /*< OK */
#define IAM20680_ERR    0x01 /*< ERROR */
 
/**\name Who Am I */
#define IAM20680_CHIP_ID    0xA9

/**\name Interface */
#define IAM20680_SPI    0x00
#define IAM20680_I2C    0x01

/** Gyro Full Scale Select:*/
enum{
    GRYO_FS_SEL_250 = 0, //±250 dps
    GRYO_FS_SEL_500 = 1, //±500 dps
    GRYO_FS_SEL_1000 = 2, //±1000 dps
    GRYO_FS_SEL_2000 = 3 //±2000 dps
};

/** Acce Full Scale Select:*/
enum{
    ACCE_FS_SEL_2G = 0, //±2g (00)
    ACCE_FS_SEL_4G = 1, //±4g (01)
    ACCE_FS_SEL_8G = 2, //±8g (10)
    ACCE_FS_SEL_16G = 3 //±16g (11)
};

/**
 * @brief IAM-20680 accelerometer and gyrometer data.
 */
struct iam20680_data {
    struct rt_sensor_data acce;
    struct rt_sensor_data temp;       /*< Temperature data */
    struct rt_sensor_data gyro;
};

/**
 * @brief IAM-20680 register settings.
 */
struct iam20680_settings {
    char* bus_name;
    char* spi_name;
    char  sample_div; //SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV), Where INTERNAL_SAMPLE_RATE = 1 kHz
    char  gryo_fs_sel;
    char  acce_fs_sel;
    rt_base_t spi_cs_pin;
};

/**
 * @brief IAM-20680 device parameters.
 */
struct iam20680_dev {
    struct iam20680_settings settings;  /*< Sensor settings */
    uint8_t status;                     /*< Returned status of read/write functions */
    uint8_t chip_id;                    /*< Chip ID */
};


/*****************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ****************************************************************************/ 

rt_err_t  iam20680_init_simple(struct iam20680_dev *dev);

/**
 * \ingroup iam20680
 * \defgroup iam20680ApiRegister Registers
 * @brief Generic API for accessing sensor registers
 */

/*!
 * \ingroup iam20680ApiRegister
 * \page iam20680_api_iam20680_write_regs iam20680_write_regs
 * \code
 * uint8_t iam20680_write_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address of the sensor
 *
 * @param[in] reg_addr  : Register address to where the data is to be written.
 * @param[in] reg_data  : Pointer to data buffer which is to be written in the reg_addr of sensor.
 * @param[in] len       : Number of bytes of data to write.
 * @param[in, out]      : Structure instance of iam20680_dev.
 *
 * @regurn Result of API execution status.
 *
 * @retval 0 -> Success.
 * @retvan Non-zero -> Fail. 
 *
 */
rt_err_t  iam20680_write_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev);

/*!
 * \ingroup iam20680ApiRegister
 * \page iam20680_api_iam20680_read_regs iam20680_read_regs
 * \code
 * uint8_t iam20680_read_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address of the sensor
 *
 * @param[in] reg_addr  : Register address from where the data is to be read.
 * @param[in] reg_data  : Pointer to data buffer to store the read data.
 * @param[in] len       : Number of bytes of data to be read.
 * @param[in, out]      : Structure instance of iam20680_dev.
 *
 * @regurn Result of API execution status.
 *
 * @retval 0 -> Success.
 * @retvan Non-zero -> Fail. 
 *
 */
rt_err_t  iam20680_read_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev);

/*!
 * \ingroup iam20680ApiRegister
 * \page iam20680_api_iam20680_delay iam20680_delay
 * \code
 * uint8_t iam20680_delay(uint32_t delay, iam20680_dev *dev);
 * \endcode
 * @details This API provides a blocking delay with one ms resolution
 *
 * @param[in] delay     : Delay in ms.
 * @param[in, out]      : Structure instance of iam20680_dev.
 *
 * @regurn Result of API execution status.
 *
 * @retval 0 -> Success.
 * @retvan Non-zero -> Fail. 
 *
 */

rt_err_t  iam20680_get_data(struct iam20680_data *data, struct iam20680_dev *dev);


#ifdef __cplusplus
}
#endif

#endif /* __IAM20680_H */
