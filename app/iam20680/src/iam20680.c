#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h> 
#include <board.h>
#include "drv_spi.h"
#include <drv_gpio.h>
#include "iam20680.h"

#define IAM20680_SPI_MAX_SPEED (8 * 1000 * 1000) // M

/*!
 * @brief This API must be called before other APIs. It verifies the chip ID of the sensor.
 */
rt_err_t iam20680_init_simple(struct iam20680_dev *dev)
{
    uint8_t buff;
    uint8_t status = 0x00;

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = IAM20680_SPI_MAX_SPEED; /* Set spi max speed */

    struct rt_spi_device *spi_dev = (struct rt_spi_device *)rt_device_find((dev->settings.spi_name));
    spi_dev->bus->owner = spi_dev;
    rt_spi_configure(spi_dev, &cfg);

    rt_pin_write(dev->settings.spi_cs_pin, PIN_LOW);
    rt_thread_mdelay(1);
    rt_pin_write(dev->settings.spi_cs_pin, PIN_HIGH);

    // Reset driver states.
    status |= iam20680_read_regs((uint8_t)IAM20680_PWR_MGMT_1, &buff, 1, dev);
    buff |= 0x80; // DEVICE_RESET
    status |= iam20680_write_regs((uint8_t)IAM20680_PWR_MGMT_1, &buff, 1, dev);

    rt_thread_mdelay(100);
    
    buff = 0x00;
    while ((buff & (0x80)) != 0x00)
    {
        iam20680_read_regs((uint8_t)IAM20680_PWR_MGMT_1, &buff, 1, dev);
        rt_thread_mdelay(1);
    }

    // Let device select best clock source.
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_PWR_MGMT_1, &buff, 1, dev);
    buff &= ~0x07;  // Clear CLKSEL
    buff |= 0x01;   // Set CLKSEL
    status |= iam20680_write_regs((uint8_t)IAM20680_PWR_MGMT_1, &buff, 1, dev);
    
    // Select ODR.
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_SMPLRT_DIV, &buff, 1, dev);
    buff = dev->settings.sample_div;   // Set ODR
    status |= iam20680_write_regs((uint8_t)IAM20680_SMPLRT_DIV, &buff, 1, dev);
    
    // Select FS range.
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_ACCEL_CONFIG, &buff, 1, dev);
    buff &= 0x18; 
    buff |= dev->settings.acce_fs_sel << 3;
    status |= iam20680_write_regs((uint8_t)IAM20680_ACCEL_CONFIG, &buff, 1, dev);
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_GYRO_CONFIG, &buff, 1, dev);
    buff &= 0x18; 
    buff |= dev->settings.gryo_fs_sel << 3;
    status |= iam20680_write_regs((uint8_t)IAM20680_GYRO_CONFIG, &buff, 1, dev);
    
    // Select filter.
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_ACCEL_CONFIG2, &buff, 1, dev);
    buff &= 0x07; 
    buff |= 5 << 0; 
    status |= iam20680_write_regs((uint8_t)IAM20680_ACCEL_CONFIG2, &buff, 1, dev);
    buff = 0x00;
    status |= iam20680_read_regs((uint8_t)IAM20680_CONFIG, &buff, 1, dev);
    buff &= 0x07; 
    buff |= 5 << 0; 
    status |= iam20680_write_regs((uint8_t)IAM20680_CONFIG, &buff, 1, dev);
    

    return status;
}

/*!
 * @brief This API writes the data to the given register address of the sensor.
 */
rt_err_t iam20680_write_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev)
{
    struct rt_spi_device *spi_dev = (struct rt_spi_device *)rt_device_find((dev->settings.spi_name));
    spi_dev->bus->owner = spi_dev;

	// Write the data.
	reg_addr &= 0x7f;
    
    dev->status = rt_spi_send_then_send(spi_dev, &reg_addr, 1, reg_data, len);

	return dev->status;
}

/*!
 * @brief This api reads the data from the given register address of the sensor.
 */
rt_err_t iam20680_read_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct iam20680_dev *dev)
{

    struct rt_spi_device *spi_dev = (struct rt_spi_device *)rt_device_find((dev->settings.spi_name));
    spi_dev->bus->owner = spi_dev;

	reg_addr |= 0x80;

	// Read the data.
    dev->status = rt_spi_send_then_recv(spi_dev, &reg_addr, 1, reg_data, len);  

	return dev->status;
}

/*!
 * @brief This api gets acceleromter, temperature, and gyrometer data.
 */
rt_err_t iam20680_get_data(struct iam20680_data *data, struct iam20680_dev *dev)
{
    uint8_t buff[14] = {0};     // Accel, temp, and gyro two bytes each.

    dev->status = iam20680_read_regs((uint8_t)IAM20680_ACCEL_XOUT_H, &buff[0], sizeof(buff), dev);
    data->acce.data.acce.x = (buff[0] << 8) | buff[1];
    data->acce.data.acce.y = (buff[2] << 8) | buff[3];
    data->acce.data.acce.z = (buff[4] << 8) | buff[5];
    data->temp.data.temp = (buff[6] << 8) | buff[7];
    data->gyro.data.gyro.x = (buff[8] << 8) | buff[9];
    data->gyro.data.gyro.y = (buff[10] << 8) | buff[11];
    data->gyro.data.gyro.z = (buff[12] << 8) | buff[13];

    return dev->status;
}
