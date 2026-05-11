#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h> 
#include <board.h>
#include "iam20680.h"
#include "drv_spi.h"
#include "AttitudeAlgorithm.h"

#define IAM20680_CS0_Pin BSP_IO_PORT_01_PIN_13
#define IAM20680_BUS_NAME "spi0"
#define IAM20680_SPI_NAME "iam20680"

struct iam20680_dev iam20680_dev;
struct iam20680_data iam20680_data;

void imu_update_thread_entry(void *parameter)
{
    // 配置spi,配置片选引脚(要在初始化之前配置,因为器件初始化中涉及到引脚操作)
    rt_hw_spi_device_attach(IAM20680_BUS_NAME, IAM20680_SPI_NAME, BSP_IO_PORT_01_PIN_13);
    
    iam20680_dev.settings.spi_cs_pin = IAM20680_CS0_Pin;
    iam20680_dev.settings.bus_name = IAM20680_BUS_NAME;
    iam20680_dev.settings.spi_name = IAM20680_SPI_NAME;
    //SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV), Where INTERNAL_SAMPLE_RATE = 1 kHz
    iam20680_dev.settings.sample_div = 0; //1kHz
    iam20680_dev.settings.gryo_fs_sel = GRYO_FS_SEL_2000;
    iam20680_dev.settings.acce_fs_sel = ACCE_FS_SEL_16G;

    iam20680_init_simple(&iam20680_dev);

    while (1)
    {
        iam20680_get_data(&iam20680_data, &iam20680_dev);

        sensor.Tempreature = iam20680_data.temp.data.temp; //tempreature
        sensor.Tempreature_C = sensor.Tempreature/326.8f + 25 ;//sensor.Tempreature/340.0f + 36.5f;

        //调整物理坐标轴与软件坐标轴方向定义一致
        sensor.Acc_Original[X] = iam20680_data.acce.data.acce.x;
        sensor.Acc_Original[Y] = iam20680_data.acce.data.acce.y;
        sensor.Acc_Original[Z] = iam20680_data.acce.data.acce.z;

        sensor.Gyro_Original[X] = iam20680_data.gyro.data.gyro.x;
        sensor.Gyro_Original[Y] = iam20680_data.gyro.data.gyro.y;
        sensor.Gyro_Original[Z] = iam20680_data.gyro.data.gyro.z;

        Sensor_Data_Prepare();
        IMU_update(&imu_state,sensor.Gyro_rad, sensor.Acc_cmss,&imu_data);

        //rt_kprintf("acce value:[X]:%d [Y]:%d [Z]:%d\n", iam20680_data.acce.data.acce.x, iam20680_data.acce.data.acce.y, iam20680_data.acce.data.acce.z);
        //rt_kprintf("gyro value:[X]:%d [Y]:%d [Z]:%d\n\n", iam20680_data.gyro.data.gyro.x, iam20680_data.gyro.data.gyro.y, iam20680_data.gyro.data.gyro.z);

        rt_thread_mdelay(1);
    }
}

void imu_update_thread_init(void)
{
    rt_thread_t imu_update_thread = rt_thread_create("imu_update_thread", imu_update_thread_entry, RT_NULL, 4096, 5, 10);
    if(imu_update_thread != RT_NULL)
    {
        rt_thread_startup(imu_update_thread);
    }
}
INIT_APP_EXPORT(imu_update_thread_init);
