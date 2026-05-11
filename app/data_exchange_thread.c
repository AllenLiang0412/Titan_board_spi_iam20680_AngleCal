#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h> 
#include <board.h>
#include "AttitudeAlgorithm.h"

void data_exchange_thread(void *parameter)
{

    while (1)
    {
        calculate_RPY();

        rt_kprintf("angle value of Pitch, Roll, Yaw:%0.4f,%0.4f,%0.4f\n", imu_data.pit, imu_data.rol, imu_data.yaw);

        rt_thread_mdelay(10);
    }
}

void data_exchange_thread_init(void)
{
    rt_thread_t data_exchange = rt_thread_create("data_exchange_thread", data_exchange_thread, RT_NULL, 4096, 10, 10);
    if(data_exchange != RT_NULL)
    {
        rt_thread_startup(data_exchange);
    }
}
INIT_APP_EXPORT(data_exchange_thread_init);
