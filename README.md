使用Titanboard读取iam20680，通过互补滤波进行姿态解算得到姿态角度

1、数据预处理
软件上电和复位时，通过静止检测后，进行加速度计零偏计算

    if(sensor.acc_z_auto_CALIBRATE)
    {

        acc_z_auto_cnt++;

        acc_auto_sum_temp[0] += sensor_val_ref[A_X];
        acc_auto_sum_temp[1] += sensor_val_ref[A_Y];
        acc_auto_sum_temp[2] += sensor_val_rot[A_Z];

        if(acc_z_auto_cnt>=OFFSET_AV_NUM)
        {

            sensor.acc_z_auto_CALIBRATE = 0;
            acc_z_auto_cnt = 0;

            for(unsigned char i = 0;i<3;i++)
            {
                acc_z_auto[i] = acc_auto_sum_temp[i]/OFFSET_AV_NUM;

                acc_auto_sum_temp[i] = 0;
            }

            acc_z_auto[3] = my_sqrt( GRAVITY_ACC_PN16G*GRAVITY_ACC_PN16G - (my_pow(acc_z_auto[0]) + my_pow(acc_z_auto[1])) );

            acc_offset[Z] = acc_z_auto[2] - acc_z_auto[3];


        }

    }
 软件滤波后，根据传感器初始化时配置的量程进行标幺值到有名值的折算

    /*转换单位*/
    for(unsigned char i =0 ;i<3;i++)
    {
        /*陀螺仪转换到度每秒，量程+-2000度*/
        sensor.Gyro_deg[i] = sensor.Gyro[i] *RANGE_PN2000_TO_DEG ;//  /65535 * 2000; +-2000度

        /*陀螺仪转换到弧度度每秒，量程+-2000度*/
        sensor.Gyro_rad[i] = sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;//  /65535 * 2000 *pi/180

        /*加速度计转换到厘米每平方秒，量程+-8G*/
        sensor.Acc_cmss[i] = sensor.Acc[i] *RANGE_PN16G_TO_CMSS ;//   /65535 * 16*981; +-16G

    }
  
2、线性滤波姿态解算

由上一周期的四元素计算旋转矩阵

    q0q1 = imu->w * imu->x;
    q0q2 = imu->w * imu->y;
    q1q1 = imu->x * imu->x;
    q1q3 = imu->x * imu->z;
    q2q2 = imu->y * imu->y;
    q2q3 = imu->y * imu->z;
    q3q3 = imu->z * imu->z;
    q1q2 = imu->x * imu->y;
    q0q3 = imu->w * imu->z;

	// 载体坐标下的x方向向量，单位化。RCb 与 (1 0 0)的转置相乘
    att_matrix[0][0] = imu->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
    att_matrix[0][1] = imu->x_vec[Y] = 2*q1q2 - 2*q0q3;
    att_matrix[0][2] = imu->x_vec[Z] = 2*q1q3 + 2*q0q2;
		
	// 载体坐标下的y方向向量，单位化。RCb 与 (0 1 0)的转置相乘
    att_matrix[1][0] = imu->y_vec[X] = 2*q1q2 + 2*q0q3;
    att_matrix[1][1] = imu->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
    att_matrix[1][2] = imu->y_vec[Z] = 2*q2q3 - 2*q0q1;
		
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。RCb 与 (0 0 1)的转置相乘
    att_matrix[2][0] = imu->z_vec[X] = 2*q1q3 - 2*q0q2;
    att_matrix[2][1] = imu->z_vec[Y] = 2*q2q3 + 2*q0q1;
    att_matrix[2][2] = imu->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);
  

  修正后的三轴加速度进行单位化，然后计算与重力向量的误差

    // 测量值与等效重力向量的叉积（计算向量误差）。
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);

    

使用一阶龙格库塔法求解欧拉角的微分方程，采用线性滤波的方式融合加速度和角速度数据，差分计算角度增量

    for(unsigned char i = 0;i<3;i++)
    {
        //误差积分
        vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *ki_use;

        d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use ) * dT / 2 ;
    }
  

根据上一周期四元素姿态及角度增量差分计算得到当前周期的四元素姿态

    // 计算姿态。
    imu->w = imu->w            - imu->x*d_angle[X] - imu->y*d_angle[Y] - imu->z*d_angle[Z];
    imu->x = imu->w*d_angle[X] + imu->x            + imu->y*d_angle[Z] - imu->z*d_angle[Y];
    imu->y = imu->w*d_angle[Y] - imu->x*d_angle[Z] + imu->y            + imu->z*d_angle[X];
    imu->z = imu->w*d_angle[Z] + imu->x*d_angle[Y] - imu->y*d_angle[X] + imu->z;

    q_norm_l = my_sqrt_reciprocal(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w *= q_norm_l;
    imu->x *= q_norm_l;
    imu->y *= q_norm_l;
    imu->z *= q_norm_l;
 

3、计算三轴欧拉角

由旋转矩阵计算欧拉角

	///////////////////////输出姿态角///////////////////////////////
    t_temp = LIMIT(1 - my_pow(att_matrix[2][0]),0,1);

    if(ABS(imu_data.z_vec[Z])>0.05f)//避免奇点的运算
    {
        imu_data.pit =  fast_atan2(att_matrix[2][0],my_sqrt(t_temp))*57.30f;
        imu_data.rol =  fast_atan2(att_matrix[2][1], att_matrix[2][2])*57.30f;
        imu_data.yaw = -fast_atan2(att_matrix[1][0], att_matrix[0][0])*57.30f;

    }
  

在1ms线程中执行spi读取6轴原始数据，然后进行姿态解算

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
 

在10ms线程中执行旋转矩阵计算欧拉角度，然后通过串口发送到上位机

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
