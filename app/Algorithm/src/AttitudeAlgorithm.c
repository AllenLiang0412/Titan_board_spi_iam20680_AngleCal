#include "AttitudeAlgorithm.h"
#include "rtthread.h"

//需要调用引用的外部变量：
#define X_POS_OFFSET_CM    (0)//(center_pos_cm[X]); //X轴中心偏移存储值
#define Y_POS_OFFSET_CM    (0)//(center_pos_cm[Y]); //Y轴中心偏移存储值
#define Z_POS_OFFSET_CM    (0)//(center_pos_cm[Z]); //Z轴中心偏移存储值

float   acc_offset[VEC_XYZ];    //加速度计零偏
float   gyro_offset[VEC_XYZ];   //陀螺仪零偏
//=========mapping===========

void Sensor_Basic_Init()
{
    /*设置重心相对传感器的偏移量*/
    Center_Pos_Set();

    sensor.acc_z_auto_CALIBRATE = 1; //开机自动对准Z轴
    sensor.gyr_CALIBRATE = 2;//开机自动校准陀螺仪
}
INIT_APP_EXPORT(Sensor_Basic_Init);

_center_pos_st center_pos;
_sensor_st sensor;

int sensor_val[6];
int sensor_val_rot[6];
int sensor_val_ref[6];
//float sensor_val_lpf[2][6];


int sum_temp[7]={0,0,0,0,0,0,0};
int acc_auto_sum_temp[3];
short int acc_z_auto[4];

unsigned short int acc_sum_cnt = 0,gyro_sum_cnt = 0,acc_z_auto_cnt;

short int g_old[VEC_XYZ];
float g_d_sum[VEC_XYZ] = {500,500,500};

void mpu_auto_az()
{
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
}

static char motionlessflag;

void motionless_check(unsigned char dT_ms)
{
    unsigned char t = 0;

    for(unsigned char i = 0;i<3;i++)
    {
        g_d_sum[i] += 3*ABS(sensor.Gyro_Original[i] - g_old[i]) ;

        g_d_sum[i] -= dT_ms ;

        g_d_sum[i] = LIMIT(g_d_sum[i],0,200);

        if( g_d_sum[i] > 10)
        {
            t++;
        }

        g_old[i] = sensor.Gyro_Original[i];
    }

    if(t>=2)
    {
        motionlessflag = 0;
    }
    else
    {
        motionlessflag = 1;
    }

}

void Sensor_Data_Offset()
{
    static unsigned char off_cnt;


    if(sensor.gyr_CALIBRATE || sensor.acc_CALIBRATE || sensor.acc_z_auto_CALIBRATE)
    {



///////////////////复位校准值///////////////////////////
        if(motionlessflag == 0 || sensor_val[A_Z]<(GRAVITY_ACC_PN16G/2))
        {
                gyro_sum_cnt = 0;
                acc_sum_cnt=0;
                acc_z_auto_cnt = 0;

                for(unsigned char j=0;j<3;j++)
                {
                    acc_auto_sum_temp[j] = sum_temp[G_X+j] = sum_temp[A_X+j] = 0;
                }
                sum_temp[TEM] = 0;
        }




///////////////////////////////////////////////////////////
        off_cnt++;
        if(off_cnt>=10)
        {
            off_cnt=0;



            if(sensor.gyr_CALIBRATE)
            {

                gyro_sum_cnt++;

                for(unsigned char i = 0;i<3;i++)
                {
                    sum_temp[G_X+i] += sensor.Gyro_Original[i];
                }
                if( gyro_sum_cnt >= OFFSET_AV_NUM )
                {

                    for(unsigned char i = 0;i<3;i++)
                    {
                        gyro_offset[i] = (float)sum_temp[G_X+i]/OFFSET_AV_NUM;

                        sum_temp[G_X + i] = 0;
                    }
                    gyro_sum_cnt =0;
                    sensor.gyr_CALIBRATE = 0;

                }
            }

            if(sensor.acc_CALIBRATE == 1)
            {
                acc_sum_cnt++;

                sum_temp[A_X] += sensor_val_rot[A_X];
                sum_temp[A_Y] += sensor_val_rot[A_Y];
                sum_temp[A_Z] += sensor_val_rot[A_Z] - GRAVITY_ACC_PN16G;
                sum_temp[TEM] += sensor.Tempreature;

                if( acc_sum_cnt >= OFFSET_AV_NUM )
                {

                    for(unsigned char i=0 ;i<3;i++)
                    {
                        acc_offset[i] = sum_temp[A_X+i]/OFFSET_AV_NUM;

                        sum_temp[A_X + i] = 0;
                    }

                    acc_sum_cnt =0;
                    sensor.acc_CALIBRATE = 0;
                }
            }
        }
    }
}





short int roll_gz_comp;
float wh_matrix[VEC_XYZ][VEC_XYZ] =
{
    {1,0,0},
    {0,1,0},
    {0,0,1}

};

void Center_Pos_Set()
{
    center_pos.center_pos_cm[X] = X_POS_OFFSET_CM;//+0.0f;
    center_pos.center_pos_cm[Y] = Y_POS_OFFSET_CM;//-0.0f;
    center_pos.center_pos_cm[Z] = Z_POS_OFFSET_CM;//+0.0f;
}

static float gyr_f[5][VEC_XYZ],acc_f[5][VEC_XYZ];

void Sensor_Data_Prepare()
{
    float hz = 0 ;

    unsigned char dT_ms;
    static uint32_t current_t,pre_t=0;
    current_t=rt_tick_get();
    dT_ms=current_t-pre_t;

    if(dT_ms != 0) hz = 1000/dT_ms;

    /*静止检测*/
    motionless_check(dT_ms);

    Sensor_Data_Offset(); //校准函数


    /*得出校准后的数据*/
    for(unsigned char i=0;i<3;i++)
    {

        sensor_val[A_X+i] = sensor.Acc_Original[i];

        sensor_val[G_X+i] = sensor.Gyro_Original[i] - gyro_offset[i];
        //sensor_val[G_X+i] = (sensor_val[G_X+i] >>2) <<2;
    }

    /*赋值*/
    for(unsigned char i = 0;i<6;i++)
    {
        sensor_val_rot[i] = sensor_val[i];
    }

    /*数据坐标转90度*/
    sensor_val_ref[G_X] =  sensor_val_rot[G_Y] ;
    sensor_val_ref[G_Y] = -sensor_val_rot[G_X] ;
    sensor_val_ref[G_Z] =  sensor_val_rot[G_Z];


    sensor_val_ref[A_X] =  (sensor_val_rot[A_Y] - acc_offset[Y] ) ;
    sensor_val_ref[A_Y] = -(sensor_val_rot[A_X] - acc_offset[X] ) ;
    sensor_val_ref[A_Z] =  (sensor_val_rot[A_Z] - acc_offset[Z] ) ;

    /*单独校准z轴模长*/
    mpu_auto_az();

//======================================================================

    /*软件低通滤波*/
    for(unsigned char i=0;i<3;i++)
    {
        //
        gyr_f[4][X +i] = (sensor_val_ref[G_X + i] );
        acc_f[4][X +i] = (sensor_val_ref[A_X + i] );
        //
        for(unsigned char j=4;j>0;j--)
        {
            //
            gyr_f[j-1][X +i] += GYR_ACC_FILTER *(gyr_f[j][X +i] - gyr_f[j-1][X +i]);
            acc_f[j-1][X +i] += GYR_ACC_FILTER *(acc_f[j][X +i] - acc_f[j-1][X +i]);
        }

    }

    /*旋转加速度补偿*/
//======================================================================

    for(unsigned char i=0;i<3;i++)
    {
        center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
        center_pos.gyro_rad[i] =  gyr_f[0][X + i] *RANGE_PN2000_TO_RAD;//0.001065f;
        center_pos.gyro_rad_acc[i] = hz *(center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
    }

    center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[Z];
    center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Z];
    center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Y];

//======================================================================
    /*赋值*/
    for(unsigned char i=0;i<3;i++)
    {


        sensor.Gyro[X+i] = gyr_f[0][i];//sensor_val_ref[G_X + i];

        sensor.Acc[X+i] = acc_f[0][i] - center_pos.linear_acc[i]/RANGE_PN16G_TO_CMSS;//sensor_val_ref[A_X+i];//
    }

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
    pre_t=current_t;
}

//世界坐标平面XY转平面航向坐标XY
void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ])
{
	h[X] =  w[X] *  ref_ax[X]  + w[Y] *ref_ax[Y];
	h[Y] =  w[X] *(-ref_ax[Y]) + w[Y] *ref_ax[X];
	
}
//平面航向坐标XY转世界坐标平面XY
void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ])
{
	w[X] = h[X] *ref_ax[X] + h[Y] *(-ref_ax[Y]);
	w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];
	
}

//载体坐标转世界坐标（约定等同与地理坐标）
float att_matrix[3][3]; //必须由姿态解算算出该矩阵
void a2w_3d_trans(float a[VEC_XYZ],float w[VEC_XYZ])
{
		for(unsigned char i = 0;i<3;i++)
		{
			float temp = 0;
			for(unsigned char j = 0;j<3;j++)
			{
				
				temp += a[j] *att_matrix[i][j];
			}
			w[i] = temp;
		}
}

_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

static float vec_err[VEC_XYZ];
static float vec_err_i[VEC_XYZ];
static float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0, //四元素
static float imu_reset_val;		

static unsigned short int reset_cnt;
					 
_imu_state_st imu_state = {1,1,1,1,1,1,1,1};

static float mag_2d_w_vec[2][2] = {{1,0},{1,0}};//地理坐标中，水平面磁场方向恒为南北 (1,0)

extern int sensor_val_ref[];

static unsigned char reset_imu_f;

float imu_test[3];
void IMU_update(_imu_state_st *state,float gyr[VEC_XYZ], int acc[VEC_XYZ],_imu_st *imu)
{
    static uint32_t current_t,pre_t=0;
    unsigned char dT_ms;
    current_t=rt_tick_get();
    dT_ms = current_t-pre_t;
    float dT = dT_ms*1e-3f;

//	const float kp = 0.2f,ki = 0.001f;
//	const float kmp = 0.1f;
    if(reset_imu_f==0 )
    {
        imu_state.G_reset = 1;//自动复位
        sensor.gyr_CALIBRATE = 2;//校准陀螺仪，不保存
        reset_imu_f = 1;     //已经置位复位标记
    }
    /*设置重力互补融合修正kp系数*/
    state->gkp = 0.2f;//0.4f;

    /*设置重力互补融合修正ki系数*/
    state->gki = 0.01f;

	static float kp_use = 0,ki_use = 0,mkp_use = 0;

	float acc_norm_l,acc_norm_l_recip,q_norm_l;
		
	float acc_norm[VEC_XYZ];

	float d_angle[VEC_XYZ];
	


//// 辅助变量用于转换矩阵
//	q0q0 = imu->w * imu->w;
    q0q1 = imu->w * imu->x;
    q0q2 = imu->w * imu->y;
    q1q1 = imu->x * imu->x;
    q1q3 = imu->x * imu->z;
    q2q2 = imu->y * imu->y;
    q2q3 = imu->y * imu->z;
    q3q3 = imu->z * imu->z;
    q1q2 = imu->x * imu->y;
    q0q3 = imu->w * imu->z;


    if(state->obs_en)
    {
        //计算机体坐标下的运动加速度观测量。坐标系为北西天
        for(unsigned char i = 0;i<3;i++)
        {
            int temp = 0;
            for(unsigned char j = 0;j<3;j++)
            {

                temp += imu->obs_acc_w[j] *att_matrix[j][i];//t[i][j] 转置为 t[j][i]
            }
            imu->obs_acc_a[i] = temp;

            imu->gra_acc[i] = acc[i] - imu->obs_acc_a[i];
        }
    }
    else
    {
        for(unsigned char i = 0;i<3;i++)
        {
            imu->gra_acc[i] = acc[i];
        }
    }
//
    acc_norm_l_recip = my_sqrt_reciprocal(my_pow(imu->gra_acc[X]) + my_pow(imu->gra_acc[Y]) + my_pow(imu->gra_acc[Z]));
    acc_norm_l = safe_div(1,acc_norm_l_recip,0);

    // 加速度计的读数，单位化。
    for(unsigned char i = 0;i<3;i++)
    {
        acc_norm[i] = imu->gra_acc[i] *acc_norm_l_recip;
    }

		
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
		
	//水平面方向向量
	float hx_vec_reci = my_sqrt_reciprocal(my_pow(att_matrix[0][0]) + my_pow(att_matrix[1][0]));
	imu->hx_vec[X] = att_matrix[0][0] *hx_vec_reci;
	imu->hx_vec[Y] = att_matrix[1][0] *hx_vec_reci;
	
	
	// 计算载体坐标下的运动加速度。(与姿态解算无关)
    for(unsigned char i = 0;i<3;i++)
    {
        imu->a_acc[i] = (int)(acc[i] - 981 *imu->z_vec[i]);
    }
    

    //计算世界坐标下的运动加速度。坐标系为北西天
    for(unsigned char i = 0;i<3;i++)
    {
        int temp = 0;
        for(unsigned char j = 0;j<3;j++)
        {

            temp += imu->a_acc[j] *att_matrix[i][j];
        }
        imu->w_acc[i] = temp;
    }

    w2h_2d_trans(imu->w_acc,imu_data.hx_vec,imu->h_acc);

    // 测量值与等效重力向量的叉积（计算向量误差）。
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);

    for(unsigned char i = 0;i<3;i++)
    {
        //误差积分
        vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *ki_use;

        d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use ) * dT / 2 ;
    }

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



  /////////////////////修正开关///////////////////////////

    if(state->G_fix_en==0)//重力方向修正
    {
        kp_use = 0;//不修正
    }
    else
    {
        if(state->G_reset == 0)//正常修正
        {
            kp_use = state->gkp;
            ki_use = state->gki;
        }
        else//快速修正，通过增量进行对准
        {
            kp_use = 10.0f;
            ki_use = 0.0f;
            imu_reset_val = (ABS(vec_err[X]) + ABS(vec_err[Y]));

            imu_reset_val = LIMIT(imu_reset_val,0,1.0f);

            if((imu_reset_val < 0.05f) && (state->M_reset == 0))
            {
                //计时
                reset_cnt += 2;
                if(reset_cnt>400)
                {
                    reset_cnt = 0;
                    state->G_reset = 0;//已经对准，清除复位标记
                }
            }
            else
            {
                reset_cnt = 0;
            }
        }
    }

    pre_t=current_t;
}


static float t_temp;
void calculate_RPY()
{
	///////////////////////输出姿态角///////////////////////////////
    t_temp = LIMIT(1 - my_pow(att_matrix[2][0]),0,1);

    if(ABS(imu_data.z_vec[Z])>0.05f)//避免奇点的运算
    {
        imu_data.pit =  fast_atan2(att_matrix[2][0],my_sqrt(t_temp))*57.30f;
        imu_data.rol =  fast_atan2(att_matrix[2][1], att_matrix[2][2])*57.30f;
        imu_data.yaw = -fast_atan2(att_matrix[1][0], att_matrix[0][0])*57.30f;

    }
}

