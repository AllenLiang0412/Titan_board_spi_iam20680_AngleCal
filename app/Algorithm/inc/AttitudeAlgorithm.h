#ifndef ATTITUDE_ALGORITHM_H
#define ATTITUDE_ALGORITHM_H

#include "Math.h"
#include "Filter.h"

//==定义
#define OFFSET_AV_NUM 50
#define GRAVITY_ACC_PN16G    2048
#define RANGE_PN2000_TO_DEG  0.0305180f
#define RANGE_PN2000_TO_RAD  0.0005326f
#define RANGE_PN16G_TO_CMSS  0.2395056f

#define GYR_ACC_FILTER 0.20f //陀螺仪加速度计滤波系数

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 MPU_ITEMS ,
};

//==数据声明
typedef struct
{
    float center_pos_cm[VEC_XYZ];
    float gyro_rad[VEC_XYZ];
    float gyro_rad_old[VEC_XYZ];
    float gyro_rad_acc[VEC_XYZ];
    float linear_acc[VEC_XYZ];
}_center_pos_st;
extern _center_pos_st center_pos;



typedef struct
{
  unsigned char surface_CALIBRATE;
    float surface_vec[VEC_XYZ];
    float surface_unitvec[VEC_XYZ];

}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;

typedef struct
{
    unsigned char acc_CALIBRATE;
    unsigned char gyr_CALIBRATE;
    unsigned char acc_z_auto_CALIBRATE;

    short int Acc_Original[VEC_XYZ];
    short int Gyro_Original[VEC_XYZ];

    short int Acc[VEC_XYZ];
    int Acc_cmss[VEC_XYZ];
    float Gyro[VEC_XYZ];
    float Gyro_deg[VEC_XYZ];
    float Gyro_rad[VEC_XYZ];

    short int Tempreature;
    float Tempreature_C;

}_sensor_st;//__attribute__((packed))
extern _sensor_st sensor;

typedef struct
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];
	float hx_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float w_acc[VEC_XYZ];
	float h_acc[VEC_XYZ];
	
	float w_mag[VEC_XYZ];
	
	float gacc_deadzone[VEC_XYZ];
	
	float obs_acc_w[VEC_XYZ];
	float obs_acc_a[VEC_XYZ];
	float gra_acc[VEC_XYZ];
	
	float est_acc_a[VEC_XYZ];
	float est_acc_h[VEC_XYZ];
	float est_acc_w[VEC_XYZ];
	
	float est_speed_h[VEC_XYZ];
	float est_speed_w[VEC_XYZ];

	
	float rol;
	float pit;
	float yaw;
} _imu_st ;
extern _imu_st imu_data;

typedef struct
{
	float gkp;
	float gki;
	
	float mkp;
	float drag_p;
	
	unsigned char G_reset;
	unsigned char M_reset;
	unsigned char G_fix_en;
	unsigned char M_fix_en;
	
	unsigned char obs_en;
}_imu_state_st;
extern _imu_state_st imu_state;

//static
void Center_Pos_Set(void);

//public
void Sensor_Data_Prepare(void);
void Sensor_Basic_Init(void);

void IMU_duty(float);
void IMU_update(_imu_state_st *,float gyr[VEC_XYZ],int acc[VEC_XYZ],_imu_st *imu);
void calculate_RPY(void);

void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ]);

void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ]);
#endif

