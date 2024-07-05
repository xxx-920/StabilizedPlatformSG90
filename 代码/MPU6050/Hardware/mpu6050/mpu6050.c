# include "./mpu6050/mpu6050.h"
# include "math.h"

#define DEFAULT_MPU_HZ  (200)
#define q30  1073741824.0f


short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = 
{   
    -1, 0, 0,
    0,-1, 0,
    0, 0, 1
};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** 设置陀螺仪范围
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_testConnection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void) {

    IIC_Init();
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-1000度每秒
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
    MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
    MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
}




/**************************************************************************
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
**************************************************************************/
void DMP_Init(void)
{ 
    u8 temp[1]={0};
    i2cRead(0x68,0x75,1,temp);
    printf("mpu_set_sensor complete ......\r\n");
	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
    {
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_set_sensor complete ......\r\n");
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_configure_fifo complete ......\r\n");
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
            printf("mpu_set_sample_rate complete ......\r\n");
        if(!dmp_load_motion_driver_firmware())
            printf("dmp_load_motion_driver_firmware complete ......\r\n");
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
            printf("dmp_set_orientation complete ......\r\n");
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                DMP_FEATURE_GYRO_CAL))
            printf("dmp_enable_feature complete ......\r\n");
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
            printf("dmp_set_fifo_rate complete ......\r\n");
        run_self_test();
        if(!mpu_set_dmp_state(1))
            printf("mpu_set_dmp_state complete ......\r\n");
    }
}

double rotation[2][2];

void rotationMatrix(double theta, double matrix[2][2]) {
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);

    matrix[0][0] = cosTheta;
    matrix[0][1] = -sinTheta;
    matrix[1][0] = sinTheta;
    matrix[1][1] = cosTheta;
}

void rotationMatrixToAngle(double matrix[2][2], double *angle) {
    *angle = atan2(matrix[1][0], matrix[0][0]);
}

// typedef struct {
//     double w; // 实部
//     double x, y, z; // 虚部
// } Quaternion;

// Quaternion q;

/**************************************************************************
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
**************************************************************************/
void Read_DMP(void)
{	
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
    if (sensors & INV_WXYZ_QUAT )
    {    
        q0=quat[0] / q30;
        q1=quat[1] / q30;
        q2=quat[2] / q30;
        q3=quat[3] / q30;
        // q.w = quat[0];
        // q.x = quat[1];
        // q.y = quat[2];
        // q.z = quat[3];
        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll		 
//        Yaw = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;

        // // 计算旋转矩阵的各个元素
        // float m00 = 2.0 * (q.w * q.w + q.x * q.x) - 1.0;
        // float m01 = 2.0 * (q.x * q.y - q.w * q.z);
        // float m02 = 2.0 * (q.x * q.z + q.w * q.y);
        // float m12 = 2.0 * (q.y * q.z - q.w * q.x);
        // float m22 = 2.0 * (q.w * q.w + q.z * q.z) - 1.0;

        // // 计算欧拉角
        // Yaw = atan2(m01, m00);
        // Pitch = asin(-m02);
        // Roll = atan2(m12, m22);
    }

}
