# include "./servo/bsp_servo.h"

void Servo_Init(void){

    PWM_Init();
    MPU6050_initialize();
    DMP_Init();
}

/**
  * @brief  设置舵机 1 的角度
  * @param  无
  * @retval 无
  */
void Servo_SetAngle1(float Angle){

  PWM_SetCompare1(Angle / 180 *2000 + 1500);
}

/**
  * @brief  设置舵机 2 的角度
  * @param  无
  * @retval 无
  */
void Servo_SetAngle2(float Angle){

  PWM_SetCompare2(Angle / 180 *2000 + 1500);
}
