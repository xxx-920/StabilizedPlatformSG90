# include "./servo/bsp_servo.h"

void TIMx_Mode_Config(void){

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  TIM_TimeBaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_Period                = (360-1);
  TIM_TimeBaseInitStructure.TIM_Prescaler             = (1000-1);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStruct);  

  TIM_Cmd(TIM2, ENABLE);
}

void Servo_Init(void){

  PWM_Init();
  MPU6050_initialize();
  DMP_Init();
  TIMx_Mode_Config();
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

extern uint8_t main_flag;

void TIM2_IRQHandler(void){

  static uint8_t main_i = 0;
  if (TIM_GetITStatus(TIM2, TIM_FLAG_Update) == SET){

    if (main_flag == 0) {

      main_i ++;
      if (main_i >= 4){

        main_flag = 1;
        main_i = 0;
      }
    }
    TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
  }
}
