# include "./tim/bsp_pwm.h"

void PWM_GPIO_Config(void){

    RCC_APB2PeriphClockCmd(PWM_TIM_CH1_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(PWM_TIM_CH2_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef    GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = PWM_TIM_CH1_PIN | PWM_TIM_CH2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_TIM_CH1_PORT, &GPIO_InitStructure);
}

void PWM_Mode_Config(void){

    PWM_TIM_APBxClock_FUN(PWM_TIM_CLK, ENABLE);


    /*时基配置*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_ARR_Period;
    TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_PSC_Prescaler;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIMx, &TIM_TimeBaseInitStructure);

    /*输出通道配置*/
    TIM_OCInitTypeDef   TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(PWM_TIMx, &TIM_OCInitStructure);
    TIM_OC2Init(PWM_TIMx, &TIM_OCInitStructure);
    TIM_Cmd(PWM_TIMx, ENABLE);
}

/**
 *  @brief  PWM 初始化
 *  @param  无
 *  @retval 无
 */
void PWM_Init(void){

    PWM_GPIO_Config();
    PWM_Mode_Config();
}

/**
 *  @brief  设置通道 1占空比
 *  @param  无
 *  @retval 无
 */
void PWM_SetCompare1(uint16_t Compare1){
    
    TIM_SetCompare1(PWM_TIMx, Compare1);
}

/**
 *  @brief  设置通道 2占空比
 *  @param  无
 *  @retval 无
 */
void PWM_SetCompare2(uint16_t Compare1){
    
    TIM_SetCompare2(PWM_TIMx, Compare1);
}
