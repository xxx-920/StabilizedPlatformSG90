# ifndef _BSP_PWM_H
# define _BSP_PWM_H

# include "main.h"

/*定时器定义*/
# define PWM_TIMx                   TIM3                  
# define PWM_TIM_APBxClock_FUN      RCC_APB1PeriphClockCmd
# define PWM_TIM_CLK                RCC_APB1Periph_TIM3

/*配置ARR 和PSC*/
# define PWM_ARR_Period             (20000-1)
# define PWM_PSC_Prescaler          (72-1)

/*输出通道1*/
# define PWM_TIM_CH1_GPIO_CLK       RCC_APB2Periph_GPIOA
# define PWM_TIM_CH1_PORT           GPIOA
# define PWM_TIM_CH1_PIN            GPIO_Pin_6

/*输出通道2*/
# define PWM_TIM_CH2_GPIO_CLK       RCC_APB2Periph_GPIOA
# define PWM_TIM_CH2_PORT           GPIOA
# define PWM_TIM_CH2_PIN            GPIO_Pin_7

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare1);
void PWM_SetCompare2(uint16_t Compare1);

# endif
