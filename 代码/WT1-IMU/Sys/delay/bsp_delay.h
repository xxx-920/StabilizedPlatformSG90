# ifndef _BSP_DELAY_H
# define _BSP_DELAY_H

# include "stm32f10x.h"

/*----------------------函数定义----------------------*/
void Delay_us(uint32_t xus);                    //微妙延时
void Delay_ms(uint32_t xms);                    //毫秒延时
void Delay_s(uint32_t xs);                      //秒延时

# endif /*_BSP_DELAY*/
