#ifndef __FILTER_H
#define __FILTER_H

# include "main.h"

extern float angle, angle_dot; 	

void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
