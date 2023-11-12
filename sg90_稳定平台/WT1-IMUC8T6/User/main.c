# include "stm32f10x.h"
# include "./oled/OLED.h"
# include "./uart/bsp_uart.h"
# include "./sg90/sg90_drive.h"
# include "./delay/bsp_delay.h"

extern WT1_IMU_ANGLE WT1_IMU_AngleData;

int main(void){


	OLED_Init();
	MyUSART_Init();
	PWM_Init();

	OLED_ShowString(2, 1, "Roll:  ");
	OLED_ShowString(3, 1, "Pitch: ");
	OLED_ShowString(4, 1, "Yaw:   ");

	printf("Hello\n");
	
	while(1){
		if (USARTx_GetRxFlag() == 1){

			OLED_Showfloat(2, 8, WT1_IMU_AngleData.Roll);
			OLED_Showfloat(3, 8, WT1_IMU_AngleData.Pitch);
			OLED_Showfloat(4, 8, WT1_IMU_AngleData.Yaw);

			printf("Roll = %.1f\n", WT1_IMU_AngleData.Roll);
			printf("Pitch = %.1f\n", WT1_IMU_AngleData.Pitch);	
			printf("Yaw = %.1f\n", WT1_IMU_AngleData.Yaw);	
		}
		Servo_SetAngle1(WT1_IMU_AngleData.Roll);
		Servo_SetAngle2(WT1_IMU_AngleData.Pitch);
	}
}
