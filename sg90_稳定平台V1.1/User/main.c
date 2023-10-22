# include "main.h"

extern float Pitch,Roll,Yaw; 

uint8_t Way_Angle = 1;
float Angle_Balance; 

int main(void){

    OLED_Init();
    DEBUG_UART_Config();
    Servo_Init();

    while(1){
        Read_DMP();
        Servo_SetAngle1(Roll);
        Servo_SetAngle2(Pitch);
        // OLED_ShowString(2, 1, "Roll: ");
        // OLED_ShowString(3, 1, "Pitch: ");
        // OLED_ShowString(4, 1, "Yaw: ");
        // OLED_Showfloat(2, 7, Roll);
        // OLED_Showfloat(3, 8, Pitch);
        // OLED_Showfloat(4, 6, Yaw);
    }
}
