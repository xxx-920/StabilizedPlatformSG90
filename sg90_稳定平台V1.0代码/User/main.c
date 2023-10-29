# include "main.h"

extern float Pitch,Roll,Yaw; 

uint8_t main_flag = 0;
float Angle_Balance; 

int main(void){

    OLED_Init();
    DEBUG_UART_Config();
    Servo_Init();

    OLED_ShowString(2, 1, "Roll: ");
    OLED_ShowString(3, 1, "Pitch: ");
    
    while(1){

        Read_DMP();
        Servo_SetAngle1(Roll);
        Servo_SetAngle2(Pitch);
        if (main_flag){

            OLED_Showfloat(2, 7, Roll);
            OLED_Showfloat(3, 8, Pitch);
            main_flag = 0;
        }
    }
}
