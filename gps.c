#include "C:\Users\amr\Desktop\tm4c123gh6pm.h"
#include<stdint.h>
#include<math.h>
#define PI 3.142857

//Function Declarations
void delay_ms(int n);
void delay_us(int n);
void Ports_init();
void LCD_init();
void LCD_command(unsigned char c);
void LCD_data(unsigned char d);
long double degreesToRadians(long double degrees);
float getDistance(float lat1, float  lon1, float lat2, float lon2);
void display_distance(float distance);
void  led_output(float data);

void Ports_init(){
  //Port F
     GPIO_PORTF_LOCK_R=0x4C4F434B;
     GPIO_PORTF_CR_R|=0x0E;
     GPIO_PORTF_AMSEL_R &=~0x0E;
     GPIO_PORTF_PCTL_R &=~0x0000FFF0;
     GPIO_PORTF_AFSEL_R &=~ 0x0E;
     GPIO_PORTF_DIR_R |=0x0E;
     GPIO_PORTF_DEN_R |=0x0E;
     GPIO_PORTF_DATA_R&=~0x0E;

     //Port D for UART
     SYSCTL_RCGCUART_R |=0X04;
     SYSCTL_RCGCGPIO_R |=0X04;
     GPIO_PORTD_LOCK_R=0x4C4F434B; 
     GPIO_PORTD_CR_R|=0xC0; 
     GPIO_PORTD_AMSEL_R &=~0xC0; 
     GPIO_PORTD_PCTL_R |=0x11000000; 
     GPIO_PORTD_AFSEL_R |=0xC0; 
     GPIO_PORTD_DEN_R |=0xC0; 
     UART2_CTL_R &=~0X00000001;
     UART2_IBRD_R=520;
     UART2_FBRD_R=30;
     UART2_LCRH_R|=0X00000070;
     UART2_CTL_R |=0X00000001;

     delay_ms(20);
}

