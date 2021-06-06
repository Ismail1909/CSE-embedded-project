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


