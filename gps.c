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

int main(){


        Ports_init();
        LCD_init();

        LCD_command (0x01);
        LCD_command (0x80);
        delay_ms(300);
        
         float distance = getDistance(30.078087354233926, 31.273906559461306,30.077870396044233, 31.275209508481343);
        //float distance = getDistance(30.07803610431271, 31.274169123429644,30.0779182293887, 31.274840339591478);

        display_distance(distance);
        delay_ms(500);
        led_output(distance);
    }

void delay_ms(int n){
  int i,j;
  for (i=0 ; i<n ; i++)
  for(j=0; j<3180; j++){}
    }

void delay_us(int n){
  int i,j;
  for (i=0 ; i<n ; i++)
  for(j=0; j<3; j++){}
    }

void Ports_init(){
      SYSCTL_RCGCGPIO_R |= 0x2B;
      while((SYSCTL_PRGPIO_R&0x2B)==0){};
  //Port A
     SYSCTL_RCGCUART_R |= 0x00000001;
     GPIO_PORTA_LOCK_R=0x4C4F434B;
     GPIO_PORTA_CR_R |=0xE2;
     GPIO_PORTA_DEN_R  |= 0xE3;
     GPIO_PORTA_AFSEL_R |= 0x03;
     GPIO_PORTA_PCTL_R  |= 0x00000011;
     GPIO_PORTA_DIR_R = 0xE0;
     GPIO_PORTA_AMSEL_R= 0x00;
     UART0_CTL_R &= ~ UART_CTL_UARTEN;
     UART0_IBRD_R = 104;
     UART0_FBRD_R = 11;
     UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
     UART0_CTL_R |= (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
  //Port B
     GPIO_PORTB_LOCK_R=0x4C4F434B;
     GPIO_PORTB_CR_R|=0xFF;
     GPIO_PORTB_AMSEL_R &=~0xFF;
     GPIO_PORTB_PCTL_R &=~0xFFFFFFFF;
     GPIO_PORTB_AFSEL_R &=~0xFF;
     GPIO_PORTB_DIR_R |=0xFF;
     GPIO_PORTB_DEN_R |=0xFF;
     GPIO_PORTB_DATA_R&=~0xFF;


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
     SYSCTL_RCGCUART_R |= 0x00000004;
     GPIO_PORTD_LOCK_R = 0x4C4F434B;
     GPIO_PORTD_CR_R = 0xC0;
     GPIO_PORTD_DIR_R = 0x00;
     GPIO_PORTD_DEN_R = 0xC0;
     GPIO_PORTD_AFSEL_R = 0xC0;
     GPIO_PORTD_AMSEL_R = 0x00;
     GPIO_PORTD_PCTL_R = 0x11000000;
     UART2_CTL_R &= ~0x0001;
     UART2_IBRD_R = 104;
     UART2_FBRD_R = 11;
     UART2_LCRH_R = 0x0070;
     UART2_CTL_R = 0x00301;

     delay_ms(20);
}
void LCD_command (unsigned char c){

    GPIO_PORTA_DATA_R = 0x00;
    GPIO_PORTB_DATA_R = c;
    GPIO_PORTA_DATA_R = 0x80;
    delay_us(1);
    GPIO_PORTA_DATA_R = 0x00;
    if (c < 4)
        delay_ms(2);
    else
        delay_us(40);


}
void LCD_data (unsigned char d){
    GPIO_PORTA_DATA_R = 0x20;
    GPIO_PORTB_DATA_R = d;
    GPIO_PORTA_DATA_R = 0x80 | 0x20;
    delay_us(1);
    GPIO_PORTA_DATA_R = 0;
    delay_us(40);
}

void LCD_init(){
    LCD_command (0x30);
         delay_ms(5);
         LCD_command (0x30);
         delay_us(100);
         LCD_command (0x30);


         LCD_command (0x38);
         LCD_command (0x06);
         LCD_command (0x01);
         LCD_command (0x0F);
}
long double degreesToRadians(long double degrees) {
    return degrees * PI / 180;
}

float getDistance(float lat1, float  lon1, float lat2, float lon2) {
        int earthRadiusKm = 6371;
        lat1 = degreesToRadians(lat1);
        lat2 = degreesToRadians(lat2);
        lon2 = degreesToRadians(lon2);
        lon1 = degreesToRadians(lon1);
        float lat = lat2 - lat1;
        float lon = lon2 - lon1;
        // Haversine formula :
         // a = sin²(Δφ / 2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ / 2)
         //c = 2 ⋅ atan2(√a, √(1−a))
         //d = R ⋅ c
         float  a = sin(lat / 2) * sin(lat / 2) +
            pow(sin(lon / 2), 2) * cos(lat1) * cos(lat2);
         float c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return earthRadiusKm * c * 1000;
     }

void display_distance(float distance)
     {

       int digit;
         int i ;

         for(i=0; i<3; i++)
      {
         digit =(int)distance%10;
         digit+=48; //to convert from ASCII into the real number
         LCD_command(0x80+(2-i));
         LCD_data(digit);
         distance/=10;
      }

     }

void  led_output(float data ){

     if(data>=100){
       GPIO_PORTF_DATA_R|=0x08; //GREEN
        }
     }
