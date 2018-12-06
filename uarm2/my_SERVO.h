#ifndef F_CPU 
#define F_CPU 16000000UL
#endif

#ifndef _SERVO_H_
#define _SERVO_H_

#include <util/delay.h>

void Servo_init()
{
	DDRD|= 0xFF;
	TCCR1A |= 1<<WGM11|1<<COM1A0|1<<COM1A1;
	TCCR1B |= 1<<WGM12|1<<WGM13|1<<CS10|1<<CS11;			//Prescaler-64
	ICR1 = 4999;			//to generate 50Hz pulse		//For Prescaler-1(ICR1=19999)
}
// 
// void free_it()
// {
// 	OCR1A = ICR1-700;		//for 180 degree
// }
// 
// void grab_it()
// {
// 	OCR1A = ICR1-75;		//for 180 degree
// }

#endif

// 				OCR1A =ICR1-400;		//for 0 degree
// 				_delay_ms(3000);
// 				OCR1A = ICR1-730;		//for 45 degree
// 				_delay_ms(200);
// 				OCR1A = ICR1-1164;		//fo	 90 degree
// 				_delay_ms(200);
// 				OCR1A = ICR1-1700;		//for 135 degree
// 				_delay_ms(200);