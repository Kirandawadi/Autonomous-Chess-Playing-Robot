/*
 * Servo.h
 *
 * Created: 7/7/2018 9:24:32 पूर्वाह्न
 *  Author: laceup216
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include <avr/io.h>
#include <util/delay.h>

void Servo_init()
{
	DDRD|= 1<<PIND5;
	TCCR1A |= 1<<WGM11|1<<COM1A0|1<<COM1A1|1<<COM1B0|1<<COM1B1;		//initializing both channels
	TCCR1B |= 1<<WGM12|1<<WGM13|1<<CS10|1<<CS11;			//Prescaler-64
	ICR1 = 4999;			//to generate 50Hz pulse		//For Prescaler-1(ICR1=19999)
}

void release(void)
{
	OCR1A = ICR1-50;			//releasing wala
	_delay_ms(600);
	OCR1A = ICR1-0;
	
}

void grip(void)
{
	OCR1A = ICR1-500;			//grabbing wala
	_delay_ms(600);
	OCR1A = ICR1-0;
}


#endif /* SERVO_H_ */