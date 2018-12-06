/*
 * uarm_lib.c
 *
 * Created: 6/29/2018 2:47:10 PM
 *  Author: Kiran Dawadi
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "uarm_lib.h"
extern volatile double dcounter;

void motor_anti_clockwise(void)
{
	PORTC = 0x00;
	PORTC &= ~(1<<motor_3_dir);
	PORTC |= 1<<motor_3;
	_delay_ms(1);
	PORTC = 0x00;
	_delay_ms(1);
}

void motor_clockwise(void)
{
	PORTC = 0x00;
	PORTC |= 1<<motor_3_dir;
	PORTC |= 1<<motor_3;
	_delay_ms(1);
	PORTC &= ~(1<<motor_3);
	_delay_ms(1);
}

void motor_halt(void)
{
	PORTC = 0x00;
}

 void set_position_degrees(double angle)
 {
	 angle = angle*11.8;
	 if(angle > dcounter)
	 {
		 while( angle > dcounter)
		 {
			 motor_anti_clockwise();
		 }
	 }
	 if(angle < dcounter)
	 {
		 while( angle < dcounter)
		 {
			 motor_clockwise();
		 }
	 }
	 
	 motor_halt();
 }