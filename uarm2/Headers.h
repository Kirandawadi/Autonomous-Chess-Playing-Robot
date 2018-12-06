/*
 * Headers.h
 *
 * Created: 6/26/2018 3:48:51 PM
 *  Author: Kiran Dawadi
 */ 


#ifndef HEADERS_H_
#define HEADERS_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "USART_RS232_H_file.h"
#include "uarm_lib.h"
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"
#include "lcd.h"
#include "uarm_lib.h"
#include "mpu6050.h"
#include "PID.h"
#include "Servo.h"




#endif /* HEADERS_H_ */