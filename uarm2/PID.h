/*
 * PID.h
 *
 * Created: 6/30/2018 9:54:49 AM
 *  Author: Kiran Dawadi
 */ 


#ifndef PID_H_
#define PID_H_

#include "Headers.h"

extern int counter;                                    //To count the time for timer 2 to make it as 16bit timer
extern int receive,j,tem,a,button;                               //Used in USART to receive string
extern char buffer[20], float_[10];                            //used in converting number to string and then Transmiting via Bluetooth
extern char buff_[100];										//used in USART
extern float q1,q2,q3;                                      //Used in receiving angles sent by the user
extern uint8_t set_gyro_angle;
extern uint8_t motor_counter;
extern float Xa,Ya,Za,t,angle_yaw;
extern float Xg,Yg,Zg;
extern double del_x;
extern int converted,state,state_2;
extern float angle_reminder;
extern long int counter_motor;
extern uint8_t le;
extern uint8_t val_rec;
extern volatile double dcounter;
extern uint16_t angle;

class PID
{
	public:
	float pid_error,i_scalar=0,p_scalar=5,d_scalar=0,motor,pid;
	uint8_t port,port_dir;
	int  throttel=0,throttel_counter=0,throttel_previous_memory=0,throttel_previous_step;
	PID(uint8_t port1,uint8_t port1_dir): port(port1),port_dir(port1_dir){}
	void set_PID()
	{
	/*	int rece=0;
		if((UCSRA &(1<<RXC)))
		{
			rece= UDR-48;
		}
		
		switch(rece)
		{
			case 1:
			p_scalar += 0.05;
			break;
			
			case 2:
			p_scalar -= 0.05;
			break;
			
			case 3:
			i_scalar += 0.01;
			break;
			case 4:
			i_scalar -= 0.01;
			break;
			
			case 5:
			d_scalar += 0.05;
			break;
			case 6:
			d_scalar -= 0.05;
			default:
			
			break;
		}*/
	}
	
	void PID_calculate(float angle_gyr,int pid_setpoint=0)
	{
		pid_error = angle_gyr-pid_setpoint;
		float proportional = pid_error * p_scalar;
		static float integral = 0;
		integral += pid_error * i_scalar;
		if(integral >  400) integral = 400; // limit wind-up
		if(integral < -400) integral = -400;
		static float previous_error = 0;
		float derivative = (pid_error - previous_error) * d_scalar;
		previous_error = pid_error;
		pid = proportional+derivative+integral;
		if(pid > 400) pid = 400;
		if(pid < -400)pid = -400;
	    if(pid <5 && pid>-5) pid =0;                                   //Create a dead-band to stop the motors when the robot is balanced
		if(pid > 0)pid = 405 - (1/(pid + 9)) * 5500;
		else if(pid<0) pid= -405 - (1/(pid- 9)) * 5500;
		if(pid > 0)motor = 400 - pid;                                  //Calculate the needed pulse time for `stepper motor controllers
		else if(pid < 0)motor = -400 - pid;
		else motor = 0;
		throttel = motor;
		
	}
	
	void stepper_counter()
	{
		throttel_counter ++;                                           //Increase the throttel_left_counter variable by 1 every time this routine is executed
		if(throttel_counter > throttel_previous_memory)
		{                                                              //If the number of loops is larger then the throttel_previous_memory variable
			throttel_counter = 0;                                      //Reset the throttel_left_counter variable
			throttel_previous_memory = throttel;                       //Load the next throttle_left_motor variable
			if(throttel_previous_memory < 0){                          //If the throttel_previous_memory is negative
				PORTC &= (~(1<<port_dir));                             //Set output 3 low to reverse the direction of the stepper controller
				throttel_previous_memory *= -1;                        //Invert the throttel_previous_memory variable
			}
			else PORTC |= (1<<port_dir);                               //Set output 3 high for a forward direction of the stepper motor
		}
		
		else if(throttel_counter == 1)PORTC |= (1<<port);             //Set output 2 high to create a pulse for the stepper controller
		
		else if(throttel_counter == 2)PORTC &= (~(1<<port));
		
	}
	
	void stepper_angle_calulator()
	{
		throttel_counter ++;                                           //Increase the throttel_left_counter variable by 1 every time this routine is executed
		if(throttel_counter > throttel_previous_memory)
		{                                                              //If the number of loops is larger then the throttel_previous_memory variable
			throttel_counter = 0;                                      //Reset the throttel_left_counter variable
			throttel_previous_memory = throttel;                       //Load the next throttle_left_motor variable
            throttel_previous_step=throttel;
			if(throttel_previous_memory < 0){                          //If the throttel_previous_memory is negative
				PORTC &= (~(1<<port_dir));                             //Set output 3 low to reverse the direction of the stepper controller
				throttel_previous_memory *= -1;                        //Invert the throttel_previous_memory variable
			}
			else PORTC |= (1<<port_dir);                               //Set output 3 high for a forward direction of the stepper motor
		}
		
		else if(throttel_counter == 1)PORTC |= (1<<port);             //Set output 2 high to create a pulse for the stepper controller
		
		else if(throttel_counter == 2)
		{
		PORTC &= (~(1<<port));
			if(throttel_previous_step>0)
				angle_reminder -= 0.039375;
				
			else
				angle_reminder += 0.039375;
		}
	}
	
	void motor_anti_clockwise(void)
	{
		PORTC = 0x00;
		PORTC &= ~(1<<port_dir);
		PORTC |= 1<<port;
		_delay_us(100);
		PORTC = 0x00;
		_delay_us(100);
	}

	void motor_clockwise(void)
	{
		PORTC = 0x00;
		PORTC |= 1<<port_dir;
		PORTC |= 1<<port;
		_delay_us(100);
		PORTC &= ~(1<<port);
		_delay_us(100);
	}

	void motor_halt(void)
	{
		PORTC = 0x00;
	}

	void set_position_degrees(uint16_t deg)
	{
		angle = uint16_t(deg*11.8);				
			if( angle > dcounter)				//while loop is already in the main.cpp
			{
				motor_anti_clockwise();
			}
			if( angle < dcounter)
			{
				motor_clockwise();
			}
		
		motor_halt();
	}
};





#endif /* PID_H_ */