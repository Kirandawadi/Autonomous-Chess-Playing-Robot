/*
 * mpu6050.h
 *
 * Created: 6/29/2018 10:19:00 PM
 *  Author: Kiran Dawadi
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_

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

class mpu6050
{
	uint16_t address_w,address_r;
	public:
	float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z,angle_gyro=0;
	long int angle_gyro_cal_y=0,angle_gyro_cal_x=0,angle_gyro_cal_z=0;
	mpu6050(uint16_t add_w,uint16_t add_r){address_w=add_w; address_r=add_r;}
	void Gyro_Init();
	void MPU_Start_Loc();
	void Read_RawValue();
	float angle();
	void Gyro_cal();

};

void mpu6050::Gyro_Init()								/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(address_w);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(address_w);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(address_w);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(address_w);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x00);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(address_w);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
	
	I2C_Start_Wait(address_w);
	I2C_Write(0x1C);								/* Write to interrupt enable register */
	I2C_Write(0x08);
	I2C_Stop();
}

void mpu6050::MPU_Start_Loc()
{
	I2C_Start_Wait(address_w);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
	I2C_Repeated_Start(address_r);							/* I2C start with device read address */
}

void mpu6050::Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack())-1);
	Acc_y = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack())-1);
	Acc_z = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack())-1);
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack())-1);
	Gyro_y = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack())-1);
	Gyro_z = ~((((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack())-1);
	I2C_Stop();
}
float mpu6050::angle()
{
	Read_RawValue();
	Xa = Acc_x/8,192;								/* Divide raw value by sensitivity scale factor to get real values */
	Ya = Acc_y/8,192;
	Za = Acc_z/8,192;
	
	Xg = (Gyro_x-angle_gyro_cal_x)/131;
	Yg = (Gyro_y-angle_gyro_cal_y)/131;
	Zg = (Gyro_z-angle_gyro_cal_z)/131;
	
	if(TCNT2<200)
	{
		counter += TCNT2;
	}
	
	del_x=(counter/500000);

	//TCNT2=0;
	counter=0;
	
	angle_gyro += Zg*del_x;					/* Convert temperature in °/c using formula */
	angle_yaw  += Xg*del_x;
	double tot = sqrt(Xa*Xa+Ya*Ya+Za*Za);
	float angle1 = asin((float)Ya/tot)*57.2957795130;
	if(set_gyro_angle)
	{
		angle_gyro = angle_gyro*0.96 +angle1*0.04;
	}
	else
	{
		angle_gyro = angle1;
		set_gyro_angle =1;
	}
	
	return angle_gyro;
}

void mpu6050::Gyro_cal()
{
	for(int i=0;i<200;i++)
	{
		Read_RawValue();
		angle_gyro_cal_x += Gyro_x;
		angle_gyro_cal_y += Gyro_y;
		angle_gyro_cal_z += Gyro_z;
		
		_delay_ms(1);
	}
	angle_gyro_cal_x /= 200;
	angle_gyro_cal_y /= 200;
	angle_gyro_cal_z /= 200;
}



#endif /* MPU6050_H_ */