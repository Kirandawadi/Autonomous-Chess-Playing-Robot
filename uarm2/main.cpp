/*
 * uarm_swift_pro.cpp
 *
 * Created: 1/13/2018 3:41:12 AM
 * Author : Kiran Dawadi
 */ 

#include "Headers.h"
int counter=0;                                          //To count the time for timer 2 to make it as 16bit timer
int receive=0,j=0,tem=0,a=0,button;                               //Used in USART to receive string
char buffer[20], float_[10];                            //used in converting number to string and then Transmiting via Bluetooth
char buff_[100]={0};										//used in USART
float q1=50,q2=0,q3=0;                                      //Used in receiving angles sent by the user
int pos = 25;
uint8_t set_gyro_angle=0;
uint8_t motor_counter=0;
float Xa,Ya,Za,t,angle_yaw=0;
float Xg=0,Yg=0,Zg=0;
double del_x;
int converted,state=0,state_2=0;
float angle_reminder=0;
long int counter_motor=0;
uint8_t le=0;
uint8_t val_rec=0;
volatile double dcounter;
uint16_t angle;
#define bit_is_set(sfr,bit) (_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr,bit) (!(_SFR_BYTE(sfr) & _BV(bit)))

void clean_the_buffer(void);

PID M1(motor_1,motor_1_dir);
PID M2(motor_2,motor_2_dir);
PID M3(motor_3,motor_3_dir);
		
int main(void)
{   
	sei();                                     //Global interrupt enable
	float ang1=0,ang2=0;
	int my_angle2 = 0 , my_angle1 = 0 , temp = 0;
	uint16_t round_error = 0;
	double degrees;
	DDRC = 0xFF;				//Output side for motor controllers
	DDRB &= ~(1<<PINB0);		//Input for interrupt of Encoder(PB0)
	DDRD &= ~(1<<PD2);			//Input for interrupt of Encoder(INT0)	
	GICR |= (1<<INT0);			//Activate Interrupt 0
	MCUCR |= (1<<ISC00);		//Interrupt for both rising and falling edge
	
	/*Initialization of class mpu6050 with its write and read value*/
	//mpu6050 mp1(0xD0,0xD1);                  
	//mpu6050 mp2(0xD2,0xD3);  
	                    
	USART_Init(38400);			//Initialize UART with 38400 baud
    I2C_Init();
    //mp1.Gyro_Init();
    //mp2.Gyro_Init();
	//Servo_init();
	TIMSK |= 1<<OCIE0 | 1<<OCIE2;             //Interrupt enable for Timer 0 and 2
	TCCR2 |= 1<<WGM21 | 1<<CS21 | 1<<CS20;    //Prescaler 32,CTC mode
	OCR2 =200;
	TCCR0 |= 1<<WGM01 | 1<<CS01;  //CTC mode and Prescaler to 8
	OCR0 =156;// 20us interrupt
	
 	//USART_SendString("Calibrating your mpu6050....\n");
  	//mp1.Gyro_cal();
    //mp2.Gyro_cal();
 	//USART_SendString("Done Calibrating \n");
	
    TCNT2 =0;
    while (1) 
    {
		M3.set_position_degrees(25);
		M1.PID_calculate(0,50);
		M2.PID_calculate(50,0);
		/*ang1 = mp1.angle();
		my_angle1 = (-1)*int(ang1);
		ang2 = mp2.angle();
		my_angle2 = (-1)*int(ang2);			//If angle is in minus , send i before ending character
		
		//my_angle = abs(my_angle);
		
		    				//dtostrf(my_angle, 3, 2, float_ );
		    				sprintf(buffer,"%d",my_angle1);
		   				   USART_SendString(buffer);
						   USART_SendString("m");
						   _delay_us(100);
						   
						   sprintf(buffer,"%d",my_angle2);
						   USART_SendString(buffer);
						   USART_SendString("n");
						   _delay_us(100);*/
						   
						  /* dtostrf(ang2, 3, 2, float_ );
						   sprintf(buffer,"mp2 angle = %s\t",float_);
						   USART_SendString(buffer);
						   
						    dtostrf(dcounter, 3, 2, float_ );
						    sprintf(buffer,"dcounter = %s\t",float_);
						    USART_SendString(buffer); 
						   
						    dtostrf(degrees, 3, 2, float_ );
						    sprintf(buffer,"degrees = %s\n",float_);
						    USART_SendString(buffer);*/

 /*if((abs(M1.pid_error)<=1) && (abs(M2.pid_error) <= 1)&&(round_error==0) &&(state==1))		//round_error is to be added
	 {
		 USART_TxChar('1');
		 //DDRC &= ~(1<<motor_3);			//if all has been okay ,stop the base motor
		 //USART_SendString("\n");
		 state=0;
	}*/
	}		
	return 0;
}


ISR(TIMER2_COMP_vect)
{
	counter += 200;
	counter_motor+=1;
	TCNT2 = 0;
}

ISR(TIMER0_COMP_vect)
{
	if(motor_counter<=20)
	{
		motor_counter ++;
	}
	
	else
	{
		M1.stepper_counter();
		M2.stepper_counter();
		M3.stepper_angle_calulator();
	}
}

//Interrupt Service Routine for INT0
ISR(INT0_vect)
{
	
	if(((bit_is_clear(PINB,PB0))==(bit_is_clear(PIND,PD2)))||((bit_is_set(PINB,PB0))==(bit_is_set(PIND,PD2))))
	{
		dcounter++;
	}
	else
	{
		dcounter--;
	}
}

ISR(USART_RXC_vect)
{
	receive = UDR;
	le = receive;
	
	if(receive == 'g')
	{
		USART_SendString("grip completed");
		receive = 0;
		grip();
	}
	
    if(receive == 'r')
	{
		USART_SendString("release completed");
		receive = 0;
		release();
	}
	
  /*	
	switch(receive)
	{
		case 'g':
			USART_SendString("grip completed");
			grip();
			break;
		
		case 'r':
			release();
			break;
		
		default:
			USART_SendString("grip not completed");
			break;
	}
	*/
	if(receive==46)
	{
		buff_[j]='\0';                                                    //copying null character to terminate the string
		converted=atoi(buff_);
		if(tem==0)
		{
			if(converted>=85)converted=85;
			q1=converted;
			tem=1;
			// 				dtostrf(q1, 3, 2, float_ );
			// 				sprintf(buffer,"1st angle is %s\n",float_);
			// 				USART_SendString(buffer);
		}

		else if(tem==1)
		{
			if(converted>=72)converted=72;
			if(converted<=-24)converted=-24;
			q2=converted;
			tem=2;
			// 				dtostrf(q2, 3, 2, float_ );
			// 				sprintf(buffer,"2nd angle--%s\n",float_);
			// 				USART_SendString(buffer);
		}
		
		else if(tem==2)
		{
			pos=converted;
			tem=0;
			state=1;
			// 						dtostrf(q3, 3, 2, float_ );
			// 						sprintf(buffer,"3rd angle--%s\n",float_);
			// 						USART_SendString(buffer);
		}
		converted=0;
		j=0;
	}
	
	if((receive<=57 && receive>=48)||(receive==45)||(receive<=122&&receive>=97))			//45 for minus
	{
		if((receive<=57 && receive>=48)||(receive==45))
		{
			buff_[j]=receive;
			j++;
		}
		/*
		if(receive<=122&&receive>=97)
		{
			switch(receive)
			{
				case 'g':
				grip();
				break;
				
				case 'r':
				release(); 
				break;
				
				default:
				break;
			}
		}
		*/
		
	}
	receive=0;
}

void clean_the_buffer(void)
{
	for(int i=0;i<100;i++)
	{
		buff_[i] = '\0';
	}
}