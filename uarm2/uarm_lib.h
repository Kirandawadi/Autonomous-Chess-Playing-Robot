#ifndef uarm_lib
#define uarm_lib

#include <avr/io.h>

#define  motor_1 PINC7
#define  motor_1_dir PINC6
#define  motor_2 PINC5
#define  motor_2_dir PINC4
#define  motor_3 PINC3
#define  motor_3_dir PINC2

void motor_anti_clockwise(void);
void motor_clockwise(void);
void set_position_degrees(double angle);
void motor_halt(void);





#endif 