#ifndef Motor_H
#define Motor_H

#include <Arduino.h>
/*
 *	For comments about what the functions does, see the source file!
 */
class Motor {
public:
	Motor(char,char,char,char,char,float,float);   
	void initiate();
	void apply_input(float);
	float get_state(float);	
  float get_pos();
	void sens_isr();
  float volt_conversion = 255/14.94; //Derived conversion between volts and PWM
private:
	float volt2pwm(float);
	float saturate(float);
	float weight; //Weight for the moving average
	char pwm_pin; //the pin signal PWM is connected to.
	char dir_pin; //the pin signal DIR is connected to.
	char sens_A_pin; //the pin A is connected to
	char sens_B_pin; //the pin B is connected to
	long interrupt_counter; //Keeps track of the #interupts per cycle
	double angle; //The current angle of the motor, in radians.
	double angular_vel; //The current angular velocity of the motor, in radians/s.
  float counts2rad; //Derived transformation from number of interrupts to radians
   
	float lambda; //Moving average constant

	const int resolution = 8; //Resolution of PWM 
	char led_channel; //channel of PWM
	const int pwm_freq = 490; //Freq of PWM

	const int sat_limit = 12; //Maximum 12 V input.

  	
};


#endif //Motor_H
