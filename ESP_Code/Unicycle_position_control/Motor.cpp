#include "Motor.h"

 
 /*
 * Constructor, Input defines which pin the motor is connected to.
 */
Motor::Motor(char pwm, char dir,char sens_A,char sens_B,char led_ch,float conversion,float mov_avg_const)
{
  pwm_pin = pwm;     
  dir_pin = dir;
  sens_A_pin = sens_A;
  sens_B_pin = sens_B;
  led_channel = led_ch;
  counts2rad = conversion;
  lambda = mov_avg_const;
}

/*
 ***** Public methods *****
 */

 /*
 * Function that initializes the Motor
 */
void Motor::initiate()
{
  // Tells driver what direction the motor should turn.
  pinMode(dir_pin,OUTPUT);
  //Tells driver what voltage the motor should get.
  pinMode(pwm_pin,OUTPUT);
  //Motor sensor pins, PIN A is attached to external interrupt. 
  pinMode(sens_A_pin,INPUT_PULLUP); //If no signal is attached, it will be high.
  pinMode(sens_B_pin,INPUT_PULLUP); //If no signal is attached, it will be high.
  //Will count the number of interrupt occured to derive the angular veloicty. 
  interrupt_counter = 0;

  //Weight for a moving average of the angular velocity. 
  weight = 0;
  
  //Setup for the PWM signal:
  ledcSetup(led_channel, pwm_freq, resolution);
  ledcAttachPin(pwm_pin, led_channel);
	
  //Initiation done
  Serial.print("Motor initiated on core ");
  Serial.println(xPortGetCoreID());
}

/*
 * Function that applies the input (volts) to the motor
 */
void Motor::apply_input(float volt_in)
{
  volt_in = saturate(volt_in);
  //Convert to PWM signal and apply it:
  ledcWrite(led_channel,volt2pwm(volt_in));
}
 
/*
 * Function that derives and returns the Motor angles
 */
float Motor::get_state(float sample_time)
{
  //Derive the angular difference since last sample (in radians)
  float anglediff = (float)(interrupt_counter)*counts2rad;
  //Reset counter to next sample ASAP
  interrupt_counter = 0;
  //Update the weight for the moving average:
  weight = lambda*weight + 1;
  //Serial.println(weight);
  //Derive the angular velocity using a moving average:
  angular_vel = (1-1/weight)*angular_vel+(1/weight)*anglediff / sample_time;
  //angular_vel = anglediff/sample_time;
  //Update the angle
  angle = angle + anglediff;
  

  return angular_vel;
}

float Motor::get_pos()
{
  return angle;
}

/*
 * Function that converts a voltage input to a pwm input with range resolution. 
 */
float Motor::volt2pwm(float volt_in){
  if (volt_in > 0){
    digitalWrite(dir_pin,LOW);
  }
  else{
    digitalWrite(dir_pin,HIGH);
  }
  return volt_conversion*fabs(volt_in);
}

/*
 * Function that saturates the voltage within the current saturation limit
 */
float Motor::saturate(float volt_in){
  if(volt_in > sat_limit){
    volt_in = sat_limit;
  }else if (volt_in < -sat_limit){
    volt_in = -sat_limit;
  }
  return volt_in;
}

/*
 * Function that will be called on at every interrupt. Given the current state of the 
 * interrupt pin and the other channel, increment or decrement the interrupt counter. 
 */
void Motor::sens_isr(){
  /*Serial.print(digitalRead(sens_A_pin));
  Serial.print(" ");
  Serial.println(digitalRead(sens_B_pin));*/
  if(digitalRead(sens_A_pin) == 1){ 
    if(digitalRead(sens_B_pin) == 0){
      interrupt_counter ++;
    }else{
      interrupt_counter --;
    }
  }
  else{ 
    if(digitalRead(sens_B_pin) == 0){
      interrupt_counter --;
    }else{
      interrupt_counter ++;
    }
  }
}
