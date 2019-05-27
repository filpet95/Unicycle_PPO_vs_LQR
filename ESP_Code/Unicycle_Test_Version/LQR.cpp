#include "LQR.h"
double volt_wheel;
double volt_disk;
double volt_wheel_old;
double volt_disk_old;
const float lambda = 0.9; //Moving average constant
double weight;
void LQR(double state [6],double input [2]){
	volt_wheel = -(K_11*state[0] + K_12*state[1] + K_13*state[2] + K_14*state[3] + K_15*state[4] + K_16*state[5] + K_17*volt_wheel_old);
  //volt_wheel = -(K_13*state[2] + K_14*state[3] + K_15*state[4] + K_16*state[5]);

	volt_disk = -(K_21*state[0] + K_22*state[1] + K_23*state[2] + K_24*state[3] + K_25*state[4] + K_26*state[5] + K_27*volt_wheel_old);
  /*Serial.print(K_11*state[0]);
  Serial.print(" ");
  Serial.print(K_14*state[3]);
  Serial.print(" ");
  Serial.println(K_16*state[5]);*/

  weight = lambda*weight + 1;
  //Derive the angular velocity using a moving average:
  volt_wheel = (1-1/weight)*volt_wheel_old+(1/weight)*volt_wheel;
  volt_wheel_old = volt_wheel;
  input[0] = volt_wheel;
  input[1] = volt_disk;
  /*
  Serial.print(K_22*state[1]);
  Serial.print(" ");
  Serial.print(K_23*state[2]);
  Serial.print(" ");
  Serial.println(K_25*state[4]);*/

}
