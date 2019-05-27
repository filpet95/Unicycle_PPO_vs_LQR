#include "Arduino.h"
#include "DL.h"
//#define OBS_SPACE_8 
double volt_wheel_old_DL;
double volt_disk_old_DL;
const float lambda_DL = 0.9;
const float lambda_DL_disk = 0.45;
double weight_DL;
double weight_DL_disk;
void DL(double state[6],double input[2])
{
  double z0[64];
  double z1[64];
  int i;
  int j;
  double tmp_1;
  
  
  
#ifdef OBS_SPACE_8
  for (i = 0; i < 64; i++) {
    tmp_1 = 0.0;
    for (j = 0; j < 2; j++){
      tmp_1 += pi_fc0_w[8 * i + j] * input[j];
    }
    for (j = 0; j < 6; j++) {
      tmp_1 += pi_fc0_w[8 * i + j + 2] * state[j];
    }
  
    z0[i] = tanh(tmp_1 + pi_fc0_b[i]);
  }
#else
  for (i = 0; i < 64; i++) {
    tmp_1 = 0.0;
    for (j = 0; j < 6; j++) {
      tmp_1 += pi_fc0_w[6 * i + j] * state[j];
    }
  
    z0[i] = tanh(tmp_1 + pi_fc0_b[i]);
  }
#endif

  for (i = 0; i < 64; i++) {
    tmp_1 = 0.0;
    for (j = 0; j < 64; j++) {
      tmp_1 += pi_fc1_w[(i << 6) + j] * z0[j];
    }

    z1[i] = tanh(tmp_1 +pi_fc1_b[i]);
  }

  for (i = 0; i < 2; i++) {
    tmp_1 = 0.0;
    for (j = 0; j < 64; j++) {
      tmp_1 += pi_w[(i << 6) + j] * z1[j];
    }

    input[i] = tmp_1 + pi_b[i];
  }
  
  input[0] = input[0];
  input[1] = input[1]; 
  weight_DL = lambda_DL*weight_DL + 1;
  //weight_DL_disk = lambda_DL_disk*weight_DL_disk + 1;
  //Derive the angular velocity using a moving average:
  input[0] = (1-1/weight_DL)*volt_wheel_old_DL+(1/weight_DL)*input[0];
  //input[1] = (1-1/weight_DL_disk)*volt_disk_old_DL+(1/weight_DL_disk)*input[1];
  volt_wheel_old_DL = input[0];
  //volt_disk_old_DL = input[1];
  
}
