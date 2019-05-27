#ifndef IMU_H
#define IMU_H
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Arduino.h>
#include "EEPROM.h"

// Constant for complementary filter
#define GAMMA 0.99

//Constants for the kalman filter of the angular velocities
#define Q_VAL 5e-4
#define R_VAL 5e-6

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define UPRIGHT_THRESHHOLD 80*PI/180 //Shut of all motors if threshold exceeded.

#define EEPROM_BASE_ADDRESS 0
/*
 *	For comments about what the functions does, see the source file!
 */
class IMU {
public:  
	void initiate();
	void update_state(float,double *,double *,double *,double *);

private:
  double acc2roll(float,float,float);
  double acc2pitch(float,float);
  double complementaryUpdate(double,float,double);
  float deg2rad(float deg);
  float rad2deg(float rad);
  void estimate_orientation(unsigned long);

  void kalman_prediction(float Ts,double* omega_hat,double* domega_hat,double P[2][2]);
  void kalman_update(double* omega_hat,double* domega_hat,double P[2][2],double gyro_meas); 

  int eeprom_address = 0;
  
  //Current sample time 
  float compl_Ts;
	
  //State variables
  double roll;
  double pitch;
  double roll_vel;
  double pitch_vel;
  double roll_acc;
  double pitch_acc;

 /* float pitch_lambda = 0.3;
  float roll_lambda = 0.3;
  float pitch_w = 0;
  float roll_w = 0;
*/
  //Measured angular velocities of the roll and pitch
  double roll_meas;
  double pitch_meas;

  //Current covariance of roll and pitch
  double P_roll[2][2];
  double P_pitch[2][2];

  //float gx_offset = 1.313000e+02;
  //float gy_offset = -1.678000e+02;
  //float gz_offset = -5.520000e+01;
  //float ax_offset = -3.449000e+02;
  //float ay_offset = 2.056000e+02;
  //float az_offset = -3.359000e+02;

  float gx_offset = 133.11;
  float gy_offset = -175.216;
  float gz_offset = -403.828;
  float ax_offset = -255.3;
  float ay_offset = -116.43;
  float az_offset = -199.742;

  //Accelerometer vars
  float ax;
  float ay;
  float az;

  //Gyro vars
  float gx;
  float gy;
  float gz;

  float int2rps;

  //IMU object
	LSM9DS1 imu;
};


#endif //IMU_H
