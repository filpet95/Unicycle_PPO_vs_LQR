#include "IMU.h"


/*
 ***** Public methods *****
 */

 /*
 * Function that initializes the IMU. Returns -1 if uni is not in upright position.
 */
void IMU::initiate()
{
	imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  //------Gyro settings------------------------------
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; //Maximal output from gyro
  int2rps = 0.00875*PI/180; //Conversion from integer to radians per second. Either 0.00875,0.0175 or 0.07 for 245 , 500, 2000 respectively. 

  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6 [Hz]
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 6;
  
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  //Low value -> low cutoff
  imu.settings.gyro.bandwidth = 0;
  imu.settings.gyro.lowPowerEnable = false;

  //------Accelerometer settings--------------------
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 2;
  
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 6;
  
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = -1;

  //High resolution, basically no1 knows what it is
  imu.settings.accel.highResEnable = false;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initiate EEPROM.");
    while (1);
  }
  
  //Check what orientation the uni starts in:
  estimate_orientation(3e6);

  
  eeprom_address = EEPROM_BASE_ADDRESS;
  gx_offset = EEPROM.readFloat(eeprom_address);
  eeprom_address += sizeof(float);
  gy_offset = EEPROM.readFloat(eeprom_address);
  eeprom_address += sizeof(float);
  gz_offset = EEPROM.readFloat(eeprom_address);
  eeprom_address += sizeof(float);
  ax_offset = EEPROM.readFloat(eeprom_address);
  eeprom_address += sizeof(float);
  ay_offset = EEPROM.readFloat(eeprom_address);
  eeprom_address += sizeof(float);
  az_offset = EEPROM.readFloat(eeprom_address);
  //If not upright, calibrate the gyro and wait a while for the user to pick it up. 
  if(fabs(roll) > UPRIGHT_THRESHHOLD || fabs(pitch) > UPRIGHT_THRESHHOLD){
    imu.readGyro();
    if((fabs(int2rps*imu.gx)+abs(int2rps*imu.gy)+abs(int2rps*imu.gz))<0.03){
      imu.calibrate(false);
      gx_offset = imu.gBiasRaw[0];
      gy_offset = imu.gBiasRaw[1];
      gz_offset = imu.gBiasRaw[2];
      ax_offset = imu.aBiasRaw[0];
      ay_offset = imu.aBiasRaw[1];
      az_offset = imu.aBiasRaw[2];

      eeprom_address = EEPROM_BASE_ADDRESS;
      EEPROM.writeFloat(eeprom_address,gx_offset);
      eeprom_address += sizeof(float);
      EEPROM.writeFloat(eeprom_address,gy_offset);
      eeprom_address += sizeof(float);
      EEPROM.writeFloat(eeprom_address,gz_offset);
      eeprom_address += sizeof(float);
      EEPROM.writeFloat(eeprom_address,ax_offset);
      eeprom_address += sizeof(float);
      EEPROM.writeFloat(eeprom_address,ay_offset);
      eeprom_address += sizeof(float);
      EEPROM.writeFloat(eeprom_address,az_offset);
      EEPROM.commit();
      
      /*Serial.print("float gx_offset = ");
      Serial.print(gx_offset);
      Serial.println(";");
      Serial.print("float gy_offset = ");
      Serial.print(gy_offset);
      Serial.println(";");
      Serial.print("float gz_offset = ");
      Serial.print(gz_offset);
      Serial.println(";");
      Serial.print("float ax_offset = ");
      Serial.print(ax_offset);
      Serial.println(";");
      Serial.print("float ay_offset = ");
      Serial.print(ay_offset);
      Serial.println(";");
      Serial.print("float az_offset = ");
      Serial.print(az_offset);
      Serial.println(";");*/
      Serial.println("calibration of IMU done");
    }
  }

  //Initiation done
	Serial.print("IMU initiated at core ");
	Serial.println(xPortGetCoreID());

}

/*
 * Function that updates the IMU states and assign them to the input pointers. 
 */
void IMU::update_state(float Ts, double *roll_ptr, double *pitch_ptr, double *roll_vel_ptr, double *pitch_vel_ptr)
{
  compl_Ts = Ts;
  //update the gyro and acc data:
  imu.readAccel();
  imu.readGyro();
  /*
  Serial.print(imu.ax);
  Serial.print(" ");
  Serial.print(imu.ay);
  Serial.print(" ");
  Serial.print(imu.az);
  Serial.print(" ");
  Serial.print(imu.gx);
  Serial.print(" ");
  Serial.print(imu.gy);
  Serial.print(" ");
  Serial.println(imu.gz);*/

  //Remove the offset
  ax = imu.ax - ax_offset;
  ay = imu.ay - ay_offset;
  az = imu.az - az_offset;
  gx = imu.gx - gx_offset;
  gy = imu.gy - gy_offset;
  gz = imu.gz - gz_offset;
  
  //Derive the roll and pitch from the accelerometer:
  roll_acc = -acc2roll(imu.calcAccel(ax), imu.calcAccel(ay), imu.calcAccel(az)) -0.00873;// - 0.0271;//-0.0175;
  pitch_acc = -acc2pitch(imu.calcAccel(ax), imu.calcAccel(az)) + 0.014;// 0.0297;// + 0.0175;//75  

  //Derive the roll and pitch angular velocity from the gyro:
  roll_meas = int2rps * gx * sin(pitch) - int2rps * gz * cos(pitch); //Roll is defined in the world frame, i.e. before the pitch. Readings are with the pitch, need to get it back.
  pitch_meas = -int2rps * gy;   

  //If too much force acting on the IMU, just purely integrate angle with gyro data
  if (sqrt(sq(imu.calcAccel(ax)) + sq(imu.calcAccel(ay)) + sq(imu.calcAccel(az))) > 1.1)
  {
    roll = roll + compl_Ts * (roll_meas);
    pitch = pitch + compl_Ts * (pitch_meas);
  }
  else
  { //Else, update the angle using complementary filter.
    roll = complementaryUpdate(roll, (roll_meas), roll_acc);
    pitch = complementaryUpdate(pitch, (pitch_meas), pitch_acc);
  }
  //Do a kalman update of the angular velocity of roll and pitch:
  kalman_update(&roll_vel, &roll_acc, P_roll, roll_meas);
  kalman_update(&pitch_vel, &pitch_acc, P_pitch, pitch_meas);

  //Assign the new state values to the input pointers:
  *roll_ptr = roll;
  *pitch_ptr = pitch;
  *roll_vel_ptr = roll_vel;
  *pitch_vel_ptr = pitch_vel;

  //Do a Kalman prediction for use in next timestep:
  kalman_prediction(Ts, &roll_vel, &roll_acc, P_roll);
  kalman_prediction(Ts, &pitch_vel, &pitch_acc, P_pitch);


  //Uncomment to see how much stack space is used:
  //Serial.println(uxTaskGetStackHighWaterMark(NULL));
}

/*
 * Function that converts acceleration data (m/s^2) to roll angle, see report
 */
double IMU::acc2pitch(float ax, float az){
  double roll_local = atan2(-az,ax);
  return roll_local;
}

/*
 * Function  that converts acceleration data (g) to pitch angle, see report
 */
double IMU::acc2roll(float ax,float ay, float az){
  return atan2(ay,sqrt((ax*ax)+(az*az)));
}

/*
 * Function  that does the update step of the complementary filter, see report
 */
double IMU::complementaryUpdate(double theta_old,float gyromeas,double accangle){
  return (1-GAMMA)*accangle + GAMMA*(theta_old+compl_Ts*gyromeas);
}

/*
 * Function  that does the prediction step of the Kalman filter, see report. 
 */
void IMU::kalman_prediction(float Ts,double* omega_hat,double* domega_hat,double P[2][2]){
  *omega_hat = *omega_hat + Ts*(*domega_hat);
  *domega_hat = *domega_hat; 

  double P_tmp [2][2] = {
    {P[0][0],P[0][1]},
    {P[1][0],P[1][1]}
  };

  P[0][0] = P_tmp[1][1]*Ts*Ts + 2*P_tmp[0][1]*Ts + P_tmp[0][0];
  P[0][1] = P_tmp[0][1] + P_tmp[1][1]*Ts;
  P[1][0] = P[0][1];
  P[1][1] = P_tmp[1][1] + Q_VAL;
}

/*
 * Function  that does the update step of the Kalman filter, see report. 
 */
void IMU::kalman_update(double* omega_hat,double* domega_hat,double P[2][2],double gyro_meas){
  *domega_hat = *domega_hat - ((P[0][1]*(*omega_hat-gyro_meas))/(P[0][0]+R_VAL));
  *omega_hat = (P[0][0]*gyro_meas + (*omega_hat)*R_VAL) / (P[0][0]+R_VAL);
  
  double P_tmp [2][2] = {
    {P[0][0],P[0][1]},
    {P[1][0],P[1][1]}
  };

  P[0][0] = P_tmp[0][0]*R_VAL / (P_tmp[0][0] + R_VAL);
  P[0][1] = P_tmp[0][1]*R_VAL / (P_tmp[0][0] + R_VAL);
  P[1][0] = P[0][1];
  P[1][1] = P_tmp[1][1] - (P_tmp[0][1] * P_tmp[0][1] / (P_tmp[0][0] + R_VAL));
}

/*
 * Function  that converts degrees to radians. 
 */
float IMU::deg2rad(float deg){
  return PI*deg/180;
}

/*
 * Function  that converts radians to degrees. 
 */
float IMU::rad2deg(float rad){
  return 180*rad/PI;
}

/*
 * Function  that estimates the uni's orientation during duration amount of time.
 */
void IMU::estimate_orientation(unsigned long duration){
  double tmp1,tmp2,tmp3,tmp4;
  unsigned long time_init = micros();
  unsigned long time_var = time_init;
  while(micros()-time_init < duration){
    if(micros()-time_var > 1e4){
      time_var = micros();
      update_state(0.01,&tmp1,&tmp2,&tmp3,&tmp4);
    }
  }
}
