#include "Unicycle_v39.h"
#include "IMU.h"
#include "Motor.h"
#include "LQR.h"
#include "DL.h"

// Function pointer to the control algorithm, initally LQR.
void (*alg_ptr)(double state[6],double input[2]);  

// ----Object Declarations-----------------------------------------------------
IMU imu; //IMU object
Motor wheel_motor(WHEEL_PWM_PIN,WHEEL_DIR_PIN,WHEEL_A_PIN,WHEEL_B_PIN,0,wheel_counts2rad,0.45); //Wheel object
Motor disk_motor(DISK_PWM_PIN,DISK_DIR_PIN,DISK_A_PIN,DISK_B_PIN,1,disk_counts2rad,0.45); //disk object
// ----ISR functions ----------------------------------------------------------
void IRAM_ATTR wheel_motor_ISR(){ //ISR for the wheel motor encoder
  wheel_motor.sens_isr();
}
void IRAM_ATTR disk_motor_ISR(){ //ISR for the disk motor encoder
  disk_motor.sens_isr();
}

void IRAM_ATTR control_alg_ISR(){ //ISR for the control algorithm button
  //Read to check whether the pin is high or low.
  if(digitalRead(CONTROL_ALG_PIN) == HIGH ){//If high, DL. 
    alg_ptr = &DL;
  }
  else{ //If low, LQR. 
    alg_ptr = &LQR;
  }
}

void IRAM_ATTR enable_control_ISR(){ //ISR for the enable control button
  //Read to check whether the pin is high or low.
  if(digitalRead(ENABLE_CONTROL_PIN) == HIGH ){ //If high, enable
    //xSemaphoreGive(xBinary_enable_control);
    //enable_control = true;
    first_enable = true;
  }
  else{ //If low, disable. 
    //Serial.println("disable");
    //xSemaphoreTake(xBinary_enable_control,portMAX_DELAY);
    enable_control = false;
    ESP.restart();
    //wheel_motor.apply_input(0);
    //disk_motor.apply_input(0);
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  //Runs by default on core 1. All ISR started here will run at core 1, i.e. when initiating the
  //motors, the sensor interrupt will occur on core 1! 

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  /*
  double test_st[6] = {0.1,0.1,0.1,0.1,1*PI/180,1*PI/180};
  double inp_test[2] = {0.1,0.1};
  DL(test_st,inp_test);
  Serial.print(inp_test[0]);
  Serial.print(" ");
  Serial.println(inp_test[1]);*/
  //Initiate all objects
  wheel_motor.initiate();
  disk_motor.initiate();
  imu.initiate();
  //Reset the emergency_shutoff semaphore. 
  xSemaphoreGive(xBinary_emergency_shutoff);

  //Attach the interrupts for the two motor encoders
  attachInterrupt(digitalPinToInterrupt(WHEEL_A_PIN), wheel_motor_ISR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(DISK_A_PIN), disk_motor_ISR, CHANGE);
  
  //Attach the interrupts for the buttons and initialize the button states
  pinMode(CONTROL_ALG_PIN,INPUT_PULLUP);
  pinMode(ENABLE_CONTROL_PIN,INPUT_PULLUP);
  control_alg_ISR();
  if(digitalRead(ENABLE_CONTROL_PIN) == HIGH ){ //If high, enable
    enable_control = true;
  }
  attachInterrupt(digitalPinToInterrupt(CONTROL_ALG_PIN), control_alg_ISR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENABLE_CONTROL_PIN), enable_control_ISR, CHANGE); 

  //If the uni is not in upright position, calibration has been made. 
  //Wait for the uni to get upright before proceeding. 
  //Just to be completely sure, enable emergency shutoff. 
  xSemaphoreTake(xBinary_emergency_shutoff,portMAX_DELAY);
  wheel_motor.apply_input(0);
  disk_motor.apply_input(0);
  //Create local tmp variables for updating imu state. 
  double tmp1,tmp2,tmp3,tmp4;
  unsigned long time_var =  micros();
  //wait for the uni to become upright. 
  
  Serial.print("waiting for uni to stand ... ");
  int upright = -1;
  while(upright == -1){
    if(micros()-time_var > CONTROL_TS*1e3){
      time_var = micros();
      imu.update_state(CONTROL_TS*1e-3,&tmp1,&tmp2,&tmp3,&tmp4);
      if(fabs(tmp1) < EMERGENCY_SHUTOFF_THRESHOLD && fabs(tmp2) < EMERGENCY_SHUTOFF_THRESHOLD){
        upright = 1;
      }
    }
  }
  Serial.println("done!");
  xSemaphoreGive(xBinary_emergency_shutoff);
  

  // Create a task for the state updates, should run on core 1 with the highest priority. 
  xTaskCreatePinnedToCore(
    Task_State_Update
    ,  "Task_State_Update"   //Arbitrary name
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); // core

  //Create a task for the Control updates, should run on core 0 with highest priority.
  xTaskCreatePinnedToCore(
    Task_Control_Algorithm
    ,  "Task_Control_Algorithm"
    ,  4096  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  0); // core

  // Create a task for the User interface, should be run on core 1 with low priority
  xTaskCreatePinnedToCore(
    Task_Voltage_Reading
    ,  "Task_Voltage_Reading"   //Arbitrary name
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); // core
    Serial.println("init done");
}

void loop()
{
  //This can be seen as the idle task, is run on core 1 with priority 1.
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task_State_Update(void *pvParameters)  // Sensors update task
{
  //SETUP
  //std::vector<double> state_local;
  //Define everything in the state: 
  double phi;
  double theta;
  double dphi;
  double dtheta;
  double dalpha_w;
  double dalpha_d;

  int emergency;

  double curr_Ts; //Current sample time.
  unsigned long time_var = micros(); //used for deriving current sample time.

  //Amount of time of the state update Task should wait after the prev. state has been used:
  const TickType_t xDelay = STATE_UPDATE_WAIT_TIME / portTICK_PERIOD_MS;
  
  //In the initial state, we are ready to assign value to the new state. 
  xSemaphoreGive(xBinary_state_used); 
  
  /*
  Serial.print("State Update Task is running at core ");
  Serial.println(xPortGetCoreID());*/
  
  for (;;) 
  {  
    //Wait until control algorithm has used the last state: 
    xSemaphoreTake(xBinary_state_used,portMAX_DELAY);

    //Wait sufficient amount of time so the task doesn't block the control task, 
    //but the updated state will be relatively new: 
    vTaskDelay(xDelay);

    //Derive the current sample time: 
    curr_Ts = (float(micros()-time_var))/1e6;
    time_var = micros();
    
    //Get the motor state: 
    dalpha_w = wheel_motor.get_state(curr_Ts);
    dalpha_d = disk_motor.get_state(curr_Ts);

	  //Update the IMU state and save state to the variables:
    imu.update_state(curr_Ts, &phi, &theta, &dphi, &dtheta);
    //Serial.println(180*theta/PI);
    /*
    Serial.print(180*phi/PI);
    Serial.print(" ");
    Serial.print(180*theta/PI);
    Serial.println(" ");*/
    //Uncomment for graph of IMU states
    /*
    Serial.print(180*phi/PI);
    Serial.print(" ");
    Serial.print(180*dphi/PI);
    Serial.print(" ");
    Serial.print(180*dtheta/PI);
    Serial.print(" ");
    Serial.println(180*theta/PI);
    */
    //Uncomment for graph of Motor states
    /*
    Serial.print(180*dalpha_w/PI);
    Serial.print(" ");
    Serial.println(180*dalpha_d/PI);
    */

    //Check how far ahead of control algorithm we are: 
    /*
    Serial.print(micros()/1e3);
    Serial.print(" ");
    */
    
    //Update the state (globally)
	  state[0] = dalpha_w;
    state[1] = dalpha_d;
    state[2] = dphi;
    state[3] = dtheta;
    state[4] = phi;
    state[5] = theta;
    
    //If the angle exceed a threshold, disable the motors. 
    if(fabs(phi) > EMERGENCY_SHUTOFF_THRESHOLD || fabs(theta) > EMERGENCY_SHUTOFF_THRESHOLD){
      xSemaphoreTake(xBinary_emergency_shutoff,portMAX_DELAY);
      wheel_motor.apply_input(0);
      disk_motor.apply_input(0);
      Serial.println("Maximum allowed angle exceeded. Please reset.");
    }
    //Let the control algorithm know that there is a new state to fetch.
    xSemaphoreGive(xBinary_new_state_ready);
    
    
  }
}

void Task_Control_Algorithm(void *pvParameters)  // Control task
{
  //SETUP
  //Local version of state 
  double state_local[6];

  //Variables for current sample time:
  unsigned long time_init;
  double curr_control_Ts;

  //Variable for control input: 
  double input [2];
  int ind = 0;
  int size_inp = 301;

  //Variables for telemetry
  char phi_text[30];
	char theta_text[30];
  char dphi_text[30];
  char dtheta_text[30];
  char dalpha_w_text[30];
  char dalpha_d_text[30];
  char control_start_text[30];
  char u_w_text[30];
  char u_d_text[30];

  //sample time for control algorithm:
  const TickType_t xFrequency = CONTROL_TS / portTICK_PERIOD_MS;

  /*
  Serial.print("Control Task is running at core ");
  Serial.println(xPortGetCoreID());*/
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  for (;;)
  {
    //Wait until next sample time. 
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Take the semaphore to read the state (if ever blocked, decrease STATE_UPDATE_WAIT_TIME)
    xSemaphoreTake(xBinary_new_state_ready,portMAX_DELAY);
    for(int i = 0; i<6; i++){
      state_local[i] = state[i];
    }
    
    //Let the state update task know the recent state is used. 
    xSemaphoreGive(xBinary_state_used);

    //Calculate the update for the control system, given the current state:
    curr_control_Ts = (float(micros()-time_init))/1e6;
    time_init = micros();
    (*alg_ptr)(state_local,input);
    
    //Uncomment to check how old state values we have: 
    //Serial.println(time_init/1e3);
    if(first_enable == true && fabs(state_local[2]) > 3*PI/180){
      first_enable = false;
      enable_control = true;
    }

    //Give the input to the motors given the emergency shutoff and enable control is inactive.
    xSemaphoreTake(xBinary_emergency_shutoff,portMAX_DELAY);
    if(enable_control){
      control_start = 1;
      wheel_motor.apply_input(input[0]);
      disk_motor.apply_input(input[1]);
      //Serial.println(input[1]);
    }
    else{
      wheel_motor.apply_input(0);
      disk_motor.apply_input(0);
    }
    
    /*
    Serial.print(input[0]);
    Serial.print(" ");
    Serial.println(input[1]);*/
    
    xSemaphoreGive(xBinary_emergency_shutoff);
    
    
    dtostrf(input[0], 10, 10, u_w_text);
    dtostrf(input[1], 10, 10, u_d_text);
    dtostrf(state_local[0], 10, 10, dalpha_w_text);
    dtostrf(state_local[1], 10, 10, dalpha_d_text);
    dtostrf(state_local[2], 10, 10, dphi_text);
    dtostrf(state_local[3], 10, 10, dtheta_text);
    dtostrf(state_local[4], 10, 10, phi_text);
    dtostrf(state_local[5], 10, 10, theta_text);
    dtostrf(control_start, 10, 10, control_start_text);
    //Serial.println(uxTaskGetStackHighWaterMark(NULL));
    

    char text[249];
    snprintf(text, 249, "%s,%s,%s,%s,%s,%s,%s,%s,%s", u_w_text, u_d_text, dalpha_w_text, dalpha_d_text, dphi_text, dtheta_text, phi_text, theta_text, control_start_text);
    Serial.println(text);
    //Serial.println(input[0]);
    //last_input = input[1];
  }
}
void Task_Voltage_Reading(void *pvParameters)  // User Interface task
{
  //SETUP
  //Variables for measuring the voltage of LIPO battery: 
  const float lambda = 0.97;
  //const float voltage_convert_constant = 0.16733870967741935483870967741935;
  //const float voltage_convert_constant = 0.1402;
  const float voltage_convert_constant = 0.1145;
  float reading = 4095;
  //float batt_voltage;
  float weight = 1;
  pinMode(INPUTVOLTAGE_PIN,INPUT);

  //sample time for volt read algorithm:
  const TickType_t xFrequency = VOLTAGE_READING_TS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  
  for (;;) 
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    weight = lambda*weight + 1;
    //Derive the moving average of the voltage:
    reading = (1-1/weight)*reading+(1/weight)*float(analogRead(INPUTVOLTAGE_PIN));
    //Serial.println(analogRead(INPUTVOLTAGE_PIN));
    batt_voltage = reading*3.6/(4095*voltage_convert_constant);
    disk_motor.volt_conversion = 255/15.8; //Derived conversion between volts and PWM
    wheel_motor.volt_conversion = 255/15.8; //Derived conversion between volts and PWM
    /*if(batt_voltage > 14.4 && batt_voltage < 17.2){
      disk_motor.volt_conversion = 255/batt_voltage; //Derived conversion between volts and PWM
      wheel_motor.volt_conversion = 255/batt_voltage; //Derived conversion between volts and PWM
    }else{
      disk_motor.volt_conversion = 255/15.8; //Derived conversion between volts and PWM
      wheel_motor.volt_conversion = 255/batt_voltage; //Derived conversion between volts and PWM
    }*/
    //Serial.println(batt_voltage);
    if( batt_voltage < 14){
      //Serial.println("charge battery!");
    }
  }
}
