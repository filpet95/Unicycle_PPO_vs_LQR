// Main program definitions, global variables, function declarations and objects:
#define STATE_UPDATE_WAIT_TIME 5 //Time to wait after control task has grabbed the last state, (ms)
#define CONTROL_TS 10 //Ts for control (ms)
#define VOLTAGE_READING_TS 100

#define EMERGENCY_SHUTOFF_THRESHOLD 30*PI/180 //Shut of all motors if threshold exceeded.
//---------PINOUT ------------------------------------------------------------
#define WHEEL_PWM_PIN 13 // A12
#define DISK_PWM_PIN 12 // A11
#define WHEEL_DIR_PIN 27 //A10
#define DISK_DIR_PIN 33 //A9
#define WHEEL_A_PIN 26 //A0
#define DISK_A_PIN 15 //A8
#define WHEEL_B_PIN 25 //AA7
#define DISK_B_PIN 32 //A1

#define INPUTVOLTAGE_PIN A2 //Used to measure voltage of LIPO battery. 

#define CONTROL_ALG_PIN A4
#define ENABLE_CONTROL_PIN A3


//----Function declarations----------------------------------------------------
void Task_State_Update( void *pvParameters );
void Task_Control_Algorithm( void *pvParameters );
void Task_UI( void *pvParameters );

//----Global variables---------------------------------------------------------
double state [6]; //Global state of uni.

float debug_var = 0; // Used for different debugging purposes

float last_input = 0; // Last input if needed. 

const float wheel_counts2rad = 2*PI/(32*26.9); //Conversion of wheel motor interrupt counts to radians
const float disk_counts2rad = 2*PI/(24*19.203);//Conversion of disk motor interrupt counts to radians

bool enable_control = false;

float batt_voltage;

//Used if one would like to apply open loop input. 
//double inp_v [301] = {-12.000000 ,-12.000000 ,-8.426456 ,-3.745304 ,0.000000 ,1.658846 ,3.632510 ,5.269719 ,6.753147 ,7.703603 ,8.066847 ,8.409920 ,8.666206 ,8.869620 ,8.976672 ,9.022024 ,8.471128 ,8.215125 ,7.354349 ,7.072062 ,7.526591 ,6.979358 ,7.128122 ,6.720193 ,5.959360 ,5.765160 ,5.943599 ,5.698954 ,5.405408 ,5.103874 ,5.329390 ,5.262727 ,5.384263 ,4.583920 ,3.972062 ,4.272048 ,3.924766 ,4.225597 ,3.908019 ,3.673984 ,3.132671 ,2.804310 ,2.151347 ,2.071205 ,1.685682 ,1.920489 ,2.403947 ,2.606395 ,2.573662 ,2.370469 ,1.816243 ,1.321236 ,1.333316 ,1.569053 ,1.107310 ,0.000000 ,1.347205 ,1.097483 ,0.000000 ,0.000000 ,1.444306 ,1.508641 ,1.500779 ,1.328383 ,1.264428 ,0.000000 ,1.548229 ,1.546303 ,1.540947 ,1.692212 ,1.578992 ,1.705831 ,1.429141 ,1.750374 ,1.925878 ,1.427123 ,1.252923 ,1.320996 ,0.000000 ,1.275392 ,1.243368 ,1.410694 ,1.060644 ,0.000000 ,0.000000 ,1.311170 ,1.001795 ,1.293501 ,1.197805 ,1.144630 ,1.114560 ,0.000000 ,0.000000 ,0.000000 ,1.062391 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,1.513993 ,1.241637 ,0.000000 ,1.227408 ,0.000000 ,0.000000 ,1.060110 ,0.000000 ,0.000000 ,1.069593 ,0.000000 ,1.052564 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.092379 ,0.000000 ,0.000000 ,0.000000 ,-1.170909 ,-1.038189 ,0.000000 ,0.000000 ,0.000000 ,-1.280902 ,0.000000 ,-1.205680 ,-1.154046 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.429284 ,-1.605374 ,-1.271047 ,0.000000 ,0.000000 ,0.000000 ,-1.018776 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.062033 ,0.000000 ,0.000000 ,0.000000 ,-1.017133 ,0.000000 ,-1.188449 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,1.011204 ,0.000000 ,1.280093 ,0.000000 ,0.000000 ,1.161259 ,1.370629 ,1.003487 ,0.000000 ,1.101773 ,0.000000 ,1.051590 ,1.153555 ,0.000000 ,0.000000 ,1.260936 ,0.000000 ,0.000000 ,0.000000 ,1.225473 ,0.000000 ,0.000000 ,0.000000 ,1.191205 ,0.000000 ,1.117681 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.317292 ,-1.347169 ,-1.024487 ,-1.121590 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.335634 ,0.000000 ,-1.186599 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,1.022814 ,1.071777 ,0.000000 ,1.363814 ,1.203944 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,1.119644 ,0.000000 ,1.187856 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.119010 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.171467 ,0.000000 ,0.000000 ,-1.379175 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,0.000000 ,-1.195257 ,0.000000};


//Semaphores:
SemaphoreHandle_t xMutex_state = xSemaphoreCreateMutex(); //not used
SemaphoreHandle_t xBinary_new_state_ready = xSemaphoreCreateBinary(); //Tells the control algorithm a new state is ready
SemaphoreHandle_t xBinary_state_used = xSemaphoreCreateBinary(); //Tells the state update algorithm that the state has been used.
SemaphoreHandle_t xBinary_emergency_shutoff = xSemaphoreCreateBinary(); //Used to turn off the system if threshold exceeded.
//SemaphoreHandle_t xBinary_enable_control = xSemaphoreCreateBinary(); //Used to ENABLE / disable control system. 


	
