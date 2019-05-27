#include "Arduino.h"


void LQR(double state[6],double input[2]);


/*
const float K_11 = -1.863460;
const float K_12 = -0.000336;
const float K_13 = -0.055186;
const float K_14 = 9.843693;
const float K_15 = -0.141884;
const float K_16 = 51.179796;
const float K_21 = 0.000355;
const float K_22 = -0.432960;
const float K_23 = -29.623740;
const float K_24 = -0.005138;
const float K_25 = -91.760492;
const float K_26 = -0.019579; //Really good with inp filter*/
/*
const float K_11 = -3.591436;
const float K_12 = 0.000015;
const float K_13 = 0.004004;
const float K_14 = 12.256988;
const float K_15 = 0.009714;
const float K_16 = 76.777100;
const float K_17 = 1.414760;
const float K_21 = -0.000563;
const float K_22 = -0.432960;
const float K_23 = -29.623758;
const float K_24 = -0.000616;
const float K_25 = -91.760538;
const float K_26 = 0.004390;
const float K_27 = 0.000507;//Good filtered version, smooth but not super robust?*/
/*
const float K_11 = -3.457414;
const float K_12 = 0.000014;
const float K_13 = 0.003693;
const float K_14 = 11.658352;
const float K_15 = 0.009010;
const float K_16 = 73.858483;
const float K_17 = 1.287244;
const float K_21 = -0.000558;
const float K_22 = -0.432960;
const float K_23 = -29.623758;
const float K_24 = -0.000666;
const float K_25 = -91.760538;
const float K_26 = 0.004153;
const float K_27 = 0.000506;//Best filtered version`? As low as possible on the wheel. Disk is still when calibrated.*/
/*
const float K_11 = -3.457333;
const float K_12 = -0.000038;
const float K_13 = 0.000700;
const float K_14 = 11.658079;
const float K_15 = -0.007272;
const float K_16 = 73.856748;
const float K_17 = 1.287205;
const float K_21 = -0.000039;
const float K_22 = -0.524018;
const float K_23 = -27.445668;
const float K_24 = -0.004520;
const float K_25 = -158.848883;
const float K_26 = -0.010158;
const float K_27 = 0.000192;;//Filtered version, new disk motor constant*/
/*
const float K_11 = -3.977786;
const float K_12 = -0.000028;
const float K_13 = 0.001555;
const float K_14 = 13.261125;
const float K_15 = -0.003758;
const float K_16 = 83.208475;
const float K_17 = 1.367976;
const float K_18 = -0.852533;
const float K_21 = 0.000030;
const float K_22 = -0.524018;
const float K_23 = -27.445668;
const float K_24 = -0.004733;
const float K_25 = -158.848884;
const float K_26 = -0.011406;
const float K_27 = 0.000181;
const float K_28 = 0.000108;*///First try position control, a bit shaky but really good!
const float K_11 = -3.624398;
const float K_12 = -0.000035;
const float K_13 = 0.000986;
const float K_14 = 12.173211;
const float K_15 = -0.006077;
const float K_16 = 76.864122;
const float K_17 = 1.313615;
const float K_18 = -0.270329;
const float K_21 = -0.000013;
const float K_22 = -0.524018;
const float K_23 = -27.445668;
const float K_24 = -0.004599;
const float K_25 = -158.848883;
const float K_26 = -0.010624;
const float K_27 = 0.000188;
const float K_28 = 0.000041;

/*const float K_11 = -3.278497;
const float K_12 = 0.000013;
const float K_13 = 0.003504;
const float K_14 = 11.240538;
const float K_15 = 0.008635;
const float K_16 = 70.216850;
const float K_17 = 1.124936;
const float K_21 = -0.000538;
const float K_22 = -0.432960;
const float K_23 = -29.623758;
const float K_24 = -0.000702;
const float K_25 = -91.760538;
const float K_26 = 0.003850;
const float K_27 = 0.000492;*/
