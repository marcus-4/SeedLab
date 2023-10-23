/*
  Keeps constants, pin definitions, and controller parameters
*/

// Minimum interrupt time for the encoders (in microseconds)
const unsigned long minInterruptTime = 10;
// The desired update period for the control loop (in milliseconds)
const unsigned long desired_Ts = 25;
const float pi = 3.1415;
const float C = 0.15*pi; // Circumfrence of the wheels we are using, in Meters
const float r = 0.075; // meters
const float d = 0.214; // In Meters

#define BOARD_ENB_PIN 4
#define BOARD_SF_PIN 12

#define MOTOR_LEFT_ENCA 2
#define MOTOR_LEFT_ENCB 5
#define MOTOR_LEFT_DIR 7 // M1DIR fixed to pin 7
#define MOTOR_LEFT_SPEED 9 // M1PWM fixed to pin 9

#define MOTOR_RIGHT_ENCA 3
#define MOTOR_RIGHT_ENCB 6
#define MOTOR_RIGHT_DIR 8 // M2DIR fixed to pin 8
#define MOTOR_RIGHT_SPEED 10 // M2PWM fixed to pin 10

// Control system parameters, as found in Simulink
#define KP 2.6
#define KD 0.4
#define KI 0.9
#define KDP 4
#define BATTERY_VOLTAGE 7.9
#define VOLT_CAP 2
#define FF_L 1.0
#define FF_R 1.0
