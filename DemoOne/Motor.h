#include "RobotParams.h"

enum MotorDirection {
  FORWARD,
  BACKWARD
};

/*
  A wrapper for the PID controller of an individual motor in our system. Our final implementation will create two object instances
  of this class, one for each physical motor. Each motor has a controller state depending on if it is trying to control
  position or velocity.
*/
class Motor {
  private:
    // Pins
    uint8_t dirPin;
    uint8_t speedPin;
    uint8_t encAPin;
    uint8_t encBPin;

    // Tracking Variables
    float integralError; // Tracks the ongoing integral error between updates
    float lastPos; // Keeps the position during the last update so velocity can be approximated
    float desiredPos = 0;
    float errorTolerance = 0.05; // Expressed as a percentage
    bool positionReached = false;


  public:
    volatile long encoderTurns = 0;
    volatile unsigned long lastIntTime;
    bool isFlipped = false;

  // Our constructor for the Motor class. Takes all 4 needed pins as the only input
  Motor(uint8_t _dirPin, uint8_t _speedPin, uint8_t _encAPin, uint8_t _encBPin){
    dirPin = _dirPin;
    speedPin = _speedPin;
    encAPin = _encAPin;
    encBPin = _encBPin;
  }

  // Must be called in start() instead of the constructor
  void init(){
    // Set each pin as INPUT or OUTPUT, as appropriate
    pinMode(dirPin, OUTPUT);
    pinMode(speedPin, OUTPUT);
    pinMode(encAPin, INPUT);
    pinMode(encBPin, INPUT);

    setDirection(FORWARD);
  }

  void setVoltage(float desiredVolt){
    // We must manually change direction, as the PWM is an absolute valued signal
    if(desiredVolt > 0){
      setDirection(FORWARD);
    } else {
      setDirection(BACKWARD);
    }
    if(desiredVolt > BATTERY_VOLTAGE) desiredVolt = BATTERY_VOLTAGE;
    if(desiredVolt < -BATTERY_VOLTAGE) desiredVolt = -BATTERY_VOLTAGE;
    // Convert the voltage to a PWM signal and send it to the motor
    int setPWM = round(255.0f * (abs(desiredVolt) / (float)BATTERY_VOLTAGE));
    analogWrite(speedPin, setPWM);
  }

  /*
    Position is in Radians! Positional control in meters will be taken care of elsewhere in the code. Integral error is reset to zero each time the target is
    updated to keep the number from increasing or decreasing wildly, and so there is a more consistent response as the control loop is run for a long period of time
  */
  void setDesiredPosition(float newPos){
    if(newPos == desiredPos){
      // No change is needed
      return;
    } else {
      // Do everything needed to start moving to a new position
      positionReached = false;
      integralError = 0;
      desiredPos = newPos;
    }
  }

  // Returns the Wheel's current position in Radians, with 0 radians being the position at the time the program started.
  float getPosInRadians(){
    return (2*pi*(float)encoderTurns / 3200.0f);
  }

  // Getter function for the destinationReached boolean
  bool isPosReached(){
    return positionReached;
  }

  void flip(){
    isFlipped = true;
  }

  private:
  /*
    Forward direction is defined in the code as a digital write HIGH on the dirPin. If isFlipped is true
    a digitalWrite LOW will be performed instead.
  */
  void setDirection(MotorDirection dir){
    if(!isFlipped){
      if(dir == FORWARD) digitalWrite(dirPin, HIGH);
      if(dir == BACKWARD) digitalWrite(dirPin, LOW);
    } else {
      if(dir == FORWARD) digitalWrite(dirPin, LOW);
      if(dir == BACKWARD) digitalWrite(dirPin, HIGH);
    }
  }

};