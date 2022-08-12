#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    //Constructor - Plus and Minus are the Motor output / encoder the encoder inputs
    Motor(int plus, int minus, int encoder);
    //Spin the motor with a percentage value
    void rotate(int value);
    //Motor Outputs - plus is one direction and minus is the other
    int plus;
    int minus;
    //Encoder Inputs
    int encoder;
    int wheelDirection;
};

#endif
