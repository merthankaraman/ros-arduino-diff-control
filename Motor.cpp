#include "Arduino.h"
#include "Motor.h"

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

Motor::Motor(int plus, int minus, int encoder) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(encoder,INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::encoder = encoder;
}

void Motor::rotate(int value) {
  this->wheelDirection = sgn(value);
  if(value>0){
    int out = map(value, 0, 100, 0, 255);//pow(2,13)-1);
    analogWrite(plus,out);
    analogWrite(minus,0);
  }
  else{
    int out = map(value, 0, -100, 0, 255);//pow(2,13)-1);
    analogWrite(plus,0);
    analogWrite(minus,out);
  }
}
