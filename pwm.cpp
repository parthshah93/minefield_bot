#include "Arduino.h"

const char leftWheel = 3;
const char rightWheel = 4;

class PWM{
  public:
    void init(){
      analogWriteResolution(10);
      analogWriteFrequency(leftWheel, 50);
      analogWriteFrequency(rightWheel, 50);
      analogWrite(leftWheel,76);
      analogWrite(rightWheel,76);
    }
    
    void forward(){
      analogWrite(leftWheel,67);
      analogWrite(rightWheel,67);
      delay(500);
    }
    
    void backward(){
      analogWrite(3,90);
      analogWrite(4,90);
      delay(300);
    }
    
    void right(){
      analogWrite(3,67);
      analogWrite(4,86);
      delay(300);
    }
    
    void left(){
      analogWrite(3,86);
      analogWrite(4,67);
      delay(300);
    }
    
    void stay(){
      analogWrite(3,76);
      analogWrite(4,76);
    }
};
