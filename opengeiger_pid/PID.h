#include "Arduino.h"
 
class PID {
  public:
    PID(float p, float i, float d) {
      kP = p;
      kI = i;
      kD = d;
 
      iTerm = 0;
    }
 
    void reset() {
      iTerm = 0;
    }
 
    float compute(float ref, float input, float dt) {
      float error = ref - input;
 
      pTerm = error;
      iTerm += error * dt;
      dTerm = (input - prevInput) / dt;
 
      prevInput = input;
 
      return kP*pTerm + kI*iTerm - kD*dTerm;
    }
 
    void setCoeffs(float p, float i, float d) {
      kP = p;
      kI = i;
      kD = d;
 
      iTerm = 0;
    }
 
  private:
    float prevInput;
    float pTerm, iTerm, dTerm;
    float kP, kI, kD;
};

