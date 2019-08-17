#include "Arduino.h"
#include "motorClass.h"
#include "Encoder.h"

//Constructor
motorClass::motorClass(int pwmPin,int dirPin, int encPin,float gearRatio, float encCntsRev){
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  _encPin = encPin;
  _gearRatio = gearRatio;
  _encCntsRev = encCntsRev;
  
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_encPin, OUTPUT);

  initEncoder(_encPin);
  clearEncoderCount(_encPin);
}


/////////////////////////////////Set Desired Value Functions\\\\\\\\\\\\\\\\\\\\\\\\\

void motorClass::setMotorPos(float pos){
  desiredMotorPos = pos;
}


void motorClass::setMotorVel(float vel){
  desiredMotorVel = vel;
}


/////////////////////////////////ENCODER\\\\\\\\\\\\\\\\\\\\\\\\\

void motorClass::clearEncoder(void){
  clearEncoderCount(_encPin);
}
  

signed long motorClass::readEnc(void){
  encodercount = -1*readEncoder(_encPin);
  return encodercount;
}


////////////////////////////////Helper Functions\\\\\\\\\\\\\\\\\\\\\\\\

void motorClass::storeOldVals(void){
  // time
  calc_t();
  prevTime = currentTime;

  // encoder
  encodercountPrev = encodercount;
  errorPosPrev = errorPos;
  errorVelPrev = errorVel;
}


void motorClass::calc_t(){
  currentTime = micros();
  dt = (currentTime - prevTime)/1000000.0;
}


void motorClass::stopMotor(void){
  analogWrite(_pwmPin,0);
}


void motorClass::logValues(void) {
  Serial.print(" MP: "+(String)MotorPos);
  Serial.print(" ErrorPos: "+ (String)errorPos);
  Serial.print(" dt: "+ (String)(1000*dt));
  Serial.print(" integrated Error: "+ (String)(1000*integratedPosError));
  Serial.println(" MotorCommand: " + (String)currentCommandp);

}


////////////position controller functions!\\\\\\\\\\\\\\\\\

float motorClass::motor_position_calc(void){
  calc_t();
  encodercount = readEnc();
  MotorPos = (1/_gearRatio)*(1/_encCntsRev)*(encodercount); //revolutions of the output shaft
  storeOldVals();
  return MotorPos; 
}

float motorClass::pos_proportional_control(void){
  errorPos = desiredMotorPos - MotorPos;
  pCommandp = Kpp * errorPos;
  return pCommandp;
}


float motorClass::pos_derivative_control(void){
  calc_t();
  dCommandp = Kdp * (errorPos - errorPosPrev) / dt;
  return dCommandp;
}


float motorClass::pos_integral_control(void){
  calc_t();
  integratedPosError = integratedPosError + errorPos;
  iCommandp = Kip*integratedPosError;
  return iCommandp;
}


int motorClass::pos_closedLoopController(void){
  motor_position_calc();
  errorPos = desiredMotorPos - MotorPos;
  if (abs(errorPos)<0.005){
    errorPos = 0;
    }
    
  currentCommandp = pos_proportional_control() + pos_derivative_control() + pos_integral_control();

  currentCommandp = constrain(map(currentCommandp, -1000, 1000, -255, 255),-255,255);

  if (currentCommandp < -0.001) {
    digitalWrite(_dirPin, LOW);
  }
  else if(currentCommandp > 0.001){
    digitalWrite(_dirPin, HIGH);
  }
  analogWrite(_pwmPin, abs(currentCommandp));
  return currentCommandp; 
}


////////////velocity controller functions!\\\\\\\\\\\\\\\\\

//float motorClass::motor_velocity_calc(void){
//  calc_t();
//  encodercount = readEnc();
//  MotorVel = (1/_gearRatio)*(1/_encCntsRev)*(encodercount-encodercountPrev)/dt; //motor shaft revolutions per second
//  storeOldVals();
//  return MotorVel; 
//}
//
//
//float motorClass::vel_proportional_control(void){
//  errorVel = desiredMotorVel - MotorVel;
//  pCommandv = Kpv * errorVel;
//  return pCommandv;
//}
//
//
//float motorClass::vel_derivative_control(void){
//  calc_t();
//  dCommandv = Kdv * (errorVel - errorVelPrev) / dt;
//  return dCommandv;
//}
//
//
//float motorClass::vel_integral_control(void){
//  calc_t();
//  errorVel = desiredMotorVel - MotorVel;
//  integratedVelError = integratedVelError + errorVel;
//  
//  iCommandv = Kiv*integratedVelError;
//  return iCommandv;
//}
//
//
//int motorClass::vel_closedLoopController(void){
//  motor_velocity_calc();
//  motor_position_calc();
//
//  float Pv=vel_proportional_control();
//  float Dv=vel_derivative_control();
//  float Iv=vel_integral_control();
//
//  currentCommandv = Pv+Iv+Dv;
//
//  currentCommandv = constrain(map(currentCommandv, -1000, 1000, -255, 255),-255,255);
//  if (currentCommandv < -0.001) {
//    digitalWrite(_dirPin, HIGH);
//  }
//  else if(currentCommandv > 0.001){
//    digitalWrite(_dirPin, LOW);
//  }
//  analogWrite(_pwmPin,abs(currentCommandv));
//  return currentCommandv; 
//}
