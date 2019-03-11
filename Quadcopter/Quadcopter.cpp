#include "Quadcopter.h"
#include "mbed.h"
#include "Servo.h" 

Quadcopter::Quadcopter(PinName FL, PinName FR, PinName BL, PinName BR){ //we have 4 motors to control
    motor[0] = new Servo (FL);  //motors are of class Servo as ESC are used in the similar manner
    motor[1] = new Servo (FR);  
    motor[2] = new Servo (BL);
    motor[3] = new Servo (BR);
    
    min_calibrate = 0.0;
    max_calibrate = 1.0;
}
 
//------------------------------Function for ESC calibration---------------------
void Quadcopter::calibrate (){        
    for (int i = 0; i < 4; i++){    //run motors for some time in min speed
        *motor[i] = max_calibrate;
    }   
    wait(6.0);                      //wait for the response from ESC modules
    for (int i = 0; i < 4; i++){
        *motor[i] = min_calibrate;  //run motors at maximum speed
    }
    wait(2.0);      //again wait for response
}
//-------------------------------------Function for Stabilising---------------
void Quadcopter::stabilise (float* speed, float* actSpeed, float rollDiff, float pitchDiff){
    actSpeed[0] = speed[0] + (rollDiff / 2) + (pitchDiff / 2);  //each motor has actual Speed and speed at which we want them to fly...
    actSpeed[1] = speed[1] - (rollDiff / 2) + (pitchDiff / 2);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
    actSpeed[2] = speed[2] + (rollDiff / 2) - (pitchDiff / 2);
    actSpeed[3] = speed[3] - (rollDiff / 2) - (pitchDiff / 2);
}
//-----------------------Function for producing thrust in Z direction --------
void Quadcopter::run (float* speed){
    //simply map values in the correct range and run PWM signals for each motor
    for (int i = 0; i < 4; i++){
        if (speed[i] < 0.0)
            *motor[i] = min_calibrate;
        else if (speed[i] > 1.0)
            *motor[i] = max_calibrate;
        else
            *motor[i] = this->map(speed[i], 0.0, 1.0, min_calibrate, max_calibrate);
    }
}
//--------------------------Function for setting calibration limits-----------
void Quadcopter::setLimits(float min, float max){
    if (min > max){             //here detect if someone tried making min to be more than max. If that is the case, then flip them together
        min_calibrate = max;
        max_calibrate = min;   
    } else {   
        min_calibrate = min;
        max_calibrate = max;
    }    
    if ((min_calibrate > 1.0) || (min_calibrate < 0.0)) //here chech if values are in correct range. If they are not, make them to be in correct range
        min_calibrate = 0.0;
    if ((max_calibrate > 1.0) || (max_calibrate < 0.0))
        max_calibrate = 1.0;
}
//-----------------------------Mapping function-----------------------------
float Quadcopter::map(float x, float in_min, float in_max, float out_min, float out_max){   //simply maps values in the given range
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//-------------------------------Functiuon for getting the lower calibration limit------------
float Quadcopter::getLowerLimit(){
    return min_calibrate;
}
//----------------------------Function for getting upper calibration limit--------------------
float Quadcopter::getUpperLimit(){
    return max_calibrate;
    
}