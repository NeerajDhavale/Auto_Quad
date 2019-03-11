#ifndef QUADCOPTER_H
#define QUADCOPTER_H
 
#include "mbed.h"
#include "Servo.h"
 
//class used for creating a user friendly interface for controlling quadcopter motors
/**
* 
* 
* Example:
* @code
* #include "mbed.h"
* #include "Quadcopter.h"
* Quadcopter quad (p21, p22, p23, p24);   //intance of the Quadcopter class
*
* #define MIN_CALIBRATE 0.3
* #define MAX_CALIBRATE 1.0
*
* int main(){
*   quad.setLimits (MIN_CALIBRATE, MAX_CALIBRATE);    
*   quad.calibrate ();
*   
*   float speed[4];
*   for (int i = 0; i < 4; i++){    //initialise speed to be 0.2
*       speed[i] = 0.2;
*   }
* 
*   while(1){
*       quad.run(speed);    //run
*   }
* }
* @endcode
*
*/
 
 
class Quadcopter {
public:
    /**
    * Constructor.
    * 
    * @param FL - Front Left motor.
    * @param FR - Front Right motor.
    * @param BL - Back Left motor.
    * @param BR - Back Right motor.
    *
    */
    Quadcopter(PinName FL, PinName FR, PinName BL, PinName BR);
    
    /**
    * Function used to calibrate all 4 ESC before actually flying the quadcopter.
    */
    void calibrate ();
    
    /**
    * Function used to calibrate all 4 ESC before actually flying the quadcopter.
    * It does not matter which of inputs is min and which is max as function checks that itself.
    * If inputs are out of boundaries, min value becomes 0.0, and max value becomes 1.0 automatically
    * 
    * @param min - minimum value for the ESC in range from 0.0 to 1.0.
    * @param max - maximum value for the ESC in range from 0.0 to 1.0.
    */
    void setLimits(float min, float max);
    
    /**
    * function for runing motors.
    *
    * @param speed - array of 4 variables, which corresponds to the speed for all 4 motors. speeds are in range from 0.0 to 1.0.
    */
    void run (float* speed);
    
    /**
    * Function used to calculate the speed at which each of the motors should run to be able to stabilise the quadcopter.
    * 
    * @param speed - current speed for each motor.
    * @param actSpeed - actual speed at which motors should run
    * @param rollDiff - calculated using PID library function PID::compute();
    * @param pitchDiff - calculated using PID library function PID::compute();
    *
    */
    void stabilise (float* speed, float* actSpeed, float rollDiff, float pitchDiff);
    
    /**
    * Function used to get the lower calibration limit.
    */
    float getLowerLimit();
    
    /**
    * Function used to get the upper calibration limit.
    */
    float getUpperLimit();
private:  
    
    float min_calibrate;    //min value at which each motor is calibrated
    float max_calibrate;    //max value ...
    Servo* motor[4];        //motors are used with Servo library as ESC take the same input as usual Servo motors...
    float map(float x, float in_min, float in_max, float out_min, float out_max);    //function for mapping values in the range from min calibrate to max_calibrate
};
 
#endif