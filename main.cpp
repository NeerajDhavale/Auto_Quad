#include "mbed.h"
#include "MPU.h"
#include "Blue.h"
#include "SerialGPS.h"
#include "Bearing.h"
#include "Quadcopter.h"
#include "PID.h"
#include "MPU6050.h"
#include "ESC.h"
float homelat,homelon,tlat,tlong;
uint16_t t1,ta;
uint8_t rpt0=0,ro=0;
SerialGPS gps(D8, D2, 9600);
Blue Blu(PC_6, D9, 9600);

#define OFFSET_SAMPLES 50

//define how the accelerometer is placed on surface
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 0

//ESC calibration values
#define MAX_CALIBRATE 1.0
#define MIN_CALIBRATE 0.0

//Just to remember which motor corresponds to which number...
#define FL   0    // Front left    
#define FR   1    // Front right
#define BL   2    // back left
#define BR   3    // back right

//input and output values for pitch
#define PITCH_IN_MIN -90.0
#define PITCH_IN_MAX 90.0
#define PITCH_OUT_MIN -0.1
#define PITCH_OUT_MAX 0.1

//input and output values for roll
#define ROLL_IN_MIN -90.0
#define ROLL_IN_MAX 90.0
#define ROLL_OUT_MIN -0.1
#define ROLL_OUT_MAX 0.1

//PID intervals/constants
#define Kc 1.0
#define Ti 0.00
#define Td 0.00
#define RATE 0.01

float map(float x, float in_min, float in_max, float out_min, float out_max);   

Quadcopter quad (A0,A1,A2,A3);   //intance of the Quadcopter class

//put Kc, Ti, Td, interval for both pitch and roll PID models
PID pitchPID (Kc, Ti, Td, RATE);
PID rollPID (Kc, Ti, Td, RATE);


int main() {
                                   
    float val = 0.0;
    char buff;  
    
    float pitchDiff;    
    float rollDiff;     
    
    float speed[4];     
    float actSpeed[4];  
    
    float accOffset[3]; 
    float gyroOffset[3];
    float angle[3];     

    float prevTime;    
    
    
                                   //Dest-Coord
    t.start();      
                       
    Blu.setvalue();
    printf("deslat = %f",Blu.deslat);
    printf("deslongi = %f",Blu.deslongi);
    while(1) {
        printf("deslongi");
        t1=(t.read_ms())/1000;
        if(t1>40){
            cal=2;
        }
        if(t1%8==0){
            if(rpt0==0){
                t.start();                          
                t1=(t.read_ms())/1000;
                INT0.disable_irq();
                cal = 4;
                rpt0=1;
                uint8_t to=1;
                if (1) {
                    gps.sample();
                    printf("\nsats %d, lat %f, long %f, alt %f\n\r", gps.sats, gps.latitude, gps.longitude, gps.alt);
                    tlat=gps.latitude;
                    tlat = tlat*10000;
                    int tlat1 = tlat; 
                    tlat = tlat1*0.0001;
                    tlong=gps.longitude;
                    tlong = tlong*10000;
                    int tlong1 = tlong; 
                    tlong = tlong1*0.0001;
                    
                                               // BEARING and DISTANCE CALCULATION  //
                    p = bearing(gps.latitude, gps.longitude, Blu.deslat, Blu.deslongi);
                    di = distance();
                    INT0.enable_irq();      
          
            
            
        
            rollPID.setProcessValue (angle[Y_AXIS]);    
            pitchPID.setProcessValue (angle[X_AXIS]);
        
            pitchDiff = pitchPID.compute();     //compute the difference
            rollDiff = rollPID.compute();
            
        //    pc.printf ("pitchDiff=%0.4f, rollDiff=%0.4f\n", pitchDiff, rollDiff);
            quad.stabilise(speed, actSpeed, rollDiff, pitchDiff);   //stabilise the speed by giving out actual Speed value
            
                    if(/*gps.lock == 1  && */ini == 0){
                        
                                                // MPU-Initialize //
                        MBED_ASSERT(Init() == true);
                        ini=1;
                        homelat = tlat;
                        homelon = tlong;
                        
                    }
                }
            }
        }else{rpt0=0;}
        if(/*(gps.lock == 1) && (ini%2 == 0) && */(rpt1 == 1)){
            rpt1=2;
            motor.throttle1000();
            wait(10);
            motor.throttlevalue(1000,1000,1000,1000,1400,1400,1400,1400);
            if(ro == 0){
                ro = 1;
                direction(dmpData.yaw,heading);
            }
            
            if(rot<0){
                if((grba - dmpData.yaw)>180){
                    turn = grba - (360 + dmpData.yaw);
                }else{turn = grba - dmpData.yaw;}
            }
            if(rot>0){
                if((grba - dmpData.yaw)<-180){
                    turn = (grba + 360) - dmpData.yaw;
                }else{turn = grba - dmpData.yaw;}
            }
            
            if(turn<-1){
                printf("Rotate : %0.2f",turn);  //control yaw
            }else if(turn>1){
                printf("Rotate : %0.2f",turn);  //control yaw
            }else{
                printf("Alligned to Destination Direction ");// control esc
            }
        }
        if(t1>30){
            t.start();
        }
        if((tlat == Blu.deslat)&&(tlong == Blu.deslongi)){
            printf("\n\r You Have Reached Your Destination!!");
            uint16_t tn = (t.read_ms())/1000;
            if(d1==1){
                ta = (t.read_ms())/1000;
                d1=2;
            }
            if(tn-ta>120){
                d1=0;
                ini = 2;
                ro = 0;
                Blu.deslat = homelat;
                Blu.deslongi = homelon;
            }
        }
    }
}