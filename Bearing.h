#include "mbed.h"
#include <math.h>
#define dtr(x) ( x * 0.01745329f )
#define rtd(x) ( x * 57.29578f )
uint8_t d1;
float gpslatR, deslatR, deltalat,deltalongi,deltalatR,deltalongiR,ai,di,X,Y,z;
float p,turn,igr,grba,rot;
float pi = 3.141592f;

float bearing(float gpslat,float gpslongi, float deslat, float deslongi){
    gpslatR = dtr(gpslat);
    deslatR = dtr(deslat);
    deltalat =  deslat - gpslat;                             
    deltalongi = deslongi - gpslongi;
    deltalatR = dtr(deltalat) ;
    deltalongiR = dtr(deltalongi) ;

    X = cos(deslatR) * sin(deltalongiR);
    Y = cos(gpslatR) * sin(deslatR);
    z = sin(gpslatR) * cos(deslatR) * cos(deltalongiR);
    z=Y-z;
    p=atan2(X,z);
    p= rtd(p);
    if(p<0){
        p += 360;
    }
    printf("Bearing = %f\n\r",p); 
    return p;
}

                             // DISTANCE CALCULATION //
float distance(){                                 
    ai = sin(deltalatR/2)*sin(deltalatR/2)+cos(gpslatR)*cos(deslatR)*sin(deltalongiR/2)*sin(deltalongiR/2);
    di = 2*atan2(sqrt(ai),sqrt(1-ai));
    di = (6371000*di)/1000;
    printf("Distance to go = %f km \n\n\r",di);
    return di;
}
float xh,yh,xp,yp,a;
void direction(float yaw, float heading){
    
    if((d1 == 0) && (ini == 2)){
        if((heading<=90)&&(heading>0)){
            xh = 1.0f;
            a = 90 - heading;
            a = dtr(a);
            yh = xh*tan(a);
        }
        if((heading>90) && (heading<180)){
            xh = 1.0f;
            a = 90 - heading;
            a = dtr(a);
            yh = xh*tan(a);
            printf("\n\rXh:%0.3f",xh);
            printf("\n\rYh:%0.3f",yh);
            
            printf("\n\rAh2:%0.3f",a);
        }
        if((heading>180) && (heading<=270)){
            xh = -1.0;
            a = (90 - heading);
            a = dtr(a);
            yh = xh*tan(a);
            
        }
        if((heading>270) && (heading<360)){
            xh = -1;
            a = 450 - heading;
            a = dtr(a);
            yh = xh*tan(a);
        }
        if((heading==0) || (heading==360)){
            xh = 0;
            yh = 1;
        }
        if(heading==180){
            xh = 0;
            yh = -1;
        }
        if((p<=90)&&(p>0)){
            xp = 1;
            a = 90 - p;
            a= dtr(a);
            yp = xp*tan(a);
        }
        if((p>90) && (p<180)){
            xp = 1.0;
            a = (90 - p);
            a = dtr(a);
            yp = xp*tan(a);
            printf("\n\rXp:%0.3f",xp);
            printf("\n\rYp:%0.3f",yp);
        }
        if((p>180) && (p<=270)){
            xp = -1.0;
            a = 90 - p;
            a = dtr(a);
            yp = xp*tan(a);
            printf("\n\rAp2:%0.3f",a);
            printf("\n\rXp:%0.3f",xp);
            printf("\n\rYp:%0.3f",yp);
        }
        if((p>270) && (p<360)){
            xp = -1;
            a = 450 - p;
            a = dtr(a);
            yp = xp*tan(a);
        }
        if((p==0) || (p==360)){
            xp = 0;
            yp = 1;
        }
        if(p==180){
            xp = 0;
            yp = -1;
        }
        rot = atan2(yh,xh) - atan2(yp,xp);
        if(rot<-pi){
            rot += 2*pi;
        }else if(rot>pi){
            rot -= 2*pi;
        }
        rot = rtd(rot);
        grba = yaw + rot;
        if(grba<0){
            grba +=360;
        }
        else if(grba>360){
            grba -=360;
        } 
        printf("\n\rVector:%0.3f",rot);
        printf("\n\rGrba:%0.3f\n\r",grba);

    } 
}