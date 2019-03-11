#include "mbed.h"

#ifndef MBED_GPS_H
#define MBED_GPS_H

class SerialGPS {
public:   
    SerialGPS(PinName tx, PinName rx, int Baud);
    
    int sample();
    
    int lock;
   
    float longitude;

    float latitude;
    
    float time;
    
    int sats;
    
    float hdop;
    
    float alt;

    float  geoid;
    
    char msg[256];
    
    
private:
    float trunc(float v);
    void getline();
    
    Serial Sgps;
};

#endif