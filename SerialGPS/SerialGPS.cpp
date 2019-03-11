#include "SerialGPS.h"

SerialGPS::SerialGPS(PinName tx, PinName rx, int Baud) : Sgps(tx, rx) {
    Sgps.baud(Baud);    
    longitude = 0.0;
    latitude = 0.0;        
}

int SerialGPS::sample() {
    char ns, ew, unit;

    while(1) {        
        getline();

        if(sscanf(msg, "GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f", &time, &latitude, &ns, &longitude, &ew, &lock, &sats, &hdop, &alt, &unit, &geoid) >= 1) { 
            if(!lock) {
                
                time = 0.0;
                longitude = 0.0;
                latitude = 0.0;
                sats = 0;
                hdop = 0.0;
                alt = 0.0;
                geoid = 0.0;        
                return 0;
            } else {
 
                if(ns == 'S') {    latitude  *= -1.0; }
                if(ew == 'W') {    longitude *= -1.0; }
                float degrees = trunc(latitude / 100.0f);
                float minutes = latitude - (degrees * 100.0f);
                latitude = degrees + minutes / 60.0f;    
                degrees = trunc(longitude / 100.0f);
                minutes = longitude - (degrees * 100.0f);
                longitude = degrees + minutes / 60.0f;
                return 1;
            }
        }
    }
}

float SerialGPS::trunc(float v) {
    if(v < 0) {
        v*= -1.0;
        v = floor(v);
        v*=-1.0;
    } else {
        v = floor(v);
    }
    return v;
}

void SerialGPS::getline() {
    while(Sgps.getc() != '$');    

    for(int i=0; i<256; i++) {
        msg[i] = Sgps.getc();
        if(msg[i] == '\r') {
            msg[i] = 0;
            return;
        }
    }
    error("Overflowed message limit");
}