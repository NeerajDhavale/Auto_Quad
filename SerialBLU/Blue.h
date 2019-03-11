#include "mbed.h"

#ifndef MBED_BLU_H
#define MBED_BLU_H

class Blue{
    
public :
    Blue(PinName Tx, PinName Rx, int Baud);
    void setvalue();
    float deslat,deslongi;
    char data[15];
     
private:
    void getdata();
    Serial tooth;
    
};
#endif     