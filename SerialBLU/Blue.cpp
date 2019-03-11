#include "Blue.h"
#include <ctype.h>

Blue::Blue(PinName tx, PinName rx, int Baud) : tooth(tx, rx) {
    tooth.baud(Baud);    
    deslat = 0.0;
    deslongi = 0.0;        
}

void Blue::setvalue(){      
            getdata();
            sscanf(data, "%f,%f",&deslat,&deslongi); 
            printf("%.4f,%.4f \n\r",deslat,deslongi);
} 


void Blue::getdata() {  
    printf("\n\rPlease Enter Latitude and Longitude \n\r: ");
    for(int i=0; i<15; i++) {
        data[i] = tooth.getc();
        if(isdigit(data[i] == 0) &&(i !=2 ||i !=7  || i !=10 )) {
            printf("\n\rInvalid Input : Please Give Input in form : Latitude,Longitude");
            getdata();
        }
    }
}