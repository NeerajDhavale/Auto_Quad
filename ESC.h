#include "mbed.h"


class ESC{
    public:
    ESC(PinName pinA,PinName pinB,PinName pinC,PinName pinD) : pwmscA(pinA),pwmscB(pinB),pwmscC(pinC),pwmscD(pinD){
        pwmscA.pulsewidth_us(1000);
        pwmscB.pulsewidth_us(1000);
        pwmscC.pulsewidth_us(1000);
        pwmscD.pulsewidth_us(1000);
        }
    void throttlevalue(int fA,int fB,int fC,int fD, int eA,int eB,int eC,int eD){
        for(xA=fA,xB=fB,xC=fC,xD=fD;xA<=eA || xB<=eB || xC<=eC || xD<=eD; xA+=1,xB+=1.0,xC+=1.0,xD+=1.0){
            if(xA<=eA)
            pwmscA.pulsewidth_us(xA);
            if(xB<=eB)
            pwmscB.pulsewidth_us(xB);
            if(xC<=eC)
            pwmscC.pulsewidth_us(xC);
            if(xD<=eD)
            pwmscD.pulsewidth_us(xD);
        //printf("\nSending left=%d and right=%d",xL,xR);
        }
        }
/*    void throttlevalued(int fL,int eL,float fR,float eR){
        for(xL=fL,xR=fR;xL>=eL || xR>=eR;xL-=1,xR-=1.0){
            if(xL>eL)
            pwmscL.pulsewidth_us(xL);
            if(xR>eR)
            pwmscR.pulsewidth_us(xR);
        }
    }*/
    void throttlevalue(int m,int val){
        if(m == 0){
            pwmscA.pulsewidth_us(val);
            }
        else if(m == 1){
            pwmscB.pulsewidth_us(val);
            }
        else if(m == 2){
            pwmscC.pulsewidth_us(val);
        }
        else if(m == 3){
            pwmscD.pulsewidth_us(val);
        }
    }
    void throttle1000(){
        pwmscA.pulsewidth_us(1000);
        pwmscB.pulsewidth_us(1000);
        pwmscC.pulsewidth_us(1000);
        pwmscD.pulsewidth_us(1000);
        printf("\nSending output=1000");
        }
    protected:
    PwmOut pwmscA,pwmscB,pwmscC,pwmscD;
    int xA,xB,xC,xD;
    
}motor(D10,D11,D12,D13);