#include "MPU6050_6Axis_MotionApps20.h"
#include "mbed.h"
#include "config.h"
#include <stdio.h>
#include"math.h"

#define M_PI 3.1415926535897932
#define dtr(x) ( x * 0.01745329f )
#define rtd(x) ( x * 57.29578f )


RawSerial pc(USBTX, USBRX);


MPU6050 mpu(PB_9, PB_8);     
InterruptIn INT0(PA_8);     

const int FIFO_BUFFER_SIZE = 128;
uint8_t fifoBuffer[FIFO_BUFFER_SIZE],rpt1,mcount,ini=0,cal=0;
uint16_t fifoCount;
uint16_t packetSize,t2;
int16_t mx,my,mz,mx1,my1,mz1,mx2,my2,mz2;
bool dmpReady;
uint8_t mpuIntStatus;
const int snprintf_buffer_size = 100;
char snprintf_buffer[snprintf_buffer_size];
Timer t;
float ypr[3], magcali[3],heading;
float heading2;

struct Offset {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
}offset = {150, -350, 1000, -110, 5, 0};    // Measured values

struct MPU6050_DmpData {
    Quaternion q;
    VectorFloat gravity;    // g
    float yaw, pitch, roll;     // rad
}dmpData;

bool Init();
void dmpDataUpdate();



bool Init() {
    pc.baud(PC_BAUDRATE);
    
    INT0.mode(PullDown);
    INT0.fall(dmpDataUpdate);
    
    mpu.initialize();
    if (mpu.testConnection()) {
        pc.puts("MPU6050 test connection passed.\n");
    } else {
        pc.puts("MPU6050 test connection failed.\n");
        return false;
    }
    mpu.initAK8975A(magcali);
    printf("\n\rMagCal: %f\t %f\t %f\t\n\r", magcali[0], magcali[1], magcali[2]);
    if (mpu.dmpInitialize() == 0) {
        pc.puts("succeed in MPU6050 DMP Initializing.\n");
    } else {
        pc.puts("failed in MPU6050 DMP Initializing.\n");
        return false;
    }
    mpu.setXAccelOffset(offset.ax);
    mpu.setYAccelOffset(offset.ay);
    mpu.setZAccelOffset(offset.az);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setXGyroOffsetUser(offset.gx);
    mpu.setYGyroOffsetUser(offset.gy);
    mpu.setZGyroOffsetUser(offset.gz);
    mpu.setDMPEnabled(true);    // Enable DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;    // Enable interrupt.
    
    pc.puts("Init finish!\n");
    return true;
}


void dmpDataUpdate() {
    if (dmpReady == false) return;   
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        pc.puts("FIFO overflow!\n");
        return;
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        
        mpu.getFIFOBytes(fifoBuffer, packetSize); 
        t2 = (t.read_ms())/1000;
        if(cal == 4){
            while(fifoCount > packetSize){
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                if(cal == 2){
                    cal = 2;
                }
                fifoCount = mpu.getFIFOCount();
            }
        }
        /*mcount++;
        if(mcount>20){
            mpu.getMag(&mx, &my, &mz);
            mcount=0;
        }*/
        if(t2%2==0){
            if((rpt1==0) && (cal == 2)){
                printf("\nTime:%d\r\n",t2);
                rpt1=1; 
                if(ini == 1){
                    ini = 2;
                }
                   
                mpu.dmpGetQuaternion(&dmpData.q, fifoBuffer);
                mpu.dmpGetGravity(&dmpData.gravity, &dmpData.q);
                mpu.dmpGetYawPitchRoll(ypr, &dmpData.q, &dmpData.gravity);      
                
               if(cal == 4){
                    while(fifoCount > packetSize){
                        mpu.getFIFOBytes(fifoBuffer, packetSize);
                        cal = 2;
                        fifoCount = mpu.getFIFOCount();
                    }
                }
                dmpData.yaw = rtd(ypr[0]);
                dmpData.pitch = rtd(ypr[1]);
                dmpData.roll = rtd(ypr[2]);
                if(dmpData.yaw<0){
                    dmpData.yaw += 360;
                }    
                //float mix = mx;
                int16_t ax,ay,az,gx,gy,gz;
                mpu.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);
                mpu.getHeading(&mx1,&my1,&mz1);
                /*if((mx>-20000) &&(mx<20000) && (my<0) && (heading>330) && (heading<360) && (heading>0) && (heading<30)){
                    my = -my;
                }*/
                printf("\n\rMagneto: %d\t %d\t %d\t\n\r", mx, my, mz);
                printf("\n\rMagneto: %d\t %d\t %d\t\n\r", mx1, my1, mz1);
                mx1 = -mx1;
                float mag0 = mx1*((10.*1229.)/4096.)*magcali[0]-5.;
                float mag1 = my1*((10.*1229.)/4096.)*magcali[1]-95.;
                float mag2 = mz1*((10.*1229.)/4096.)*magcali[2]-260.;
                //heading = rtd(atan2(mag1,mag0));
                
                printf("\n\rMagneto: %f\t %f\t %f\t\n\r",mag0,mag1,mag2 );
                //Tilt compenstaion
                float xc = mag0*cos(ypr[1]) + mag2*sin(ypr[1]);
                float yc = mag0*sin(ypr[2])*sin(ypr[1]) + mag1*cos(ypr[2]) - mag2*sin(ypr[2])*cos(ypr[1]);
                
                
               /*float xc = mag0 * cos(ypr[1]) +  mag1* sin(ypr[2]) * sin(ypr[1]) + mag2 * cos(ypr[2]) * sin(ypr[1]);
               float yc = mag1 * cos(ypr[2]) - mag2 * sin(ypr[2]);*/
                heading2 = atan2((double)xc,(double)yc);
                
                heading2 = -1*heading2;
                heading2=rtd(heading2); 
                while (heading2 < 0) heading2 += 360;
                while (heading2 > 360) heading2 -= 360;
                heading = heading2;
                printf("Heading comp: %2.2f\n",heading);
                
                /*if(mx<0 && my>=0)  heading = (90 - heading);
                if(mx<0 && my<0)   heading = (90 - heading);
                if(mx>=0 && my<0)  heading = (-90 + heading);
                if(mx>=0 && my>=0) heading = (90 + heading);*/
                
                /*if(mx<0 && my>=0)  heading = (90 + heading);
                if(mx<0 && my<0)   heading = (-90 + heading);
                if(mx>=0 && my<0)  heading = (90 + heading);
                if(mx>=0 && my>=0) heading = (90 - heading);
                printf("Headinga: %2.2f\n",heading);*/
                
                /*if(mx>=0 && my>=0)  heading2 = 90 - heading2;
                if(mx>=0 && my<0)   heading2 = 90 + heading2;
                if(mx<0 && my<0)    heading2 = 270 - heading2;
                if(mx<0 && my>=0)   heading2 = 270 + heading2;
                printf("Heading 2: %2.2f\n",heading2);*/
                //printf("Heading i: %2.2f\n",heading);
                //Keep mag straight
                /*if(mx>=0 && my>=0)  heading = 90 - heading;
                if(mx>=0 && my<0)   heading = 90 + heading;
                if(mx<0 && my<0){
                    if(my<-19000)
                    heading = 270 + heading;
                    else
                    heading = 270 - heading;
                }
                if((mx>=0 && my<0)&&(my<-23000)){
                    heading = 180 - heading;
                    }   
                
                if(mx<0 && my>=0)   heading = 270 + heading;
                 
                
                
                
                printf("Heading: %2.2f\n",heading);*/
                if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Yaw:%6.2fdeg, Pitch:%6.2fdeg, Roll:%6.2fdeg\n", dmpData.yaw, dmpData.pitch, dmpData.roll ) < 0 ) return;
                pc.puts(snprintf_buffer);
                       
                float temp = mpu.getTemperature() / 340.0 + 36.53;
                if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Temp:%4.1fdeg\n", temp ) < 0 ) return;
                pc.puts(snprintf_buffer);
         
                pc.puts("\n");
            }
        }else{rpt1=0;}
    }
}