// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
  //A4接SDA     A5接SCL  D2---INT
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu(0x68);

#define OMIGA_K 2.20 //漂移量 
#include <SoftwareSerial.h>//for Bluetooth
SoftwareSerial BTSerial(3, 4); //Connect HC-06. Use your (TX, RX) settings

#define K 300  //

#define KEY_RIGHT  'R'  //cmdCode
#define KEY_LEFT   'L'    
#define KEY_DOWN   'D'    
#define KEY_UP     'U'  
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//char str[512];
int photocellPin = 7, ana0 = 0,switchPin=6,ana1=0;

float alpha[2],omiga[2];
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 //yaw(偏航），pitch(俯仰），roll（滚转）角度
 //ypr[0];     ypr[1];       ypr[2];    
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
 
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 
void setup() {
  Serial.begin(115200);        // opens serial port, sets data rate to 9600 bps
  BTSerial.begin(9600);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  pinMode(13, OUTPUT);
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
 
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
  delay(2);
 
  // load and configure the DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP ready! Waiting for first interrupt...");
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}
//----------------------------------- 
void loop()
{
    if(isOnceModel()){
      if(isOnceAction())
        delay(80);
    }
    else
      sendCmd();
    /*getAlpha();   
    Serial.print("Alpha ");
    Serial.print(alpha);
    Serial.print("\tOmiga ");
    Serial.println(omiga);*/
   // sprintf(str, "%d",getCmd());
   // Serial.println(str);   
   // BTSerial.print(str);
    //BTSerial.write(byte(10));
    
    delay(2);
}
//===============================================================

void getData(){
  if (!dmpReady) 
    return;
 
  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) 
    return;
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
 
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //从DMP中取出Yaw、Pitch、Roll三个轴的角度，放入数组ypr。单位：弧度
    alpha[0]=-ypr[2] * 180/M_PI; // roll
    alpha[1]=-ypr[1] * 180/M_PI; //pitch
    omiga[0]=mpu.getRotationX()/16.4-OMIGA_K;        //配置是16位表示正负2000°/s, 65536/4000=16.4
    omiga[1]=mpu.getRotationY()/16.4-OMIGA_K;
   } 
}
boolean isOnceAction(){
  float val[2];
  for(int i=0;i<10;i++){
    getData();
    val[0] += omiga[0];
    val[1] += omiga[1];
    delay(2);
  }
  val[0]=val[0]/10.0;
  val[1]=val[1]/10.0;
  if(abs(val[0])>130||abs(val[1])>130){
    if((abs(alpha[0])>abs(alpha[1])+5)&&(abs(val[0])>abs(val[1]))){
      if(abs(alpha[0])>25){
        if(alpha[0]>0){
          BTSerial.print(KEY_LEFT);
          Serial.println("L");
        }
        if(alpha[0]<0){
          Serial.println("R");
          BTSerial.print(KEY_RIGHT);
        }
        return true;
      }
      else
        return false;
    }
    else if((abs(alpha[0])<abs(alpha[1])+5)&&(abs(val[0])<abs(val[1]))){
      if(abs(alpha[1])>20){
        if(alpha[1]>0){
          BTSerial.print(KEY_UP);
          Serial.println("U");
        }
        if(alpha[1]<0){
          Serial.println("D");
          BTSerial.print(KEY_DOWN);
        }
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }
  else
    return false;
}

void sendCmd(){
  // left +  ===right -/*
   getData();
   //if(abs(alpha[0])>abs(alpha[1])+5){
     if(abs(alpha[0])>20){
       if(alpha[0]>0){
         BTSerial.print(KEY_LEFT);
         //BTSerial.write(byte(10));
         Serial.println(KEY_LEFT);
       }  
       else if(alpha[0]<0){   
          BTSerial.print(KEY_RIGHT);
          //BTSerial.write(byte(10));
          Serial.println(KEY_RIGHT);
        }
     }
  // }
   //if(abs(alpha[0])<abs(alpha[1])+5){
     if(abs(alpha[1])>20){
       if(alpha[1]>0){
         BTSerial.print(KEY_UP);
         //BTSerial.write(byte(10));
         Serial.println(KEY_UP);
       }  
       else if(alpha[1]<0){   
          BTSerial.print(KEY_DOWN);
          //BTSerial.write(byte(10));
          Serial.println(KEY_DOWN);
        }
     }
  // }
}
boolean isOnceModel(){
  
  ana0=analogRead(photocellPin);
  ana1=analogRead(switchPin);
  if(ana1>1000){
    digitalWrite(13, LOW);
    return true;
  }  
  if(ana0<=K){
    digitalWrite(13, HIGH);
    return false;
  }
  else{
    digitalWrite(13, LOW);
    return true;
  }
}
