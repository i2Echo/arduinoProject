#include "UsbKeyboard.h"
//#include <SoftwareSerial.h>

#define KEY_RIGHT  79  
#define KEY_LEFT   80    
#define KEY_DOWN   81    
#define KEY_UP     82 

//SoftwareSerial BTSerial(2, 3); //Connect HC-06. Use your (TX, RX) settings

String str="";
int flag=0;

void setup()
{
  TIMSK0 &= !(1 << TOIE0);        //
  Serial.begin(9600);
 // BTSerial.begin(9600);
}
void loop()
{
    UsbKeyboard.update();
    
    //while(BTSerial.available()){
    //str+=char(BTSerial.read()); 
    while(Serial.available()){
    str+=char(Serial.read()); 
    flag=1;
    delay(2);  
    } 
   if(flag==1){ 
    Serial.print(str);
    unCode(str[0]);
    flag=0; 
    str= String("");
    delay(50);
    }
  
}
//=======================================================
void unCode(char aChar){
  switch (aChar){
    case 'L' :
        UsbKeyboard.sendKeyStroke(KEY_LEFT);
        //Serial.println('L');
        break;
    case 'R' :
        UsbKeyboard.sendKeyStroke(KEY_RIGHT);
        //Serial.println(KEY_RIGHT);
        break;
    case 'D' :
        UsbKeyboard.sendKeyStroke(KEY_DOWN);
        //Serial.println(KEY_DOWN);
        break;
    case 'U' :
        UsbKeyboard.sendKeyStroke(KEY_UP);
        //Serial.println(KEY_UP);
        break;
    default :
        break;       
  }
}
