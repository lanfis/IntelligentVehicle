// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.
#define delayTime 50

#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

String recValue;
String str ="";

void setup()
{
  SWSerial.begin(9600);
  ST.autobaud();
  Serial.begin(9600);
  Serial.setTimeout(20);
}

void loop()
{
   if(Serial.available()>0)
  {
    str = Serial.readString();
    recValue = str;
    Serial.print("Receiving : ");  Serial.print(recValue.c_str());  Serial.print("\n");
    //recValue = str.toInt();
    //recValue=Serial.read();
    
    if  (recValue == "1") 
     { 
       ST.motor(1, 20);
       ST.motor(2, 20);
       delay(delayTime);
     }

    if  (recValue == "2")          
     { 
       ST.motor(1, -20);
       ST.motor(2, -20);
       delay(delayTime);
     }
     
    if  (recValue == "3")          
     { 
       ST.motor(1, 20);
       ST.motor(2, -20);
       delay(delayTime);
     }     
     
     if  (recValue == "4")          
     { 
       ST.motor(2, 20);
       ST.motor(1, -20);
       delay(delayTime);
     }    
     
     else          
     { 
       ST.motor(1, 0);
       ST.motor(2, 0);
     }    
 
  }   
       //ST.motor(1, 0);
       //ST.motor(2, 0);
 
}

