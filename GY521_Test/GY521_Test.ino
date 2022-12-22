#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>


MPU6050 mpu;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
 

 
int val;
int prevVal;

int valax; 
int valay;
int valaz;

const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; int maxVal=402;

double x; double y; double z;

#define triggPin 12
#define echoPin 14
long duration;
long distance;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    Serial.println("Initialize MPU");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
    pinMode(triggPin,OUTPUT);
    pinMode(echoPin,INPUT);

}

void loop()
{
  digitalWrite(triggPin,LOW);
  delayMicroseconds(2);
  digitalWrite(triggPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(triggPin,LOW);

  duration = pulseIn(echoPin,HIGH);
  distance = (duration*0.034/2);
//  analogWrite(7,distance*10);
  Serial.print("Distance ->> ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(1000);
delay(1000);

Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

Serial.print("AngleX= "); Serial.println(x);

Serial.print("AngleY= "); Serial.println(y);

Serial.print("AngleZ= "); Serial.println(z); Serial.println("-----------------------------------------"); delay(400); 
}
