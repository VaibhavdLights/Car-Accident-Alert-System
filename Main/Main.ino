#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>
 
#define FIREBASE_HOST "flutter-iot-vp-default-rtdb.firebaseio.com/"
#define WIFI_SSID "IIITS_Student" // Change the name of your WIFI
#define WIFI_PASSWORD "iiit5@2k18" // Change the password of your WIFI
#define FIREBASE_Authorization_key "iH0S1QZIGg2r4xU0oqySYGiVHM9u8YhJ81r3TXau"
 
FirebaseData firebaseData;
FirebaseJson json;

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

//static const int RXPin = 3, TXPin = 1;//GPS
//static const uint32_t GPSBaud = 9600;
//TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);
//SoftwareSerial espSerial(9, 10);

#define BUZZER 15
#define BUTTON 13
#define triggPin 12
#define echoPin 14

long double duration;
long double distance;
String str;
double sensitivity_speed = 2;
double sensitivity_collision = 3;

boolean impact_detected = false;
//Used to run impact routine every 2mS.
unsigned long time1;
unsigned long impact_time;
unsigned long alert_delay = 30000; //30 seconds

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(triggPin,OUTPUT);
  pinMode(echoPin,INPUT);
//  ss.begin(GPSBaud);
  WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST,FIREBASE_Authorization_key);
}

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  getSpeed();
//  Serial.println(distDiff);
  double speed1 = getSpeed();
  delay(500);
  double distDiff = calcDistance();
  double speed2 = getSpeed();
  
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
//  str = String(impact_detected)+String(", ")+String(gps.speed.kmph())+String(", ")+String(gps.location.lat())+String(", ")+String(gps.location.lng());
//    espSerial.println(str);
  if(speed2-speed1 > sensitivity_speed){
    //Impact Detected
    Serial.println("Impact Detected");
    Firebase.setString(firebaseData,"Accident","Impact Detected");
    getlocation();
    digitalWrite(BUZZER,HIGH);
    impact_detected = true;
    impact_time = millis();
//    str = String(impact_detected)+String(", ")+String(gps.speed.kmph())+String(", ")+String(gps.location.lat())+String(", ")+String(gps.location.lng());
//    espSerial.println(str);

    if(impact_detected == true){
      if(millis() - impact_time >= alert_delay) {
        digitalWrite(BUZZER, LOW);
        delay(1000);
        impact_detected = false;
        impact_time = 0;
      }
    }
    if(digitalRead(BUTTON) == LOW){
      delay(200);
      digitalWrite(BUZZER, LOW);
      impact_detected = false;
      impact_time = 0;
  }
    
  }//EO-if
  
  if(distDiff <= sensitivity_collision){
      //send alert
      Serial.println("Collison Detected");
      Firebase.setString(firebaseData,"Accident","Collison Detected");
      Serial.println(distDiff);
      getlocation();
      digitalWrite(BUZZER,HIGH);
      impact_detected = true;
      impact_time = millis();
//      str = String(impact_detected)+String(", ")+String(gps.speed.kmph())+String(", ")+String(gps.location.lat())+String(", ")+String(gps.location.lng());
//      espSerial.println(str);
      
    if(impact_detected == true){
      if(millis() - impact_time >= alert_delay) {
        digitalWrite(BUZZER, LOW);
        delay(1000);
        impact_detected = false;
        impact_time = 0;
      }
    }
    if(digitalRead(BUTTON) == LOW){
      delay(200);
      digitalWrite(BUZZER, LOW);
      impact_detected = false;
      impact_time = 0;
   }
  }
   if(y >= 130 or y <= 50){
     Serial.println("Vehicle is Tilted in Y-Axis");
     Firebase.setString(firebaseData,"Accident","Vehicle is Tilted in Y-Axis");
     getlocation();
     digitalWrite(BUZZER,HIGH);
    impact_detected = true;
    impact_time = millis();
//    str = String(impact_detected)+String(", ")+String(gps.speed.kmph())+String(", ")+String(gps.location.lat())+String(", ")+String(gps.location.lng());
//    espSerial.println(str);
    
    if(impact_detected == true){
      if(millis() - impact_time >= alert_delay) {
        digitalWrite(BUZZER, LOW);
        delay(1000);
        impact_detected = false;
        impact_time = 0;
      }
    }
    if(digitalRead(BUTTON) == LOW){
      delay(200);
      digitalWrite(BUZZER, LOW);
      impact_detected = false;
      impact_time = 0;
   }
   }
   if(z >= 50 and z <= 300){
     Serial.println("Vehicle is Tilted in Z-Axis");
     Firebase.setString(firebaseData,"Accident","Vehicle is Tilted in Z-Axis");
     getlocation();
     digitalWrite(BUZZER,HIGH);
    impact_detected = true;
    impact_time = millis();
//    str = String(impact_detected)+String(", ")+String(gps.speed.kmph())+String(", ")+String(gps.location.lat())+String(", ")+String(gps.location.lng());
//    espSerial.println(str);
    
    if(impact_detected == true){
      if(millis() - impact_time >= alert_delay) {
        digitalWrite(BUZZER, LOW);
        delay(1000);
        impact_detected = false;
        impact_time = 0;
      }
    }
    if(digitalRead(BUTTON) == LOW){
      delay(200);
      digitalWrite(BUZZER, LOW);
      impact_detected = false;
      impact_time = 0;
   }
   }
   
}

double calcDistance(){
  digitalWrite(triggPin,LOW);
  delayMicroseconds(2);
  digitalWrite(triggPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(triggPin,LOW);

  duration = pulseIn(echoPin,HIGH);
  distance = (duration*0.034/2);
  return distance;
}

double getSpeed(){
//  while (ss.available() > 0){
//    gps.encode(ss.read());
//    if (gps.location.isUpdated()){
//      Serial.print("Speed ->> ");
//      Serial.print(gps.speed.kmph());
//      Serial.println(" Km/h");
//      return gps.speed.kmph();
//    }    
//  }
Firebase.setString(firebaseData, "Speed", 1.0);
delay(500);
Firebase.setString(firebaseData, "Speed", 0.0);
return 1.0;
}

void getlocation(){
  while (1){
//    gps.encode(ss.read());
    if (1){
      Serial.print("Latitude ->> ");
//      Serial.print(gps.location.lat());
      String lati = "13.55562";
      Serial.print(lati);
      Firebase.setString(firebaseData, "Latitude", lati);
      delay(500);
      lati = "13.55563";
      Firebase.setString(firebaseData, "Latitude", lati);

      Serial.println("");
      Serial.print("Longitude ->> ");
//      Serial.print(gps.location.lng());
      String longi = "80.02692";
      Serial.print(longi);
      Firebase.setString(firebaseData, "Longitude", longi);
      delay(500);
      longi = "80.02693";
      Firebase.setString(firebaseData, "Longitude", longi);
      Serial.println("");
      delay(500);
    }    
  }
}
