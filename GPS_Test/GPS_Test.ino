/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */
 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin = 1;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup(){
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
//  getSpeed(); 
getlocation();
}

//double getSpeed(){
//  while (ss.available() > 0){
//    gps.encode(ss.read());
//    if (gps.location.isUpdated()){
//      Serial.print("Speed ->> ");
//      Serial.print(gps.speed.kmph());
//      Serial.println(" Km/h");
//      return gps.speed.kmph();
//    }    
//  }
//}


void getlocation(){
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude ->> ");
      Serial.print(gps.location.lat());
      Serial.println("");
      Serial.print("Longitude ->> ");
      Serial.print(gps.location.lng());
      Serial.println("");
    }    
  }
}
