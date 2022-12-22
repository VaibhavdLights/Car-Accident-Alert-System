#include <FirebaseESP8266.h>
//#include <FirebaseArduino.h>
#include  <ESP8266WiFi.h>
 
#define FIREBASE_HOST "flutter-iot-vp-default-rtdb.firebaseio.com/"
#define WIFI_SSID "IIITS_Student" // Change the name of your WIFI
#define WIFI_PASSWORD "iiit5@2k18" // Change the password of your WIFI
#define FIREBASE_Authorization_key "iH0S1QZIGg2r4xU0oqySYGiVHM9u8YhJ81r3TXau"
 
FirebaseData firebaseData;
FirebaseJson json;

String s;

void setup() {
// Open serial communications and wait for port to open:
Serial.begin(115200);
while (!Serial) {
; // wait for serial port to connect. Needed for native USB port only
}
   WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
   Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST,FIREBASE_Authorization_key);
}

void loop() { // run over and over
if (Serial.available()) {
s=Serial.read();
Serial.println(s);
Firebase.pushString(firebaseData,"datax",s);
}
}
