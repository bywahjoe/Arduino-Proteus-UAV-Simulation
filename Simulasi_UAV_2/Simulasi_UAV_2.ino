#include <Servo.h>
#include "pinku.h"
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


String mylat = "", mylong = "";
//GPS
TinyGPSPlus gps;

//BMP
Adafruit_BMP085 bmp;

//Servo
Servo myservo1;
Servo myservo2;
Servo myservo3;

//Gyro
float valX, valY, valZ;
int pressure, temp;
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  myservo1.attach(pinServo1);
  myservo2.attach(pinServo2);
  myservo3.attach(pinServo3);
  runServo(90, 90, 90);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  runMotor(50);

  bmp.begin();
  Serial.println("WAIT GPS");
}

void loop() {
  //Wait first GPS Data
  if (mylat == "" || mylong == "") {
    myGPS();
  } else {
    readGPS();
    readGyro();
    readBMP();
    Serial.println();
    delay(500);
  }
}
void readGPS() {
  Serial.print("[GPS]   - ");
  Serial.print("Latitude   = ");
  Serial.println(mylat);
  Serial.print("[GPS]   - ");
  Serial.print("Longitude  = ");
  Serial.println(mylong);
}
void myGPS() {
  //TES,89,87,90,.... P:[20]
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        Serial.print("[GPS]   - ");
        Serial.print("Latitude   = ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("[GPS]   - ");
        Serial.print("Longitude  = ");
        Serial.println(gps.location.lng(), 6);
        mylat = String(gps.location.lat(),6);
        mylong = String(gps.location.lng(),6);
      }
      else
        Serial.println("Wait Valid Data..");
    }
  }
}
void runServo(int val1, int val2, int val3) {
  myservo1.write(val1);
  myservo2.write(val2);
  myservo3.write(val3);
  Serial.print("[SERVO] - "); Serial.print(val1); Serial.print(","); Serial.print(val2); Serial.print(","); Serial.println(val3);
}
void runMotor(int myspeed) {
  //digitalWrite(motorPin1,HIGH);
  myspeed = constrain(myspeed, 0, 255);

  analogWrite(motorPin1, myspeed);
  digitalWrite(motorPin2, LOW);
  Serial.print("[MOTOR] - "); Serial.print("Speed = "); Serial.println(myspeed);
}
int gryoToServo(float value) {
  int result;
  if (value == 0) result = 90;
  else if (value > 0)result = 91 + (value * 90);
  else result = 90 - (abs(value) * 90);

  return result;
}
void readGyro() {
  int valServo1, valServo2, valServo3;

  valX = ((analogRead(gyroX) * 2.0) / 1024.0) - 1;
  valY = ((analogRead(gyroY) * 2.0) / 1024.0) - 1;
  valZ = ((analogRead(gyroZ) * 2.0) / 1024.0) - 1;
  Serial.print("[GYRO]  - X:"); Serial.print(valX); Serial.print(",Y:"); Serial.print(valY); Serial.print(",Z:"); Serial.println(valZ);
  //Calculate Servo
  valServo1 = gryoToServo(valX);
  valServo2 = gryoToServo(valY);
  valServo3 = gryoToServo(valZ);
  runServo(valServo1, valServo2, valServo3);
}
void readBMP() {
  //  83654
  Serial.print("[BMP]   - AIR Pressure: "); Serial.print(bmp.readPressure()), Serial.println(" Pa");

  int speedMotor = map(bmp.readPressure(), 80000, 95000, 0, 255);
  runMotor(speedMotor);
}
