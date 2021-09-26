/*
   심박수 센서
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = 2 (or SDA)
  -SCL = 3 (or SCL)
  -INT = Not connected
*/

#include <Wire.h>
#include "MAX30105.h"
#include <SoftwareSerial.h>
#include "heartRate.h"

MAX30105 particleSensor;


#define TMP 9        //온도 센서
#define XPIN 10       //자이로 센서
#define YPIN A0
#define ZPIN A1
#define BT_RXD 16
#define BT_TXD 14
SoftwareSerial bt(BT_RXD, BT_TXD);

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

float temperature;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;
int arr = 0;

void heartRate_Sensing();
void Temperature_Sensing();
void Gyro_sensor_Sensing();
void send_message();

void setup()
{
  Serial.begin(9600);
  bt.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop()
{

  heartRate_Sensing();

  Temperature_Sensing();

  Gyro_sensor_Sensing();

  send_message();

  


}

void heartRate_Sensing() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}

void Temperature_Sensing() {
  int readValue = analogRead(A0);
  float voltage = readValue * 5000.0 / 1024.0;
  temperature = (voltage - 500) / 10.0;

  Serial.print("Temperature : ");
  Serial.println(temperature);
}

void Gyro_sensor_Sensing() {
  int xRead = analogRead(XPIN);
  int yRead = analogRead(YPIN);
  int zRead = analogRead(ZPIN);

  int xAng = map(xRead, minVal, maxVal, -90, 90);
  int yAng = map(yRead, minVal, maxVal, -90, 90);
  int zAng = map(zRead, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" | y: ");
  Serial.print(y);
  Serial.print(" | z: ");
  Serial.println(z);
}

void send_message() {
  String message = "";

  message += '#';             //시작 문자
  message += temperature;     //온도 값을 문자열로 추가
  message += ',';             //구분자
  message += beatsPerMinute;  //심박수 값을 문자열로 추가
  message += ',';             //구분자
  message += x;               //자이로센서 x축
  message += ',';             //구분자
  message += y;               //자이로센서 y축
  message += ',';             //구분자
  message += z;               //자이로센서 z축
  message += '@';             //종료 문자

  bt.print(message);
}
