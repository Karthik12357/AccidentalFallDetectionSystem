#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

const int ledPin = 2;
float sensitivity = 1.0;

unsigned long fallTime = 0;
bool fallDetected = false;
bool smsSent = false;

#define RXD2 16  // GSM RX
#define TXD2 17  // GSM TX

#define RXD1 4   // GPS RX
#define TXD1 2  // GPS TX

#define PHONE_NUMBER "+918838296344"

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // GSM
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // GPS
  pinMode(ledPin, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  Serial.println("MPU6050 ready!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z) / 9.81;

  float freefallThreshold = 0.4 * sensitivity;
  float impactThreshold   = 2.0 / sensitivity;

  bool freefall = accelMag < freefallThreshold;
  bool impact = accelMag > impactThreshold;

  if ((freefall && impact) || (impact && !fallDetected)) {
    fallDetected = true;
    fallTime = millis();
    smsSent = false;
    Serial.println("FALL DETECTED");
  }

  if (fallDetected) {
    if (millis() - fallTime < 5000) {
      digitalWrite(ledPin, (millis() / 100) % 2);
    } else {
      digitalWrite(ledPin, LOW);
      if (!smsSent) {
        sendLocationSMS();
        smsSent = true;
      }
      fallDetected = false;
    }
  }

  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  delay(50);
}

void sendLocationSMS() {
  Serial.println("Sending location via GSM...");

  double latitude = 0.0, longitude = 0.0;
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) { 
    while (Serial1.available()) {
      gps.encode(Serial1.read());
    }
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      break;
    }
  }

  if (!gps.location.isValid()) {
    Serial.println("No valid GPS data. Sending last known location or 'Unavailable'.");
  }

  gsmCommand("AT", 1000);
  gsmCommand("AT+CMGF=1", 1000);

  Serial2.print("AT+CMGS=\"");
  Serial2.print(PHONE_NUMBER);
  Serial2.println("\"");
  delay(1000);

  Serial2.print("Fall Detected!\n");
  if (gps.location.isValid()) {
    Serial2.print("Latitude: ");
    Serial2.println(latitude, 6);
    Serial2.print("Longitude: ");
    Serial2.println(longitude, 6);
    Serial2.print("https://maps.google.com/?q=");
    Serial2.print(latitude, 6);
    Serial2.print(",");
    Serial2.println(longitude, 6);
  } else {
    Serial2.println("Location: Unavailable");
  }

  Serial2.write(26);
  delay(5000);
  Serial.println("SMS sent!");
}

String gsmCommand(String cmd, int wait) {
  Serial2.println(cmd);
  delay(wait);
  return readResponse();
}

String readResponse() {
  String response = "";
  long startTime = millis();
  while (millis() - startTime < 3000) {
    while (Serial2.available()) {
      response += (char)Serial2.read();
    }
  }
  return response;
}
