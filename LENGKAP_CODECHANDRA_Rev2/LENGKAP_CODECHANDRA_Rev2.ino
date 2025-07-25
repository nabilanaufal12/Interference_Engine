#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <vector>


WiFiClient espClient;
PubSubClient client(espClient);


int counter = 0;
float lat2 = 0;
float long2 = 0;
int motor = 0;
int speed = 1800;
int addAngle = 0;
float radius = 0.8;
int turnAngle = 0;

/////////////////////////////INISIAL ESC//////////////////////////////
Servo esc;
Servo esc2;

////////////////////////////////SERVO//////////////////////////////////
Servo rudderServo1;
Servo rudderServo2;

//////////////////////////////VARIABEL PID//////////////////////////////
float Kp = 1.0; // Nilai Proporsional
float Ki = 0.0; // Nilai Integral
float Kd = 0.1; 
std::vector<double> lats;
std::vector<double> longs;
unsigned long sendTime = 0;


float setpoint = 0; // Titik setelan (azimuth yang diinginkan)

float input = 0; // Nilai masukan dari kompas
float output = 0; // Nilai keluaran untuk servo
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

//////////////////////////////GPS DAN KOMPAS//////////////////////////////
HardwareSerial ss(1);
TinyGPSPlus gps;
QMC5883LCompass compass;

//////////////////////////////SET KOORDINAT//////////////////////////////
double lat1; // Latitude awal
double lon1; // Longitude awal


//////////////////////////////FUNGSI MOTOR//////////////////////////////
void motorMundur() {
  esc.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
}

void motorStop() {
  esc.writeMicroseconds(1550);
  esc2.writeMicroseconds(1550);
}

void motorMaju() {
  esc.writeMicroseconds(min(speed, 1700));
  esc2.writeMicroseconds(min(speed, 1700));
}

//////////////////////////////RUMUS AZIMUT//////////////////////////////

//////////////////////////////RUMUS AZIMUT//////////////////////////////
double toRadians(double degree) {
  return degree * PI / 180.0;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Radius Bumi dalam meter

  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
             sin(dLon / 2.0) * sin(dLon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

float calculateHeading(float lat1, float lon1, float lat2, float lon2) {
  float delta_lon = lon2 - lon1;
  float x = cos(lat2) * sin(delta_lon);
  float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
  float heading = atan2(x, y) * 180.0 / PI;

  if (heading < 0) {
    heading += 360.0;
  }
  return heading;
}

void TaskSerial(void *pvParameters) {
    for (;;) {
        if (Serial.available() > 0) {
          String inp = Serial.readStringUntil('\n');  // Menggunakan '\n' sebagai pemisah
          StaticJsonDocument<2048> docs;
          DeserializationError err = deserializeJson(docs, inp);

          if (!err) {
            if(docs["event"]=="conf"){
                motor = docs["motor"];
                speed = docs["speed"];
                Ki = docs["ki"];
                Kd = docs["kd"];
                Kp = docs["kp"];
                radius = docs["radius"];
            }else if(docs["event"]=="ang"){
                addAngle = docs["angle"];
            }else if(docs["event"]=="latsLen"){
                int value = docs["len"];
                lats = std::vector<double>(value);
            }else if (docs["event"] == "setLats"){
                int key = docs["idx"];
                double val = docs["value"];
                lats[key] = val;
            }else if(docs["event"]=="lonsLen"){
                int value = docs["len"];
                longs = std::vector<double>(value);
            }else if (docs["event"]== "setLongs"){
                int key = docs["idx"];
                double val = docs["value"];
                longs[key] = val;
            }else if(docs["event"]=="belok"){
                int ang = docs["angle"];
                turnAngle += ang;
            }else if(docs["event"]=="sc"){
                int val = docs["value"];
                counter = val;
            }
            
            
          }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void TaskGPS(void *pvParameters) {
    for (;;) {
    while (ss.available() > 0  ){
          if (gps.encode(ss.read()) ){
            if (gps.location.isValid() ) {

              // Membaca azimuth dari kompas
              compass.read();
              int azimuth = compass.getAzimuth();
              int adjustedAzimuth = (azimuth + 360) % 360;

              // Menggunakan data dari GPS (lat, lon)
              lat1 = gps.location.lat();
              lon1 = gps.location.lng();

              // Mengambil titik tujuan berdasarkan counter
              if (lats.size()) lat2 = lats[counter];
              if (longs.size()) long2 = longs[counter];

              // Menghitung jarak dan heading
              double distance = calculateDistance(lat1, lon1, lat2, long2);
              float heading = calculateHeading(lat1, lon1, lat2, long2);
              int adjustedHeading = adjustedAzimuth - heading;
              if (adjustedHeading < 0) {
                adjustedHeading += 360.0;
              }

              const char* latDirection = gps.location.rawLat().negative ? "S" : "N";
              const char* lonDirection = gps.location.rawLng().negative ? "W" : "E";

              float speed = gps.speed.knots();
              float speedKm = gps.speed.kmph();

              
              // PID controller
              input = adjustedHeading;
              float error = setpoint - input;
              if (error > 180) error -= 360;
              else if (error < -180) error += 360;

              unsigned long current_time = millis();
              float dt = (current_time - previous_time) / 1000.0;

              integral += error * dt;
              float derivative = (error - previous_error) / dt;

              output = Kp * error + Ki * integral + Kd * derivative;
              previous_error = error;
              previous_time = current_time;

              // Mengatur servo
              int servoAngle = (90 - output) + addAngle;
              servoAngle = constrain(servoAngle, 0, 180);
              rudderServo1.write(servoAngle);
              rudderServo2.write(servoAngle);

              if(millis()-sendTime >= 500){
                sendTime = millis();
                StaticJsonDocument<200> doc;
                doc["event"] = 0;
                doc["lat"] = lat1;
                doc["latDirection"] = latDirection;
                doc["lon"] = lon1;
                doc["lonDirection"] = lonDirection;
                doc["adjAzimut"] = adjustedAzimuth;
                doc["adjHeading"] = adjustedHeading;
                doc["distance"] = distance;
                doc["counter"] = counter;
                doc["radius"] = radius;
                doc["speed"] = speed;
                doc["speedKm"] = speedKm;

                // Mengirim data ke MQTT
                doc["servoAngle"] = servoAngle;
                doc["addAngle"] = addAngle;
                char jsonBuffer[512];
                serializeJson(doc, jsonBuffer);
                //client.publish("sensor/data", jsonBuffer);
                Serial.println(jsonBuffer);
              }
             

              // Periksa jarak untuk menentukan tujuan selanjutnya
              if (distance <= radius) {
                counter += 1;
              }

              // Kendalikan motor
              if (motor && counter < lats.size()) {
                motorMaju();
              } else {
                motorStop();
              }

            }
          }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
  Serial.begin(250000);

  ss.begin(9600, SERIAL_8N1, 16, 17); // RX = GPIO16, TX = GPIO17

  compass.init();

  esc.attach(18);
  esc2.attach(19);

  rudderServo1.attach(25);
  rudderServo2.attach(26);
  rudderServo1.write(90);
  rudderServo2.write(90);

  delay(3000);
  xTaskCreate(TaskSerial, "TaskSerial", 4096, NULL, 1, NULL);
  xTaskCreate(TaskGPS, "TaskGPS", 4096, NULL, 1, NULL);
}



void loop() {
      
}
