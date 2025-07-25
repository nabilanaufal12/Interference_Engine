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


const char* ssid = "PERPUSTAKAAN";
const char* password = "surahman";
const char* mqtt_server = "192.168.1.105";
const int port = 1883;
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
int counter = 0;
float lat2 = 0;
float long2 = 0;
int motor = 0;
int speed = 1800;
int addAngle = 0;

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


float setpoint = 0; // Titik setelan (azimuth yang diinginkan)

float input = 0; // Nilai masukan dari kompas
float output = 0; // Nilai keluaran untuk servo
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

//////////////////////////////GPS DAN KOMPAS//////////////////////////////
HardwareSerial ss(0);
TinyGPSPlus gps;
QMC5883LCompass compass;

//////////////////////////////SET KOORDINAT//////////////////////////////
double lat1; // Latitude awal
double lon1; // Longitude awal

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* payload, unsigned int length) { //perintah untuk menampilkan data ketika esp32 di setting sebagai subscriber
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String jsonData = "";
  for (int i = 0; i < length; i++) {
    jsonData += (char)payload[i];
  }
  Serial.println(jsonData);
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  if (String(topic) == "data/addAngle") addAngle = doc["addAngle"]; 
  if (String(topic) == "data/setCounter"){
    int value = doc["value"];
    counter  = (lats.size() + value) % lats.size() ;
  }
  if (String(topic) == "data/setLats"){
    JsonArray latArray = doc["lats"].as<JsonArray>();
    counter = doc["counter"];

      for (JsonVariant v : latArray) {
        lats.push_back(v.as<double>());
      }
      for(double lat : lats){
        Serial.print("Lat : ");
        Serial.println(lat, 8);
      }
  }
  if (String(topic) == "data/setLongs"){
    JsonArray longArray = doc["longs"].as<JsonArray>();

      for (JsonVariant v : longArray) {
        longs.push_back(v.as<double>());
      }
      for(double lon : longs){
        Serial.print("Long : ");
        Serial.println(lon, 8);
      }Serial.println("=======================");

  }
   if (String(topic) == "data/result") {
    // float distance = doc["distance"];  // Contoh key "heading"
    // double adjHeading = doc["adjHeading"];    // Contoh key "lat"
    // double adjAzm = doc["adjAzm"];   // Contoh key "lon"
    // float setpoint = doc["setpoint"];      // Contoh key "speed"
    // int counter = doc["counter"];
  
    motor = doc["motor"];
    speed = doc["speed"];
    
   }
}

void reconnect() {
  int count = 0;
  while (!client.connected()) {
    //if(count > 2) setup_wifi();
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected");
      client.subscribe("data/result");
      client.subscribe("data/addAngle");
      client.subscribe("data/setLats");
      client.subscribe("data/setLongs");
      client.subscribe("data/setCounter");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}

//////////////////////////////FUNGSI MOTOR//////////////////////////////
void motorMundur() {
  esc.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  Serial.println("Motor Mundur");
}

void motorStop() {
  esc.writeMicroseconds(1550);
  esc2.writeMicroseconds(1550);
  Serial.println("Motor Stop");
}

void motorMaju() {
  esc.writeMicroseconds(min(speed, 1700));
  esc2.writeMicroseconds(min(speed, 1700));
  Serial.println("Motor Maju");
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



void setup() {
  Serial.begin(9600);

  ss.begin(9600, SERIAL_8N1, 3, 1); // RX = GPIO16, TX = GPIO17

  compass.init();

  esc.attach(18);
  esc2.attach(19);

  rudderServo1.attach(25);
  rudderServo2.attach(26);
  rudderServo1.write(90);
  rudderServo2.write(90);

  Serial.print("gps scanning...");
  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
  delay(3000);
}

void loop() {
  
  while (ss.available() > 0 )
    if (gps.encode(ss.read()) )
      if (gps.location.isValid()) {
        if (!client.connected()) {
          reconnect();
        } client.loop();

        compass.read();
        Serial.println("Compass read");
        int azimuth = compass.getAzimuth();
        int adjustedAzimuth = (azimuth + 360)% 360;

        double lat1 = gps.location.lat();  // Latitude dari GPS
        double lon1 = gps.location.lng();  // Longitude dari GPS


        if(lats.size())lat2 = lats[counter];
        if(longs.size())long2 = longs[counter];

        double distance = calculateDistance(lat1, lon1, lat2, long2);
        float heading = calculateHeading(lat1, lon1, lat2, long2);
        int adjustedHeading = adjustedAzimuth - heading;
        if (adjustedHeading < 0) {
          adjustedHeading += 360.0;
        }

        const char* latDirection = gps.location.rawLat().negative ? "S" : "N";
        const char* lonDirection = gps.location.rawLng().negative ? "W" : "E";

        double rawLat = gps.location.rawLat().deg + gps.location.rawLat().billionths * 1e-9;
        double rawLng = gps.location.rawLng().deg + gps.location.rawLng().billionths * 1e-9;

        double speed = gps.speed.knots();

        StaticJsonDocument<200> doc;
        doc["lat"] = lat1;
        doc["latDirection"] = latDirection;
        doc["lon"] = lon1;
        doc["lonDirection"] = lonDirection;
        doc["adjAzimut"] = adjustedAzimuth;
        doc["adjHeading"] = adjustedHeading;
        doc["distance"] = distance;
        doc["counter"] = counter;
        doc["speed"] = speed;

       

        input = adjustedHeading;
        float error = setpoint - input;

        if (error > 180) {
          error -= 360;
        } else if (error < -180) {
          error += 360;
        }

        unsigned long current_time = millis();
        float dt = (current_time - previous_time) / 1000.0;

        integral += error * dt;
        float derivative = (error - previous_error) / dt;

        output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        previous_time = current_time;

        int servoAngle = 90 - output + addAngle;
        servoAngle = constrain(servoAngle, 0, 180);
        rudderServo1.write(servoAngle);
        rudderServo2.write(servoAngle);
        Serial.println(servoAngle);

        doc["servoAngle"] = servoAngle;
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);
        Serial.println("Preparing publish");
        client.publish("sensor/data", jsonBuffer);
        delay(100);

        if(distance < 2.5){
          counter += ( counter+1 < lats.size() ? 1 :  0);
        }
        Serial.println("After distance..");

        if (motor && counter < lats.size()) {
          motorMaju();
        } else {
          motorStop();
        }

      }
}
