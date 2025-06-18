#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// WiFi Credentials
#define WIFI_SSID "POCO M5"
#define WIFI_PASSWORD "bana@SJ17"

// Firebase Config
#define API_KEY "AIzaSyBDbSS56FFKvQEqAQxmIx6Xv38v9ydLEcA"
#define DATABASE_URL "https://plant-monitor-data-default-rtdb.firebaseio.com/"

// Firebase setup
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

    FirebaseJson json;

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// Motor control pins
#define IN1 23
#define IN2 22
#define IN3 21
#define IN4 19
#define ENA 18


// LEDC channels for PWM
#define CHANNEL_A 0
#define FREQ 1000
#define RESOLUTION 8 // 8-bit = 0 to 255
int speedVal = 180; // Default speed

int trig = 17;
int echo = 16;
int Find = 0;
long int duration, distance;

int IR1 = 39;
int IR2 = 34;

int soil = 36;
int pump = 5;
int servo = 35;
int buzzer = 33;

int moisture=0;
String plant_st = "";
String pump_st = "";

#include <time.h>

// NTP Server and timezone config
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;  // IST = UTC +5:30 = 19800 seconds
const int   daylightOffset_sec = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_RC_Car");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);

  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);

  pinMode(pump,OUTPUT);
   pinMode(buzzer,OUTPUT);

  // PWM setup
  ledcSetup(CHANNEL_A, FREQ, RESOLUTION);
  ledcAttachPin(ENA, CHANNEL_A);

  stopCar();
  beep(3,100);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    beep(1,10);
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
Serial.println("Time synchronized.");

  Serial.println("\nWiFi Connected!");
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = "technkit17@gmail.com";
  auth.user.password = "123456789";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.println(command);
    beep(1,10);
    object();
   
    handleCommand(command);

  }
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F':  if(Find ==1){ 
      Serial.println("Object find in fornt of car"); beep(2,500); stopCar(); break;
          }
    else{
      moveForward(); break;
    }
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopCar(); break;
    case '1': speedVal = 100; break;  // Low speed
    case '2': speedVal = 180; break;  // Medium speed
    case '3': speedVal = 255; break;  // Max speed
    case 'H': Hcheck(); Mtest(); break;
    case 'D': send(); break;
    
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setSpeed();
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setSpeed();
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setSpeed();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setSpeed();
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(CHANNEL_A, 0);

}

void setSpeed() {
  ledcWrite(CHANNEL_A, speedVal);
}


void object(){
   digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  duration= pulseIn(echo,HIGH);
  distance= (duration/2)/29;
 if (distance<30){
   Find=1;
 }
  else{
  Find=0;
  }
}

void Hcheck(){
  beep(1,100);
  if(digitalRead(IR1) ==1  && digitalRead(IR2) ==1){
    plant_st ="NO Plant detected...";
   // Serial.println("NO Plant detected...");
  }
  else if(digitalRead(IR1) ==0  && digitalRead(IR2) ==1){
    plant_st ="No Growth";
   // Serial.println("NO Growth");
  }

  else if(digitalRead(IR1) ==1  && digitalRead(IR2) ==0){
    plant_st ="Error!!! Something Wrong.";
    //Serial.println("Error!!! Something Wrong.");
  }

  else if(digitalRead(IR1) ==0  && digitalRead(IR2) ==0){
    plant_st ="OK. Plant Grown Normally.";
   // Serial.println("OK. Plant Grown Normally.");
  }
  Serial.println(plant_st);
}


void Mtest(){
   beep(2,100);
   moisture = 100- map(analogRead(soil),0,4096,0,100);
   Serial.print("Soil moisture: "); Serial.print(moisture);Serial.println("%");
 if(moisture<50){
  Serial.println("pump On");
  digitalWrite(pump,1);
  pump_st ="ON";
  delay(2000);
  digitalWrite(pump,0);
  Serial.println("pump Off");
 }
 else{
  Serial.println("pump Off");
  digitalWrite(pump,0);
   pump_st ="OFF";

 }
}

void beep(int k, int t){
  for(int i=0;i<k;i++){
    digitalWrite(buzzer,1); delay(t);
     digitalWrite(buzzer,0); delay(t);
  }
}

void send() {
    Hcheck();
    Mtest();

    FirebaseJson logData;
    logData.set("Plant_status", plant_st); 
    logData.set("Soil_Moisture", moisture); 
    logData.set("Pump", pump_st);

    String timestamp = getTimeStamp();

    // Replace characters that Firebase path doesn't allow
    timestamp.replace(":", "-");
    timestamp.replace(" ", "_");

    String path = "/Rover/Logs/" + timestamp;

    if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &logData)) {
        Serial.println("Data pushed to log with timestamp as key!");
        Serial.println("Path: " + path);
    } else {
        Serial.println("Error: " + fbdo.errorReason());
    }

    delay(1000);
}



String getTimeStamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Failed to get time";
  }
  char timeString[30];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}
