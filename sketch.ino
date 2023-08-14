#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include "PubSubClient.h"

#define P_BomNuoc 27
#define P_PhunSuong 26
#define P_NhietDoKK 25
#define P_DoAmDat 33
#define P_Servo 13
#define P_DenSuoi 32
#define P_AnhSang 12
#define P_Mua 34

#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4

#define dirPin 15
#define stepPin 2
#define READ_INTERVAL 500
#define FULL_OPEN 600
#define FULL_CLOSE -600
// Define motor interface type
#define motorInterfaceType 1
//define mqtt
const char* mqttServer = "test.mosquitto.org";
int port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);
//define wifi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
LiquidCrystal_I2C LCD(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
DHTesp dhtSensor;

//global variables
uint64_t curTime = 0, lastTime = 0;

struct SensorsData {
  TempAndHumidity temAndHumidity;
  int moiser;
  int light;
  int rain;

  SensorsData(){
    temAndHumidity.temperature = 0;
    temAndHumidity.humidity = 0;
    moiser = 0;
    light = 0;
    rain = 0;
  }
};

SensorsData lastData,curData;

// setup wifi
void wifiConnect(){
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
}

//from mqtt to ESP32
void mqttReconnect(){
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    if(client.connect("21127608")){
      Serial.println("connected");
    }
    else{
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Serial.print("Connecting to Wifi");
  wifiConnect();

  //connect mqtt
  client.setServer(mqttServer,port);

  LCD.init();
  LCD.backlight();
  LCD.setCursor(0, 0);
  LCD.print("Hello world!");
  // servo.attach(P_Servo, 500, 2400);
  dhtSensor.setup(P_NhietDoKK, DHTesp::DHT22);

  myStepper.setMaxSpeed(1000);
	myStepper.setAcceleration(125);
	myStepper.setSpeed(1000);
	myStepper.moveTo(2000);

  pinMode(P_NhietDoKK,INPUT);
  pinMode(P_DoAmDat, INPUT);
  pinMode(P_BomNuoc, OUTPUT);
  pinMode(P_PhunSuong, OUTPUT);
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Nhiet do:");


}

void loop() {
  curTime = millis();
  if (curTime - lastTime > READ_INTERVAL) {
    //read Sensor data
    curData.temAndHumidity = dhtSensor.getTempAndHumidity();
    curData.moiser = analogRead(P_DoAmDat);
    curData.light = analogRead(P_AnhSang);
    curData.rain = digitalRead(P_Mua);
  }

  if (true) {
    lastData = curData;

    LCD.print(lastData.temAndHumidity.temperature);
  }
}

/*
  if (millis() - timerUpdate >= DELAY_UPDATE) {
    timerUpdate = millis();
    TempAndHumidity  data = dhtSensor.getTempAndHumidity();
    Serial.println("\x1b[2J\x1b[;H");
    Serial.printf(" Temperature: %.2f %s",  data.temperature, "°C");
    Serial.printf("    Humidity: %.1f %s", data.humidity, "%");
  }

    // if (myStepper.distanceToGo() == 0) 
	// 	myStepper.moveTo(-myStepper.currentPosition());

	// // Move the motor one step
	// myStepper.run();
  // myStepper.run();
*/