#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h" 
#include "HardwareSerial.h"
#include <TinyGPS++.h>

#define ECHO_PIN 12 // Analog input that receives the echo signal
#define TRIG_PIN 13 // Digital output that sends the trigger signal
#define DHT_PIN 22     
#define DHT_TYPE DHT11 
#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32
#define REFRESH_RATE 5000 // Defined in miliseconds

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "ESPRESSIF";
const char *WIFI_PASSWORD = "Espressif123";
char macAddress[18];

const char *MQTT_BROKER_IP = "192.168.5.1";
const int MQTT_PORT = 1884;
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;

DHT dhtSensor(22, DHT11);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TinyGPSPlus gps; 
HardwareSerial SerialGPS(1);

static float latitud;
static float longitud;
static float temperature;
static float humidity; 
static float distance;

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT); // Connect the configured mqtt broker
  pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output

  dhtSensor.begin();

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker

  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART

  TimerHandle_t xTimer = xTimerCreate("printGpsReadings", REFRESH_RATE, pdTRUE, (void *) 0, printGpsReadings);
  xTimerStart(xTimer, 0);

}

void loop() {
  checkConnections(); // We check the connection every time

  // Publish every 2 seconds
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;
  if (elapsedTime >= 2000) {
    readValues();
    publishJson();   // Publishes a small json
    startTime = nowTime;
  }
 if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
}

void printGpsReadings(TimerHandle_t xTimer){
  
  latitud = gps.location.lat();
  longitud = gps.location.lng();

  Serial.print("LAT=");   Serial.println(String(latitud).c_str()); // Latitude in degrees (double)
  Serial.print("LONG=");  Serial.println(String(longitud).c_str()); // Longitude in degrees (double)
  Serial.print("SATS=");  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)

}

void readValues(){
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  distance = getDistance();  
  
}

void publishJson() {
  static const String topicStr = createTopic("camion");
  static const char *topic = topicStr.c_str();
   
  StaticJsonDocument<185> doc; // Create JSON document of 128 bytes
  char buffer[185]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT
  
 doc["device"] = "ESP32"; // Add names and values to the JSON document
  JsonObject sensor01 = 
      doc["sensores"].createNestedObject("DHT11"); // We can add another Object
  sensor01["t"] = temperature;
  sensor01["h"] = humidity;
  JsonObject sensor02 = 
      doc["sensores"].createNestedObject("Ultrasonido"); // We can add another Object 
  sensor02["d"] = distance;
  JsonObject sensor03 = 
      doc["sensores"].createNestedObject("GPS"); // We can add another Object 

  sensor03["lat"] = latitud;
  sensor03["lon"] = longitud;


  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

float getDistance() {
  digitalWrite(13, LOW); // Clear the TRIG_PIN by setting it LOW
  delayMicroseconds(5);

  // Trigger the sensor by setting the TRIG_PIN to HIGH for 10 microseconds
  digitalWrite(13, HIGH);
  delayMicroseconds(10);
  digitalWrite(13, LOW);

  long duration = pulseIn(12, HIGH); // pulseIn() returns the duration (length of the pulse) in microseconds

  return duration * 0.034 / 2; // Returns the distance in cm
}

String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
  return topicStr;
}

void connectToWiFiNetwork() {
  Serial.print(
      "Connecting with Wi-Fi: " +
      String(WIFI_SSID)); // Print the network which you want to connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".."); // Connecting effect
  }
  Serial.print("..connected!  (ip: "); // After being connected to a network,
                                       // our ESP32 should have a IP
  Serial.print(WiFi.localIP());
  Serial.println(")");
  String macAddressStr = WiFi.macAddress().c_str();
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker:" +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress, MQTT_USER, MQTT_PASSWORD);// Using unique mac address from ESP32
  while (!mqttClient.connected()) {
    delay(500);
    Serial.print("..");             // Connecting effect
    mqttClient.connect(macAddress); // Using unique mac address from ESP32
  }
  Serial.println("..connected! (ClientID: " + String(macAddress) + ")");
}

void checkConnections() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  } else { // Try to reconnect
    Serial.println("Connection has been lost with MQTT Broker");
    if (WiFi.status() != WL_CONNECTED) { // Check wifi connection
      Serial.println("Connection has been lost with Wi-Fi");
      connectToWiFiNetwork(); // Reconnect Wifi
    }
    connectToMqttBroker(); // Reconnect Server MQTT Broker
  }
}
