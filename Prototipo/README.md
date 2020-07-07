# PROTOTIPO

| | | | |
|-|-|-|-|
|[**Arduino**](#Arduino) |[**Azure**](#Azure) |[**Nodered**](#Nodered) |[**Raspberry**](#Raspberry)|
| | | | |


## [Arduino](/Arduino)
Sitio donde se colgaran todos los codigos que se hagan para los dispositivos que hay en el proyecto. Actualmente no son los códigos finales ya que aún falta reducir el mensaje json para que sea lo más eficiente posible.

Además, el código de LoraWAN solo se ha utilizado como prueba de comunicación con un gateway hecho por nosotros. Se ha puesto a modo de prueba ya que se utilizarán dispositivos y gateways industriales los sensores de la vaca.

### [Camion](/Arduino/camion)
**Código**
```cpp
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
const int MQTT_PORT = 1883;
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;

DHT dhtSensor(22, DHT11);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TinyGPSPlus gps; 
HardwareSerial SerialGPS(1);

static float LAT;
static float LONG;
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
    publishSmallJson();   // Publishes a small json
    publishGPS();
    startTime = nowTime;
  }
 if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
}

void printGpsReadings(TimerHandle_t xTimer){
  
  LAT = gps.location.lat();
  LONG = gps.location.lng();

  Serial.print("LAT=");   Serial.println(String(LAT).c_str()); // Latitude in degrees (double)
  Serial.print("LONG=");  Serial.println(String(LONG).c_str()); // Longitude in degrees (double)
  Serial.print("SATS=");  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)

}

void publishGPS() {

  static const String topicGPS = createTopic("GPS");
  static const char *topic = topicGPS.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes

  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT
  doc["lat"] = LAT;
  doc["long"] = LONG;

  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void readValues(){
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  distance = getDistance();  
  
}

void publishSmallJson() {
  static const String topicStr = createTopic("small_json");
  static const char *topic = topicStr.c_str();
   
  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT
  
  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  doc["sensor1"] = "Ultrasonido";
  JsonObject values1 = doc.createNestedObject("values1"); // We can add another Object
  values1["t"] = temperature;
  values1["h"] = humidity;  
  values1["d"] = distance;

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
```
[[Go to top]](#Prototipo)

### [Tanque](/Arduino/tanque)
**Código**
```cpp
#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h"
 
#define ECHO_PIN 12 // Analog input that receives the echo signal
#define TRIG_PIN 13 // Digital output that sends the trigger signal
#define DHT_PIN 22     
#define DHT_TYPE DHT11 

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "ESPRESSIF";
const char *WIFI_PASSWORD = "Espressif123";
char macAddress[18];

const char *MQTT_BROKER_IP = "192.168.5.1";
const int MQTT_PORT = 1884;
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DHT dhtSensor(21, DHT11);

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
    publishSmallJson();   // Publishes a small json
    startTime = nowTime;
  }
 
}

void readValues(){
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  distance = getDistance();  
  
}


void publishSmallJson() {
  static const String topicStr = createTopic("small_json");
  static const char *topic = topicStr.c_str();
   
  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT
  
  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  doc["sensor1"] = "Ultrasonido";
  JsonObject values1 = doc.createNestedObject("values1"); // We can add another Object
  values1["t"] = temperature;
  values1["h"] = humidity;  
  values1["d"] = distance;

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
```
[[Go to top]](#Prototipo)

### [Puerta](/Arduino/puerta)
**Código**
```cpp
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>

#define LED_PIN 25

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "AulaAutomatica";
const char *WIFI_PASSWORD = "ticsFcim";
char macAddress[18];

// Add MQTT Broker settings
const char *MQTT_BROKER_IP = "10.20.60.5";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;
const int QoS = 0; // Quality of Service for the subscriptions

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  pinMode(LED_PIN, OUTPUT); // Pinout as output
  
  mqttClient.setServer(MQTT_BROKER_IP,MQTT_PORT); // Connect the configured mqtt broker
  mqttClient.setCallback(
      callback); // Prepare what to do when a message is recieved

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker
  setSubscriptions();     // Subscribe defined topics
}

void loop() {
  checkConnections(); // We check the connection every time
}

/* Additional functions */
void setSubscriptions() {
  //subscribe("GPS");
  subscribe("LED");
}

void subscribe(char *newTopic) {
  const String topicStr = createTopic(newTopic);
  const char *topic = topicStr.c_str();
  mqttClient.subscribe(topic, QoS);
  Serial.println("Client MQTT subscribed to topic: " + topicStr +
                 " (QoS:" + String(QoS) + ")");
}

void callback(char *topic, byte *payload, unsigned int length) {
  // Register all subscription topics
  //static const String gpsTopicStr = createTopic("GPS");
  static const String ledTopic = createTopic("LED");

  String msg = unwrapMessage(payload, length);
  Serial.println(" => " + String(topic) + ": " + msg);

  // What to do in each topic case?
  if (String(topic) == ledTopic) {
    if (msg == "1"){
      digitalWrite(LED_PIN, HIGH); // Set pin to HIGH
    }
    else{
      digitalWrite(LED_PIN, LOW); // Set pin to HIGH
    }
    
  } else {
    Serial.println("[WARN] - '" + String(topic) +
                   "' topic was correctly subscribed but not defined in the "
                   "callback function");
  }
}

String unwrapMessage(byte *message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) { // Unwraps the string message
    msg += (char)message[i];
  }
  return msg;
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
  //String macAddressStr = "24:6F:28:87:F0:88";
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker: " +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress); //(macAdress,MQTT_USER, MQTT_PASSWORD); // Using unique mac address from ESP32
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
    setSubscriptions();    // Subscribes to configured topics
  }
}
```
[[Go to top]](#Prototipo)

### [LoraWanABP](/Arduino/lorawanABP)
**Libreria** -> [arduino-lmic](https://github.com/matthijskooijman/arduino-lmic)

**Código**
```cpp
#include <lmic.h> //Libreria arduino-lmic-master.zip de 
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x14, 0xBF, 0x29, 0xC0, 0xEE, 0xF2, 0xA8, 0x95, 0x93, 0x7D, 0x74, 0x63, 0xC4, 0x66, 0x83, 0x84 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x54, 0xF6, 0xB2, 0xD6, 0xCD, 0xF3, 0x92, 0x94, 0x8F, 0xDC, 0xB9, 0xC9, 0x79, 0xAD, 0xAB, 0x49 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011853 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
```
[[Go to top]](#Prototipo)

## [Azure](/Azure)
Documentación importante sobre Azure. Actualmente información sobre la creación de contenedores persistentes y/o máquinas virtuales.


## [Nodered](/Nodered)
Dashboards i base de datos entregados hasta la fecha de _ESPRESSIF_ a _RASP_.


## [Raspberry](/Raspberry)
Documentación i códigos para la configuración de las raspberrys como gateways i access point. (**Actualmente vacío**)

[[Go to top]](#Prototipo)
