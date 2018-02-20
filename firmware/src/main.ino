/*
  Copyright (c) 2017 @KmanOz

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "config_sc.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SCL_PIN       2
#define SDA_PIN       0
#define PCA_OE_PIN    4

#define HOST_PREFIX   "Ilumi_%s"
#define HEADER        "\n\n---------------------  CrespumIlumi.beta  -------------------"
#define VER           "ilumi.beta"

bool OTAupdate = false;
bool sendStatus = false;
bool requestRestart = false;
bool ringStatusOn = false;
int ledsOn = 0;
char ESP_CHIP_ID[8];
char UID[16];
int lastRelayState;
long rssi;
unsigned long TTasks1;
unsigned long count = 0;
extern "C" {
  #include "user_interface.h"
}
WiFiClient wifiClient;
TwoWire i2c = TwoWire();
PubSubClient mqttClient(wifiClient, MQTT_SERVER, MQTT_PORT);
Adafruit_PWMServoDriver leds = Adafruit_PWMServoDriver(&i2c);

void turnOffRing() {
  for (uint8_t led=0; led<16; led++)
    leds.setPWM(led, 4096, 0); // turns LED fully OFF
}

void turnOnRing() {
  int led;
  for (led=0 ; led<ledsOn ; led++)
    leds.setPWM(led, 0, 4096); // turns LED fully ON
  for (led ; led<16 ; led++)
    leds.setPWM(led, 4096, 0); // turns LED fully OFF
}

void loadingPatternRing() {
  for (uint8_t led=0; led<16; led++) {
    leds.setPWM(led, 0, 4096); // turns LED fully ON
    delay(50);
    leds.setPWM(led, 4096, 0); // turns LED fully OFF
  }
}

void onMQTTMsg(const MQTT::Publish& pub) {
  Serial.println(pub.topic());
  Serial.println(pub.payload_string());
  if (pub.topic().endsWith("set/switch")) {
    if (pub.payload_string() == "on") {
      ringStatusOn = true;
      turnOnRing();
    } else if (pub.payload_string() == "off") {
      ringStatusOn = false;
      turnOffRing();
    }
  }
  else if (pub.topic().endsWith("set/brightness")) {
    int brightness = pub.payload_string().toInt();
    ledsOn = 16.0/255*brightness;
    if (ledsOn > 0) ringStatusOn = true;
    else ringStatusOn = false;
    turnOnRing();
  }
  else if (pub.topic() == "reset") {
    requestRestart = true;
  }
  sendStatus = true;
}

void configureOTA() {
  ArduinoOTA.setHostname(UID);

  ArduinoOTA.onStart([]() {
    OTAupdate = true;
    loadingPatternRing();
    leds.setPWM(0, 4096, 0);
    Serial.println("OTA Update Initiated . . .");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Ended . . .s");
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    leds.setPWM(0, 0, 4096);
    delay(5);
    leds.setPWM(0, 4096, 0);
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    loadingPatternRing();
    OTAupdate = false;
    Serial.printf("OTA Error [%u] ", error);
    if (error == OTA_AUTH_ERROR) Serial.println(". . . . . . . . . . . . . . . Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println(". . . . . . . . . . . . . . . Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println(". . . . . . . . . . . . . . . Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println(". . . . . . . . . . . . . . . Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println(". . . . . . . . . . . . . . . End Failed");
  });

  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);

  // Setup LEDs ring driver
  pinMode(PCA_OE_PIN, OUTPUT);
  digitalWrite(PCA_OE_PIN, LOW);
  i2c.begin(SDA_PIN, SCL_PIN);
  leds.begin();
  leds.setPWMFreq(1000);  // Set to whatever you like, we don't use it in this demo!
  turnOffRing();

  sprintf(ESP_CHIP_ID, "%06X", ESP.getChipId());
  sprintf(UID, HOST_PREFIX, ESP_CHIP_ID);
  mqttClient.set_callback(onMQTTMsg);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(UID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  configureOTA();

  Serial.println(HEADER);
  Serial.print("\nUID: ");
  Serial.print(UID);
  Serial.print("\nConnecting to "); Serial.print(WIFI_SSID); Serial.print(" Wifi");

  while ((WiFi.status() != WL_CONNECTED) && kRetries --) {
    delay(500);
    Serial.print(" .");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" DONE");
    Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
    Serial.print("Connecting to ");Serial.print(MQTT_SERVER);Serial.print(" Broker . .");
    delay(500);

    while (!mqttClient.connect(MQTT::Connect(UID).set_keepalive(90).set_auth(MQTT_USER, MQTT_PASS)) && kRetries --) {
      Serial.print(" .");
      delay(1000);
    }

    if(mqttClient.connected()) {
      Serial.println(" DONE");
      Serial.println("\n----------------------------  Logs  ----------------------------");
      Serial.println();
      mqttClient.subscribe(MQTT_TOPIC"/set/#");
      loadingPatternRing();
    } else {
      Serial.println(" FAILED!");
      Serial.println("\n----------------------------------------------------------------");
      Serial.println();
    }
  } else {
    Serial.println(" WiFi FAILED!");
    Serial.println("\n----------------------------------------------------------------");
    Serial.println();
  }
}

void loop() {
  ArduinoOTA.handle();
  if (OTAupdate == false) {
    mqttClient.loop();
    timedTasks1();
    checkStatus();
  }
}

void checkConnection() {
  if (WiFi.status() == WL_CONNECTED)  {
    if (mqttClient.connected()) {
      Serial.println("mqtt broker connection . . . . . . . . . . OK");
    } else {
      Serial.println("mqtt broker connection . . . . . . . . . . LOST");
      requestRestart = true;
    }
  } else {
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    requestRestart = true;
  }
}

void checkStatus() {
  if (sendStatus) {
    if (ringStatusOn) {
      if (kRetain == 0) {
        mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/status", "on").set_qos(QOS));
      } else {
        mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/status", "on").set_retain().set_qos(QOS));
      }

      Serial.println("Ilumi . . . . . . . . . . . . . . . . . . ON");
    } else {
      if (kRetain == 0) {
        mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/status", "off").set_qos(QOS));
      } else {
        mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/status", "off").set_retain().set_qos(QOS));
      }

      Serial.println("Ilumi . . . . . . . . . . . . . . . . . . OFF");
    }

    sendStatus = false;
  }

  if (requestRestart) {
    loadingPatternRing();
    ESP.restart();
  }
}

void doReport() {
  rssi = WiFi.RSSI();
  char message_buff[120];
  String pubString = "{\"UID\": "+String(UID)+", "+"\"WiFi RSSI\": "+String(rssi)+"dBM"+", "+"\"Topic\": "+String(MQTT_TOPIC)+", "+"\"Ver\": "+String(VER)+"}";
  pubString.toCharArray(message_buff, pubString.length()+1);
  if (kRetain == 0) {
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug", message_buff).set_qos(QOS));
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/heartbeat", "OK").set_qos(QOS));
  } else {
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug", message_buff).set_retain().set_qos(QOS));
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/heartbeat", "OK").set_retain().set_qos(QOS));
  }
}

void timedTasks1() {
  if ((millis() > TTasks1 + (kUpdFreq*60000)) || (millis() < TTasks1)) {
    TTasks1 = millis();
    checkConnection();
    doReport();
  }
}
