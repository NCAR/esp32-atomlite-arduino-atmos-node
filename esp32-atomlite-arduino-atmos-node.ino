/*
    AtmosNode.ino

    Atmospheric measurement node with
    the Grove bme280, bme680 and veml6070.

    ===
    This code sleeps for 60s, wakes up, takes a measurement,
    transmits and then goes back to sleep.

    copyright (c) 2020 keith maull
    Website    :
    Author     : kmaull-ucar
    Create Time:
    Change Log :
*/
#include <WiFi.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include "IoTwx.h"          /// https://github.com/ncar/esp32-atomlite-arduino-iotwx

#define BMEX80_IIC_ADDR   uint8_t(0x76)
#define SEALEVELPRESSURE_HPA (1013.25)

IoTwx            node;
Adafruit_BME680  bme680;  
unsigned long    last_millis       = 0;
unsigned long    start_millis      = 0;
bool             bme680_attached   = false;
char*            sensor;
char*            topic;
int              timezone;
int              reset_interval;
int              publish_interval;
int              max_frequency     = 80;


void publish_bme680_measurements() {
  char s[strlen(sensor)+64];

  if (!bme680.performReading()) {
    Serial.println("[FAIL] Failed to perform BME680 reading.");
    blink_led(LED_FAIL, LED_FAST);
    return;
  }
  
  strcpy(s, sensor); strcat(s, "/bme680/temperature");
  node.publishMQTTMeasurement(topic, s, bme680.temperature, 0);
  
  strcpy(s, sensor); strcat(s, "/bme680/pressure");
  node.publishMQTTMeasurement(topic, s, bme680.pressure, 0);

  strcpy(s, sensor); strcat(s, "/bme680/humidity");
  node.publishMQTTMeasurement(topic, s, bme680.humidity, 0);

  strcpy(s, sensor); strcat(s, "/bme680/voc");
  node.publishMQTTMeasurement(topic, s, bme680.gas_resistance, 0);

  strcpy(s, sensor); strcat(s, "/bme680/altitude");
  node.publishMQTTMeasurement(topic, s, bme680.readAltitude(SEALEVELPRESSURE_HPA), 0);

  delay(2000);
}


void setup() {
  File                      file;
  StaticJsonDocument<1024>  doc;
  char                      uuid[32];
  bool                      i2c_device_connected = false;
  String                    mac = String((uint32_t)ESP.getEfuseMac(), HEX);

  strcpy(uuid, "ESP32P_AtomLite_"); strcat(uuid, (const char*) mac.c_str());

  Serial.begin(57600);
  Serial.println("[info] This is the IoTwx v2.0.0.1."); delay(500);
  Serial.println("[info] initializing now ...");

  start_millis              = millis();
  init_led(); // set up AtomLite LED

  node = IoTwx(wait_for_bluetooth_config(uuid, millis(), 1)); // initializes config.json
  if (node.isConfigured())
  {
    // initialize the I2C bus
    Wire.begin(26, 32, 10000);

    // check for i2c device connectivity
    while(!i2c_device_connected) {
      if (!bme680.begin()) {
        Serial.println("[warn]: Could not find Adafruit BME680 sensor. Check your connections and verify the address 0x76 is correct.");
        blink_led(LED_FAIL, LED_FAST);  
      } else {
        bme680_attached = true;
        i2c_device_connected = true;
        
        Serial.println("[info]: OK Found Adafruit BME680");
        blink_led(LED_OK, LED_SLOW);

        // Set up oversampling and filter initialization
        bme680.setTemperatureOversampling(BME680_OS_8X);
        bme680.setHumidityOversampling(BME680_OS_2X);
        bme680.setPressureOversampling(BME680_OS_4X);
        bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme680.setGasHeater(320, 150);  // 320*C for 150 ms
      }
    }
    delay(1000);


    Serial.println("[info]: deserializing to JSON");
    file = SPIFFS.open("/config.json", FILE_READ);
    deserializeJson(doc, file);
    file.close();

    Serial.println("[info]: reading from JSON doc (SPIFFS)");
    timezone         = atoi((const char*)doc["iotwx_timezone"]);
    sensor           = strdup((const char*)doc["iotwx_sensor"]);
    topic            = strdup((const char*)doc["iotwx_topic"]);
    reset_interval   = 1000*60*atoi((const char*)doc["iotwx_reset_interval"]);
    publish_interval = atoi((const char*)doc["iotwx_publish_interval"]);
    max_frequency    = atoi((const char*)doc["iotwx_max_frequency"]);

    // begin shutdown sequence, downthrottle, shutdown wifi and BT
    btStop(); Serial.println("[info]: BT disconnected for power reduction");
    setCpuFrequencyMhz(max_frequency); Serial.println(); Serial.print("[info] CPU downthrottled to "); Serial.print(max_frequency); Serial.println("Mhz for power reduction");
    WiFi.mode(WIFI_OFF); Serial.println("[info] Wifi shut off for power reduction");
  } else
      Serial.println("[info]: halting > configuration corrupt");
}


void loop() {
  if (millis() - last_millis > publish_interval * 60 * 1000) {
    Serial.println("[info]: measuring");

    last_millis = millis();

    node.establishCommunications();
    if (bme680_attached) publish_bme680_measurements();

    // Configure the timer to wake us up!
    delay(1000);
  }

  if ( millis() - start_millis > reset_interval ) esp_restart();

  Serial.println("[info]: sleeping");
  esp_sleep_enable_timer_wakeup(publish_interval * 60L * 1000000L);
  esp_light_sleep_start();
}
