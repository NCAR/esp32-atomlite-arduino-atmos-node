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
#include "Seeed_VEML6070.h" /// https://github.com/Seeed-Studio/Seeed_VEML6070 
#include "Seeed_BME280.h"   /// https://github.com/seeedstudio/Grove_BME280
#include "seeed_bme680.h"   /// https://github.com/seeedstudio/Seeed_BME680
#include "IoTwx.h"         /// https://github.com/iotwx

#define BMEX80_IIC_ADDR   uint8_t(0x76)
#define VEMLXX70_IIC_ADDR uint8_t(0x45) // TODO: fix this!

IoTwx         node;
Seeed_BME680  bme680(BMEX80_IIC_ADDR);
BME280        sensor_bme280;
VEML6070      sensor_veml6070;
unsigned long last_millis       = 0;
unsigned long start_millis      = 0;
bool          bme680_attached   = false;
bool          bme280_attached   = false;
bool          veml6070_attached = false;
char*         sensor;
char*         topic; 
int           timezone;    
int           reset_interval; 
int           publish_interval; 
int           max_frequency     = 80;    


void publish_bme280_measurements() {   
  float humidity    = sensor_bme280.getHumidity();
  float temperature = sensor_bme280.getTemperature();
  float pressure    = sensor_bme280.getPressure();
  float altitude    = sensor_bme280.calcAltitude(pressure);
  char  s[strlen(sensor)+64];

  strcpy(s, sensor); strcat(s, "/bme280/humidity");
  node.publishMQTTMeasurement(topic, s, humidity, 0);

  strcpy(s, sensor); strcat(s, "/bme280/temperature");
  node.publishMQTTMeasurement(topic, s, temperature, 0);

  strcpy(s, sensor); strcat(s, "/bme280/pressure");
  node.publishMQTTMeasurement(topic, s, pressure, 0);

  strcpy(s, sensor); strcat(s, "/bme280/altitude");
  node.publishMQTTMeasurement(topic, s, altitude, 0);
}


void publish_bme680_measurements() {  
  if (bme680.read_sensor_data()) {
    blink_led(LED_FAIL, LED_FAST);
    return;
  } else {
    // read the sensor data
    float humidity    = bme680.sensor_result_value.humidity;
    float temperature = bme680.sensor_result_value.temperature;
    float pressure    = bme680.sensor_result_value.pressure;
    float gas         = bme680.sensor_result_value.gas / 1000.0;
    char s[strlen(sensor)+64];

    strcpy(s, sensor); strcat(s, "/bme680/humidity");
    node.publishMQTTMeasurement(topic, s, humidity, 0); 

    strcpy(s, sensor); strcat(s, "/bme680/temperature");
    node.publishMQTTMeasurement(topic, s, temperature, 0); 
    
    strcpy(s, sensor); strcat(s, "/bme680/pressure");
    node.publishMQTTMeasurement(topic, s, pressure, 0); 

    strcpy(s, sensor); strcat(s, "/bme680/gas");
    node.publishMQTTMeasurement(topic, s, gas, 0); 
  }
}


err_t publish_veml6070_measurements() {
    err_t ret = NO_ERROR;
    u16   step;

    sensor_veml6070.wait_for_ready();
    CHECK_RESULT(ret, sensor_veml6070.read_step(step));
    
    char s[strlen(sensor)+64];

    strcpy(s, sensor); strcat(s, "/veml6070/uv");
    node.publishMQTTMeasurement(topic, s, step, 0);
    
    return ret;
}


void setup() {
  File                      file;
  StaticJsonDocument<1024>  doc;
  char                      uuid[32];
  bool                      i2c_device_connected = false;
  String                    mac = String((uint32_t)ESP.getEfuseMac(), HEX);

  strcpy(uuid, "ESP32P_AtomLite_"); strcat(uuid, (const char*) mac.c_str());
  
  Serial.begin(57600);
  Serial.println("[] This is the IoTwx AtmosNode.  Initializing ...");
  
  start_millis              = millis();
  init_led(); // set up AtomLite LED

  node = IoTwx(wait_for_bluetooth_config(uuid, millis(), 90)); // initializes config.json
  if (node.isConfigured())
  {  
    // initialize the I2C bus
    Wire.begin(26, 32, 10000);
  
    // check for i2c device connectivity
    while(!i2c_device_connected) {
      // check for BMEx80 device
      Wire.beginTransmission((uint8_t)BMEX80_IIC_ADDR);
      if (Wire.endTransmission() != 0) {
        Serial.println("[]: BME sensor did not ACK; check your connections.");
        blink_led(LED_FAIL, LED_FAST);
        delay(5000);
      } else i2c_device_connected = true;
  
      // check for vemlxx70 device
      Wire.beginTransmission((uint8_t)VEMLXX70_IIC_ADDR);
      if (Wire.endTransmission() != 0) {
        Serial.println("[warn]: VEML sensor did not ACK; check your connections.");
        blink_led(LED_FAIL, LED_FAST);
        delay(5000);
      } else i2c_device_connected = true;
    }
    
    // initiliaze the BME680 sensor
    if (!bme680.init()) {
      Serial.println("[error]: Device BME680 initialization failed.");    
      blink_led(LED_FAIL, LED_FAST);
    }
    else {
      Serial.println("[]: Device BME680 initialization OK");
      blink_led(LED_OK, LED_SLOW);
      bme680_attached = true;
    }
  
    // NOTE: BUG in init() still returns 1 when the device is not connected!
    if (bme680_attached || !sensor_bme280.init()) {
      Serial.println("[error]: Device BME280 initialization failed.");
      blink_led(LED_FAIL, LED_FAST);
    }
    else {
      // NOTE: necessary to deal with the bug, ugh!
      Wire.beginTransmission((uint8_t)BMEX80_IIC_ADDR);
      if (Wire.endTransmission() != 0) {
        Serial.println("[error]: Device BME280 initialization failed.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        Serial.println("[info]: Device BME280 initialization OK");
        //blink_led(LED_OK, LED_SLOW);
        bme280_attached = true;    
      }
    }
  
    delay(1000);
    if (sensor_veml6070.init() != -2) {
      Serial.println("[error]: Device VEML6070 initialization failed.");
      blink_led(LED_FAIL, LED_FAST);
    }
    else {
      Serial.println("[]: Device VEML6070 initialization OK");
      blink_led(LED_OK, LED_SLOW);
      veml6070_attached = true;    
    }

    Serial.println("[] deserializing to JSON");
    file = SPIFFS.open("/config.json", FILE_READ);
    deserializeJson(doc, file);
    file.close();

    Serial.println("[] reading from JSON doc (SPIFFS)");
    timezone         = atoi((const char*)doc["iotwx_timezone"]);
    sensor           = strdup((const char*)doc["iotwx_sensor"]);
    topic            = strdup((const char*)doc["iotwx_topic"]);  
    reset_interval   = 1000*60*atoi((const char*)doc["iotwx_reset_interval"]);  
    publish_interval = atoi((const char*)doc["iotwx_publish_interval"]);  
    max_frequency    = atoi((const char*)doc["iotwx_max_frequency"]);

    // begin shutdown sequence, downthrottle, shutdown wifi and BT
    btStop(); Serial.println("[] BT disconnected for power reduction");  
    setCpuFrequencyMhz(max_frequency); Serial.println(); Serial.print("[] CPU downthrottled to "); Serial.print(max_frequency); Serial.println("Mhz for power reduction");  
    WiFi.mode(WIFI_OFF); Serial.println("[] Wifi shut off for power reduction");  
  } else 
      Serial.println("[] halting: configuration corrupt");
}


void loop() {
  if (millis() - last_millis > publish_interval * 60 * 1000) {
    Serial.println("[info]: measuring");

    last_millis = millis();
    
    node.establishCommunications();
    if (bme680_attached) publish_bme680_measurements();
    if (bme280_attached) publish_bme280_measurements();
    // if (veml6070_attached) publish_veml6070_measurements();
        
    // Configure the timer to wake us up!
    delay(1000);
  }

  if ( millis() - start_millis > reset_interval ) esp_restart(); 

  Serial.println("[info]: sleeping");
  esp_sleep_enable_timer_wakeup(publish_interval * 60L * 1000000L);
  esp_light_sleep_start();     
}
