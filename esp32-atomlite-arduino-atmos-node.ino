/*
    AtmosNode.ino

    Atmospheric measurement node with
    the Adafruit chips:

      bme680   (thpvoc)
      pmsa003i (air quality)
      scd4x    (true co2)
      ltr390   (uva+b)
      sht40    (th)
    
    ===
    This code sleeps for 60s, wakes up, takes a measurement,
    transmits and then goes back to sleep.

    copyright (c) 2020-2023 keith maull
    Website    :
    Author     : kmaull-ucar
    Create Time:
    Change Log :
      5/26 == removed seeed/grove code, inserted qwiic/adafruit

*/
#include <WiFi.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_PM25AQI.h"
#include "Adafruit_LTR390.h"
#include <SensirionI2CScd4x.h>'
#include "Adafruit_SHT4x.h"
#include "IoTwx.h"          /// https://github.com/ncar/esp32-atomlite-arduino-iotwx

#define BMEX80_IIC_ADDR   uint8_t(0x76)
#define SEALEVELPRESSURE_HPA (1013.25)

IoTwx             node;
Adafruit_BME680   bme680;
Adafruit_PM25AQI  aqi = Adafruit_PM25AQI();
SensirionI2CScd4x scd4x;
Adafruit_LTR390   ltr = Adafruit_LTR390();
Adafruit_SHT4x    sht4 = Adafruit_SHT4x();

unsigned long    last_millis       = 0;
unsigned long    start_millis      = 0;

bool             bme680_attached   = false;
bool             pm25aqi_attached  = false;
bool             scd4x_attached    = false;
bool             ltr390_attached   = false;
bool             sht4x_attached    = false;

char*            sensor;
char*            topic;
int              timezone;
int              reset_interval;  
int              publish_interval;
int              max_frequency     = 80;


void publish_ltr390_measurements() {
  char s[strlen(sensor) + 64];

  if (ltr.newDataAvailable()) {
    strcpy(s, sensor); strcat(s, "/ltr390/uvs");
    node.publishMQTTMeasurement(topic, s, ltr.readUVS(), 0);
  }

}


void publish_scd4x_measurements() {
  char s[strlen(sensor) + 64];
  uint16_t co2;
  float temperature;
  float humidity;
  uint16_t scd4x_error;

  scd4x.begin(Wire);
  delay(5000);    
  scd4x_error = scd4x.readMeasurement(co2, temperature, humidity);
  
  if (scd4x_error) {
      Serial.print("[error]: scd4x error trying to execute readMeasurement(): ");
  } else if (co2 == 0) {
      Serial.println("[warn]: scd4x invalid sample detected, skipping.");
  } else {
      strcpy(s, sensor); strcat(s, "/scd4x/co2");
      node.publishMQTTMeasurement(topic, s, co2, 0);
      
      strcpy(s, sensor); strcat(s, "/scd4x/temperature");
      node.publishMQTTMeasurement(topic, s, temperature, 0);
      
      strcpy(s, sensor); strcat(s, "/scd4x/humidity");
      node.publishMQTTMeasurement(topic, s, humidity, 0);
  }
}


void publish_sht4x_measurements() {
  char s[strlen(sensor) + 64];
  sensors_event_t humidity, temp;

  delay(1500);
  
  sht4.getEvent(&humidity, &temp);

  strcpy(s, sensor); strcat(s, "/sht4x/temperature");
  node.publishMQTTMeasurement(topic, s, temp.temperature, 0);
  
  strcpy(s, sensor); strcat(s, "/sht4x/humidity");
  node.publishMQTTMeasurement(topic, s, humidity.relative_humidity, 0);
}


void publish_pmsa0031_measurements() {
  PM25_AQI_Data data;
  char s[strlen(sensor) + 64];

  if (! aqi.read(&data)) {
    Serial.println("[warn] could not read from AQI");
    delay(500);
    return;
  }

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm10standard");
  node.publishMQTTMeasurement(topic, s, data.pm10_standard, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm25standard");
  node.publishMQTTMeasurement(topic, s, data.pm25_standard, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm100standard");
  node.publishMQTTMeasurement(topic, s, data.pm100_standard, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm10env");
  node.publishMQTTMeasurement(topic, s, data.pm10_env, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm25env");
  node.publishMQTTMeasurement(topic, s, data.pm25_env, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/pm100env");
  node.publishMQTTMeasurement(topic, s, data.pm100_env, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount03um");
  node.publishMQTTMeasurement(topic, s, data.particles_03um, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount05um");
  node.publishMQTTMeasurement(topic, s, data.particles_05um, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount10um");
  node.publishMQTTMeasurement(topic, s, data.particles_10um, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount25um");
  node.publishMQTTMeasurement(topic, s, data.particles_25um, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount50um");
  node.publishMQTTMeasurement(topic, s, data.particles_50um, 0);

  strcpy(s, sensor); strcat(s, "/pmsa003i/partcount100um");
  node.publishMQTTMeasurement(topic, s, data.particles_100um, 0);
}


void publish_bme680_measurements() {
  char s[strlen(sensor) + 64];

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
  uint16_t                  scd4x_error;
  
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
    Serial.println("");

    // check for i2c device connectivity
    while (!i2c_device_connected) {
      /// bme680
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

      if (!sht4.begin()) {
        Serial.println("[warn]: Could not find Adafruit Temp/Humidity sensor. Check your connections and verify the address 0x44 is correct.");
        blink_led(LED_FAIL, LED_FAST);        
      } else {
        sht4.setHeater(SHT4X_NO_HEATER);
        sht4.setPrecision(SHT4X_HIGH_PRECISION);

        sht4x_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit SHT4x");
        blink_led(LED_OK, LED_SLOW);
      }
      
      /// pm25aqi
      if (! aqi.begin_I2C()) {
        Serial.println("[warn]: Could not find Adafruit PMSA003I AQ sensor. Check your connections and verify the address 0x12 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        pm25aqi_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit PM25AQI");
        blink_led(LED_OK, LED_SLOW);
      }

      /// scd4x
      scd4x.begin(Wire);
      scd4x_error = scd4x.stopPeriodicMeasurement();
      if (scd4x_error) {
          Serial.println("[error]: Error trying to execute stopPeriodicMeasurement(): ");
          blink_led(LED_FAIL, LED_FAST);
      } else {
        scd4x_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit SCD4x");
        blink_led(LED_OK, LED_SLOW);
      }

      /// ltr390
      if ( ! ltr.begin() ) {
          Serial.println("[error]: Couldn't find LTR390 sensor!");
          blink_led(LED_FAIL, LED_FAST);
      } else {
        Serial.println("[info]: OK Found LTR390 sensor");
        ltr.setResolution(LTR390_RESOLUTION_16BIT);
        ltr.setGain(LTR390_GAIN_3);
        ltr.setMode(LTR390_MODE_UVS);
        ltr.setThresholds(100, 1000);
          
        ltr390_attached = true;
        i2c_device_connected = true;
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
    reset_interval   = 1000 * 60 * atoi((const char*)doc["iotwx_reset_interval"]);
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
    if (bme680_attached)  publish_bme680_measurements();
    if (pm25aqi_attached) publish_pmsa0031_measurements();
    if (scd4x_attached) publish_scd4x_measurements();
    if (ltr390_attached) publish_ltr390_measurements();
    if (sht4x_attached) publish_sht4x_measurements();
    
    // Configure the timer to wake us up!
    delay(1000);
  }

  if ( millis() - start_millis > reset_interval ) esp_restart();

  Serial.println("[info]: sleeping");
  esp_sleep_enable_timer_wakeup(publish_interval * 60L * 1000000L);
  esp_light_sleep_start();
}
