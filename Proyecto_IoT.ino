/* 
 *  
 *    Proyecto de IoT
 *    Sensors:
 *      - HX711   20 kg loadcell            pin 18 (SCK) and 33 (DIN)           
 *      - DHT11   Temperature and humidity  pin 32                        
 *      - YL38    Soil conductivity         pin 34                              
 *      - DS18B20 Soil temperature          pin 19 with a 3.3k pullup resistor  
 *      - Relay0  Food door                 pin 14
 *      - Relay1  Hummus collector          pin 27
 *      - Relay2  Water valve               pin  5
 *      - LED     Built in LED              pin  2
 *      
 */         

#include "HX711.h"
#include "DHT.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>

/********* HTTP client *********/
#define AP_SSID 
#define AP_PWD  
#define POSTURL "http://167.172.209.34/sensors/store"
WiFiMulti wifiMulti;
HTTPClient http; 
int httpResponseCode;
const size_t capacity = JSON_OBJECT_SIZE(6)+516;
String requestBody, response;
String system_status;
/********* HTTP client *********/

/************** LED ************/
#define LED_BUILTIN 2
/************** LED ************/

/************ DS18B20 **********/
#define ONE_WIRE_BUS 19
#define HUMUS_TEMP_LOWER  20
#define HUMUS_TEMP_HIGHER 80
OneWire oneWire_in(ONE_WIRE_BUS);
DallasTemperature DS18B20_val(&oneWire_in);
float humusTemp;
/************ DS18B20 **********/

/************ HX711 **********/ 
#define DOUT_PIN 33
#define SCK_PIN  18
#define CALIBRATION_FACTOR -105
HX711 scale;
double curr_weight = 0, past_weight = 0, initial_weight = 0;
/************ HX711 **********/
 
/************ YL38 **********/
#define YL38_IN 34.
#define CONDUCTIVITY_LOWER  20
#define CONDUCTIVITY_HIGHER 80
int   conductivity;
float analog_conductivity;
/************ YL38 **********/

/************ DHT11 **********/
#define DHTPIN 32
#define DHTTYPE DHT11
#define AIR_TEMP_LOWER  20
#define AIR_TEMP_HIGHER 80
#define AIR_HUM_LOWER   40
#define AIR_HUM_HIGHER  80
DHT dht(DHTPIN, DHTTYPE);
float air_humidity;
float air_temperature;
/************ DHT11 **********/

/********** RELAYS **********/
#define FOOD_DOOR                14             // GPIO
#define FOOD_ACTIVATION_TIME     5000           // For 5s
#define FOOD_ACTIVATION_PERIOD   60000000       // For debgging 6s              //1123000000000  // Every 13 days
#define HUMUS_COLLECTOR          27             // GPIO
#define HUMUS_ACTIVATION_TIME    30000          // For 30s
#define HUMUS_ACTIVATION_PERIOD  90000000       // For debuggin 9s              //86400000000    // Every day
#define WATER_VALVE              5              // GPIO
#define WATER_VALVE_TIME         10000          // For 10s
/********** RELAYS **********/

/********** ALARMS **********/
hw_timer_t * JSON_TIMER  = NULL;   // Send JASON package
hw_timer_t * FOOD_TIMER  = NULL;   // Open the food door
hw_timer_t * HUMUS_TIMER = NULL;   // COllect the hummus
bool JSON_timeout, Food_timeout, Humus_timeout;
/********** ALARMS **********/

#define ONE_HOUR_PERIOD 30000000                // For debugging 3s              //3600000000         // 1 hour

bool waterValveEn = false;

/******** CALLBACKS *********/
void IRAM_ATTR JSON_Callback() {
  JSON_timeout = true;
}

void IRAM_ATTR FOOD_Callback() {
  Food_timeout = true;
}

void IRAM_ATTR HUMUS_Callback() {
  Humus_timeout = true;
}
/******** CALLBACKS *********/


void setup() {
  Serial.begin(115200);

  Serial.println("**********************************************************");
  Serial.println("IoT project starting...");
  Serial.println(" ");




/************** LED ************/
  /* Initialize digital pin LED_BUILTIN as an output */
  pinMode(LED_BUILTIN, OUTPUT);
/************** LED ************/




/********** ALARMS **********/
  /* Configure the timers */
  JSON_TIMER  = timerBegin(0, 80, true);
  FOOD_TIMER  = timerBegin(1, 80, true);
  HUMUS_TIMER = timerBegin(2, 80, true);

  /* Attach the callbacks */
  timerAttachInterrupt(JSON_TIMER,  &JSON_Callback,  true);
  timerAttachInterrupt(FOOD_TIMER,  &FOOD_Callback,  true);
  timerAttachInterrupt(HUMUS_TIMER, &HUMUS_Callback, true);

  /* Configure the alarms */
  timerAlarmWrite(JSON_TIMER,  ONE_HOUR_PERIOD,         true);
  timerAlarmWrite(FOOD_TIMER,  FOOD_ACTIVATION_PERIOD,  true);
  timerAlarmWrite(HUMUS_TIMER, HUMUS_ACTIVATION_PERIOD, true);

  /* Enable the alarms */
  timerAlarmEnable(JSON_TIMER);
  timerAlarmEnable(FOOD_TIMER);
  timerAlarmEnable(HUMUS_TIMER);
/********** ALARMS **********/




/********* HTTP client *********/
  wifiMulti.addAP(AP_SSID, AP_PWD);
   
  // Block until we are connected to WiFi
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("WiFi connection established"); 
  }
  else {
    /* WiFi connection was not stablished */
    Serial.println("WiFi connection not established");
  }
/********* HTTP client *********/



  
/************ HX711 **********/
  scale.begin(DOUT_PIN, SCK_PIN);

  /* Calibration */
  scale.set_scale();
  scale.tare();
  scale.set_scale(CALIBRATION_FACTOR);

  delay(500);
  if (scale.is_ready()) {
      initial_weight = scale.get_units(10);
  }
/************ HX711 **********/




/************ DS18B20 **********/
  DS18B20_val.begin();;
/************ DS18B20 **********/




/************ DHT11 **********/
  dht.begin();   
/************ DHT11 **********/




/********** RELAYS **********/
  pinMode(FOOD_DOOR, OUTPUT);
  pinMode(HUMUS_COLLECTOR, OUTPUT);
  pinMode(WATER_VALVE, OUTPUT);
  digitalWrite(FOOD_DOOR, HIGH);        // Turn off
  digitalWrite(HUMUS_COLLECTOR, HIGH);  // Turn off
  digitalWrite(WATER_VALVE, HIGH);      // Turn off
/********** RELAYS **********/
}

void loop() {
  if(JSON_timeout) {
    Serial.println("**********************************************************");



  
/************ HX711 **********/   
    if (scale.is_ready()) {
      curr_weight = scale.get_units(10);
      if((curr_weight > (past_weight + 10)) || (curr_weight < (past_weight - 10)))
        if(0 > curr_weight)
          past_weight = 0.00;
        else
          past_weight = curr_weight;
      Serial.print("HX711 weight reading:       ");
      Serial.print(past_weight);
      Serial.println(" grams.");
    }

    past_weight = past_weight - initial_weight;

    if(0 > past_weight)
      system_status = system_status + "Weight alert!";
/************ HX711 **********/



  
/************ DS18B20 **********/
    DS18B20_val.requestTemperatures();
    humusTemp = DS18B20_val.getTempCByIndex(0);
    Serial.print("DS18B20 reading:            ");
    Serial.print(humusTemp);
    Serial.println("°C.");

    if((HUMUS_TEMP_LOWER > humusTemp) || (HUMUS_TEMP_HIGHER < humusTemp))
      system_status = system_status + "Humus Temperature alert!";
/************ DS18B20 **********/



  
/************ YL38 **********/
    analog_conductivity = analogRead(YL38_IN);
    conductivity        = map(analog_conductivity, 0, 4095, 100, 0);
    Serial.print("YL38  conductivity reading: ");
    Serial.print(conductivity);
    Serial.println(" %.");

    if((CONDUCTIVITY_LOWER > conductivity) || (CONDUCTIVITY_HIGHER < conductivity))
      system_status = system_status + "Humus Conductivity alert!";
/************ YL38 **********/



  
/************ DHT11 **********/
    air_humidity    = dht.readHumidity();
    air_temperature = dht.readTemperature();
    Serial.print("DHT11 humidity reading:     ");
    Serial.print(air_humidity);
    Serial.println(" %.");
    Serial.print("DHT11 temperature reading:  ");
    Serial.print(air_temperature - 1);
    Serial.println(" °C.");

    if((AIR_TEMP_LOWER > air_temperature) || (AIR_TEMP_HIGHER < air_temperature))
      system_status = system_status + "Air Temperature alert!";
    if((AIR_HUM_LOWER > air_humidity) || (AIR_HUM_HIGHER < air_humidity))
      system_status = system_status + "Air Humidity alert!";
/************ DHT11 **********/




    waterValveEn = true;




/******* Posting JSON ********/
    http.begin(POSTURL);  
    http.addHeader("Content-Type", "application/json");       
  
    /* Create the Json package */
    DynamicJsonDocument doc(capacity);

    doc["peso"]                 = past_weight;
    doc["temperatura_humus"]    = humusTemp;
    doc["conductividad_humus"]  = conductivity;
    doc["humedad_aire"]         = air_humidity;
    doc["temperatura_aire"]     = air_temperature - 1;
    doc["status"]               = system_status;
    
    serializeJson(doc, requestBody);
  
    /* Post the Json package */
    Serial.println("JSON data is beign posted to server...");
    httpResponseCode = http.POST(requestBody);
    Serial.println("JSON data was posted to server...");
  
    /* Verify the response of the server */
    if(httpResponseCode>0){
       
      response = http.getString();                       
       
      Serial.println(httpResponseCode);   
      Serial.println(response);
  
      /* Toggle the LED */
      digitalWrite(LED_BUILTIN, LOW);     // Turn on  
      delay(1000);                       
      digitalWrite(LED_BUILTIN, HIGH);    // Turn off
    }
    else {
  
      /* An error occurred while posting */
      Serial.println("Error occurred while posting");
    }
    
    requestBody = "";
    system_status = "";
    
    JSON_timeout = false;
  }
/******* Posting JSON ********/




  /* Food door activation */
  if(Food_timeout) {
    Serial.println("Food door opened. Servo motor ON.");
    digitalWrite(FOOD_DOOR, LOW);         // Turn on
    delay(FOOD_ACTIVATION_TIME);
    digitalWrite(FOOD_DOOR, HIGH);        // Turn off
    Serial.println("Food dooor closed. Servo motor OFF.");
    
    Food_timeout = false;
  }




  /* Humus collector activation */
  if(Humus_timeout) {
    Serial.println("Collecting humus. Stepper motor ON.");
    digitalWrite(HUMUS_COLLECTOR, LOW);   // Turn on
    delay(HUMUS_ACTIVATION_TIME);
    digitalWrite(HUMUS_COLLECTOR, HIGH);  // Turn off
    Serial.println("Not collecting humus. Stepper motor OFF.");
    
    Humus_timeout = false;
  }



  /* Water valve activation */
  if((AIR_HUM_LOWER > air_humidity) && (CONDUCTIVITY_LOWER > conductivity) && waterValveEn) {
    Serial.println("Pouring water. Water valve ON.");
    digitalWrite(WATER_VALVE, LOW);   // Turn on
    delay(WATER_VALVE_TIME);
    digitalWrite(WATER_VALVE, HIGH);  // Turn off
    Serial.println("Not pouring water. Water valve OFF.");
    
    waterValveEn = false;
  }
    
}
