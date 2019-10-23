/************************ 
Web Room Server
ACE Lab October 2019
ESp32 update. 
ESp32 - Analog filter: https://github.com/jonnieZG/EWMA
note: Using wifi and the analog pins + this filter can sometimes cause conflicts at the hardware level. Try to avoid pins A0 and A1
file: server_october_esp32_2019.ino
*************************/
#include <Arduino.h>
#include <WiFi.h>                   // Include the Wi-Fi library
#include <WiFiClient.h>             // Client WIFI library 
#include <WebServer.h>              // Web Server Library
#include <ESPmDNS.h>                // MDNS --> so you can set something like esp.local (doesn't work on Android)
#include <ArduinoJson.h>            // Arduino JSON library
#include "DHT.h"                    // Humidity / Temp sensor library
#include <ESP32Servo.h>             // Servo library specifically for ESP32
#include <Ewma.h>                   // Smoothing filter library

/***** Smoothing *******/

Ewma adcFilter2(0.01);  // More smoothing - less prone to noise, but slower to detect changes

/********** TEMP / HUMID SENSOR **************/
//NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// You need to increase the threshold for cycle counts considered a 1 or 0.
#define DHTTYPE DHT22 
#define DHTPIN 14  
DHT dht(DHTPIN, DHTTYPE, 11);
float humidity;
float temp_c;
float temp_f; 

/********** PHOTOCELL **************/
int photocellPin = A2;     // the cell and 10K pulldown are connected to a2
int photocellReading;     // the analog reading from the sensor divider

/********** SOIL MOISTURE SENSOR **************/
int soilPin = A3;
int rawSoil;
int soilReading;

/********** PIR SENSOR **************/
int pirPin = 15;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int pirVal = 0;                    // variable for reading the pin status
String motion_status = "";

/********** SERVO **************/
int servoPin = 32;
bool servoGo;
String servo_status ="";

class Sweeper
{
  Servo servo;              // the servo
  int pos;              // current servo position 
  int increment;        // increment to move for each interval
  int  updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position
 
public: 
  Sweeper(int interval)
  {
    updateInterval = interval;
    increment = 1;
  }
  
  void Attach(int pin)
  {
    servo.attach(pin);
  }

  void Reset(){
    servo.write(0);
    servo_status = "off";
  }
  
  void Detach()
  {
    servo.detach();
  }
  
  void Update()
  {
    if(servoGo)
    {
      servo_status = "on";
      if((millis() - lastUpdate) > updateInterval)  // time to update
      {
        lastUpdate = millis();
        pos += increment;
        servo.write(pos);
        //Serial.println(pos);
        if ((pos >= 180) || (pos <= 0)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      }
    } else {
      servo.write(0);
      servo_status = "off";
    }
  }
};

Sweeper sweeper1(10);

/********** TIMER **************/
unsigned long previousMillis = 0;         // will store last time things were read
const long interval = 60000;              // miliseconds, 1 minute. 60000

/********** ESP Server and AP **************/
const char *wider_ssid = "xxxx";
const char *wider_password = "xxxx";

// IP address: 192.168.0.18
// MAC address: 80-7D-3A-F2-4B-5C

WebServer server(80);

void handleRoot();          
void handleNotFound();
void handleLED();
void handleJSON();
void handleServo();

/********** LED **************/

String led_status="";

/********** SETUP **************/

void setup() {
  Serial.begin(115200);
  Serial.println('\n');  
  Serial.println("file used: server_october_esp32_2019.ino");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    
  pinMode(pirPin, INPUT);
  
   WiFi.mode(WIFI_STA);
   WiFi.begin(wider_ssid, wider_password);

    // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(wider_ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  if (!MDNS.begin("roomstation")) {                                 // Start the mDNS responder for roomstation.local
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started");
  Serial.print("IP address:\t");
  
  server.on("/", handleRoot);                       // Call the 'handleRoot' function when a client requests URI "/"
  server.onNotFound(handleNotFound);                // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  server.on("/led", HTTP_POST, handleLED);          // Call the 'handleLED' function when a POST request is made to URI "/LED"
  server.on("/servo", HTTP_POST, handleServo);
  server.on("/ambientroom",handleJSON);

  server.begin();                           // Start the server
  Serial.println("HTTP server started");

  dht.begin();
  Serial.println("DHTxx started");
  
  Serial.println("setting initial values: ");
  led_status="off";
  
  sweeper1.Attach(servoPin);
  sweeper1.Reset();
  servoGo = false;
  
  pirState = LOW;
  
  humidity = dht.readHumidity();          // Read humidity (percent)
  temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
  temp_c = dht.readTemperature();         // read temp as celcius
  rawSoil = analogRead(soilPin);            // raw soil reading
  soilReading = adcFilter2.filter(rawSoil);
  Serial.println(humidity);
  Serial.println(temp_c);
  Serial.println(soilReading);
}

void loop() { 

  if(servoGo == 0){
    pirVal = digitalRead(pirPin);  // read pirValue
    photocellReading = analogRead(photocellPin);  // read the current light setting
    rawSoil = analogRead(soilPin);            // raw soil reading

    // every minute get new values for soil / temp / humidity
    unsigned long currentMillis = millis();
      if(currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;   
        Serial.println("getting new values");
        
        // temp and humidity
        humidity = dht.readHumidity();          // Read humidity (percent)
        temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
        temp_c = dht.readTemperature();         // read temp as celcius
        
        soilReading = adcFilter2.filter(rawSoil);
        if(soilReading >= 3000){
          soilReading = 3000;
        }else if (soilReading <= 1400){
          soilReading = 1400;
        }
        Serial.println(rawSoil);
        Serial.println(soilReading);
        Serial.println("");
        
        // if these aren't coming out as numbers
        if (isnan(humidity) || isnan(temp_f) || isnan(temp_c)) {
          Serial.println("Failed to read from DHT sensor!");
          return;
        }
      }
    
    // set the pir value
  if (pirVal == HIGH) {
      if (pirState == LOW) {
          //Serial.print(servoGo);
          Serial.println("Motion detected");
          //Serial.println(pirVal);
          motion_status = "yes";
          pirState = HIGH;
      }
    }else{
      if (pirState == HIGH){
        motion_status = "no";
        //Serial.println(pirVal);
        Serial.println("motion done");
        pirState = LOW;
      }
    }
  }else {
    pirState = LOW;
    motion_status = "no";
  }
  
    // set the LED status
    if(digitalRead(LED_BUILTIN) == 0){
      led_status = "off";
    }else{
      led_status = "on";
    }

  // update the server and the sweeper
  server.handleClient();
  sweeper1.Update();
}

/********** FUNCTIONS **************/

String createJsonResponse(){
  
  String tempF = String((float)temp_f);
  String tempC = String((float)temp_c);
  String humid = String((float)humidity);
  String light = String((int)photocellReading);
  String soil = String((int)soilReading);
  String servo = servo_status;
  String ledStatus = led_status;
  String motion = motion_status;

  StaticJsonBuffer<500> jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  JsonArray &tempCValue = root.createNestedArray("temperature_c");
  JsonArray &humidValue = root.createNestedArray("humidity");
  JsonArray &tempFValue = root.createNestedArray("temperature_f");
  JsonArray &lightValue = root.createNestedArray("light");
  JsonArray &soilValue = root.createNestedArray("soil_moisture");
  JsonArray &servoValue = root.createNestedArray("servo_running");
  JsonArray &ledValue = root.createNestedArray("led_status");
  JsonArray &motionValue = root.createNestedArray("motion_detected");

  humidValue.add(humid);
  humidValue.add("percent");

  ledValue.add(ledStatus);
  lightValue.add(light);
  motionValue.add(motion);
  servoValue.add(servo);
  soilValue.add(soil);

  tempCValue.add(tempC);
  tempCValue.add("celcius");  
  tempFValue.add(tempF);
  tempFValue.add("fahrenheit");

  String json;
  root.prettyPrintTo(json);
  return json;
}

/********** Handle the root / home page and handle the 404 error page. *********/

void handleRoot() {
  server.send(200, "text/html", "<p>hello from esp32!</p><form action=\"/led\" method=\"POST\"><input type=\"submit\" value=\"toggleLED\"></form><form action=\"/servo\" method=\"POST\"><input type=\"submit\" value=\"toggleServo\"></form>");
}

void handleNotFound(){
  servoGo = false;
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void handleLED(){
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));      // Change the state of the LED
  server.sendHeader("Location","/");        // Add a header to respond with a new location for the browser to go to the home page again
  server.send(303);                         // Send it back to the browser with an HTTP status 303 (See Other) to redirect

}

void handleServo(){
  
  if(servoGo == false){
    servoGo = true;
  }else if(servoGo == true){
    servoGo = false;
  }

  //Serial.println(servoGo);
  server.sendHeader("Location","/"); 
  server.send(303);
}


void handleJSON(){
  server.send ( 200, "text/json", createJsonResponse());
}
