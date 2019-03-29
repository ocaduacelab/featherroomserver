/************************ 
Temp / Humidity Sensor as webserver
ACE Lab November 2018
Basic temp / humidity sensor that publishes JSON to a standalone AP / webserver
*************************/

#include <ESP8266WiFi.h>                          // Include the Wi-Fi library
#include <ESP8266WiFiMulti.h>   // Include the Wi-Fi-Multi library
#include <WiFiClient.h>                           // Client WIFI library
#include <ESP8266mDNS.h>                          // MDNS --> so you can set something like esp.local (doesn't work on Android)
#include <ESP8266WebServer.h>                     // Web Server Library
#include <ArduinoJson.h>
#include "DHT.h"
#include <Servo.h> 


/********** TEMP / HUMID SENSOR **************/

#define DHTTYPE DHT22 
#define DHTPIN 2  
//NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// You need to increase the threshold for cycle counts considered a 1 or 0.
DHT dht(DHTPIN, DHTTYPE, 11);
float humidity;
float temp_c;
float temp_f; 

/********** PHOTOCELL **************/
 
int photocellPin = A0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider

/********** SERVO **************/

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

unsigned long previousMillis = 0;         // will store last temp was read
const long interval = 5000;               // interval at which to read sensor 

/********** ESP Server and AP **************/

ESP8266WiFiMulti wifiMulti;

const char* ssid     = "xxxx";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "xxxx";     // The password of the Wi-Fi network

ESP8266WebServer server(80);                      // Create a webserver object that listens for HTTP request on port 80

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
  delay(10);
  Serial.println('\n');  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);    // god that's weird. must have a built in pullup
  led_status="off";
  sweeper1.Attach(16);
  sweeper1.Reset();
  servoGo = false;

  wifiMulti.addAP(ssid, password);   // add Wi-Fi networks you want to connect to

  Serial.println("Connecting ...");
  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer

  if (!MDNS.begin("roomstation")) {                                 // Start the mDNS responder for roomstation.local
    Serial.println("Error setting up MDNS responder!");
  }

  Serial.println("mDNS responder started");

  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());                   // Send the IP address of the ESP8266 to the computer

  server.on("/", handleRoot);                       // Call the 'handleRoot' function when a client requests URI "/"
  server.onNotFound(handleNotFound);                // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  server.on("/led", HTTP_POST, handleLED);          // Call the 'handleLED' function when a POST request is made to URI "/LED"
  server.on("/servo", HTTP_POST, handleServo);
  server.on("/ambientroom",handleJSON);

  server.begin();                           // Actually start the server
  Serial.println("HTTP server started");

  dht.begin();
  Serial.println("DHTxx started");

}

void loop() { 
  server.handleClient();
  sweeper1.Update();
}

/********** FUNCTIONS **************/

void getAmbient() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;   
    humidity = dht.readHumidity();          // Read humidity (percent)
    temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
    temp_c = dht.readTemperature(); 
    photocellReading = analogRead(photocellPin); 
    // Check if any reads failed and exit early (to try again).
    if(digitalRead(LED_BUILTIN) == 1){
      led_status = "off";
    }else{
      led_status = "on";
    }

    if (isnan(humidity) || isnan(temp_f) || isnan(temp_c)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }
}


String createJsonResponse(){
  
  getAmbient();

  String tempF = String((float)temp_f);
  String tempC = String((float)temp_c);
  String humid = String((float)humidity);
  String light = String((int)photocellReading);
  String servo = servo_status;
  String ledStatus = led_status;

  StaticJsonBuffer<500> jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  JsonArray &tempCValue = root.createNestedArray("temperature_c");
  JsonArray &humidValue = root.createNestedArray("humidity");
  JsonArray &tempFValue = root.createNestedArray("temperature_f");
  JsonArray &lightValue = root.createNestedArray("light_brightness");
  JsonArray &servoValue = root.createNestedArray("servo_running");
  JsonArray &ledValue = root.createNestedArray("led_status");

  
  tempCValue.add(tempC);
  tempCValue.add("celcius");
  
  tempFValue.add(tempF);
  tempFValue.add("fahrenheit");
  
  humidValue.add(humid);
  humidValue.add("percent");
  
  lightValue.add(light);
  servoValue.add(servo);
  ledValue.add(ledStatus);

  String json;
  root.prettyPrintTo(json);
  return json;
}

/********** Handle the root / home page and handle the 404 error page. *********/

void handleRoot() {
  server.send(200, "text/html", "<p>hello from esp8266!</p><form action=\"/led\" method=\"POST\"><input type=\"submit\" value=\"toggleLED\"></form><form action=\"/servo\" method=\"POST\"><input type=\"submit\" value=\"toggleServo\"></form>");
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

  Serial.println(servoGo);
  server.sendHeader("Location","/"); 
  server.send(303);
}

void handleJSON(){
  server.send ( 200, "text/json", createJsonResponse());
}



