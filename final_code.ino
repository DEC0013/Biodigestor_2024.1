/*
#include <Wire.h>

#define sensor 13


void setup() {
  Serial.begin(115200);
}

void GASLevel() {
  int value = analogRead(sensor);
  value = map(value, 0, 4095, 0, 399);

  Serial.print("GAS Level :");
  Serial.println(value);
}

void loop() {
  GASLevel(); 
  delay(5000);
}

*/


/*
#include "DHT.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
 
// WiFi parameters
#define WLAN_SSID       "aKira"
#define WLAN_PASS       "12345678"

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"   // Adafruit IO Cloud Platform server for IoT
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "brunobinelli"
#define AIO_KEY         "aio_ltsc31zuEU41e0uMLbCfbo9ARn1x"
WiFiClient client;
 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");

#define DHTPIN 4       // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11  // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  
  Serial.begin(115200);
  
  Serial.println(F("Adafruit IO Example"));
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
 
  // connect to adafruit io
  connect();

  dht.begin();
}

void connect()
{
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}
 

 
void loop() {

   // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3))
  {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

   // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  float hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temp = dht.readTemperature();
  
   if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  Serial.println("-----------");
  //Serial.println(esp_random());
  Serial.print("Humidade: ");
  Serial.print(hum, 3);
  Serial.print("% | Temperatura: ");
  Serial.print(temp, 3);
  Serial.print("°C");

    delay(5000);
  
  if (! Temperature.publish(temp)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
       if (! Humidity.publish(hum)) {               //Publish Humidity data to Adafruit
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    }

}

*/


/*
#include <Wire.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       ""
#define WLAN_PASS       ""

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"   // Adafruit IO Cloud Platform server for IoT
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas1");

#define sensor 4


void setup() {
    
    Serial.begin(115200);

    Serial.println(WLAN_SSID);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(F("."));
    }
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
 
    // connect to adafruit io
    connect();

}

void connect()
{
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
    
      // ping adafruit io a few times to make sure we remain connected
    if(! mqtt.ping(3))
    {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
    }
    
    int value = analogRead(sensor);
    value = map(value, 0, 4095, 0, 100);

    Serial.print("GAS Level :");
    Serial.println(value);

    delay(5000);

    if (!Gas.publish(value)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
    else{
       Serial.println(F("Sent!"));
    } 
}

*/

/*
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "aKira"
#define WLAN_PASS       "12345678"

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"   // Adafruit IO Cloud Platform server for IoT
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "brunobinelli"
#define AIO_KEY         "aio_ltsc31zuEU41e0uMLbCfbo9ARn1x"
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure01");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature01");

Adafruit_BMP280 bmp;

  void setup() {
    
    Serial.begin(115200);
    Serial.println(F("BMP280 test"));
    unsigned status;
    status = bmp.begin(0x76);
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(bmp.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    //        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    //             Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    //            Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    //             Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    
    /*
    Serial.println(WLAN_SSID);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(F("."));
    }
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
 
    // connect to adafruit io
    connect();
  }

  void connect()
  {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
  }

  void loop() {
    
    if(! mqtt.ping(3))
    {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
    }

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");


    Serial.println();
    delay(5000);

    if (!Pressure.publish(bmp.readPressure())) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
    if (!Temperature.publish(bmp.readTemperature())) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
    else{
       Serial.println(F("Sent!"));
    } 
}

*/


/*
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

 
// WiFi parameters
#define WLAN_SSID       "aKira"
#define WLAN_PASS       "12345678"

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"   // Adafruit IO Cloud Platform server for IoT
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "brunobinelli"
#define AIO_KEY         "aio_ltsc31zuEU41e0uMLbCfbo9ARn1x"
WiFiClient client;
 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish ExternalTemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ExternalTemperature");
Adafruit_MQTT_Publish Pressure01 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure01");
Adafruit_MQTT_Publish Temperature01 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature01");
Adafruit_MQTT_Publish Gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas");
Adafruit_MQTT_Publish Pressure02 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure02");
Adafruit_MQTT_Publish Temperature02 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature02");

#define sensor 13

#define DHTPIN 4      // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11  // DHT 11

DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP280 bmp;
Adafruit_BMP280 bmp2;

int RELAY = 14;

int relayState = 0;

void setup() {
  
  Serial.begin(115200);

  Serial.println(F("BMP280 test"));
    unsigned status;
    status = bmp.begin(0x76);
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(bmp.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500);  

    unsigned status2;
    status2 = bmp2.begin(0x77);
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(bmp.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500);  
   
    pinMode(RELAY, OUTPUT);

    digitalWrite(RELAY, LOW); 
  
  Serial.println(F("Adafruit IO Example"));
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
 
  // connect to adafruit io
  connect();

  dht.begin();
}
 
void loop() {

    relayState = digitalRead(RELAY);

   // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3))
  {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  // Read temperature as Celsius (the default)
  float ext_temp = dht.readTemperature();
  
   if (isnan(ext_temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  GASLevel();

  float pres01 = bmp.readPressure();
  float temp01 = bmp.readTemperature();

  float pres02 = bmp2.readPressure();
  float temp02 = bmp2.readTemperature();

  /*if(relayState == HIGH)
  {
    digitalWrite(RELAY, LOW); 

  }
  else
  {
     digitalWrite(RELAY, HIGH); 
  };*/
  
  /*
  Serial.println("-----------");
  //Serial.println(esp_random());
  Serial.print("Temperatura Externa: ");
  Serial.print(ext_temp, 3);
  Serial.print("°C");

  Serial.println();

  Serial.print(F("Pressão Interna 01 = "));
  Serial.print(pres01);
  Serial.println(" Pa");

  Serial.println();

  Serial.print(F("Temperatura Interna 01 = "));
  Serial.print(temp01);
  Serial.println(" °C");

  Serial.println();

  Serial.print(F("Pressão Interna 02 = "));
  Serial.print(pres02);
  Serial.println(" Pa");

  Serial.println();

  Serial.print(F("Temperatura Interna 02 = "));
  Serial.print(temp02);
  Serial.println(" °C");

  Serial.println();


  Serial.println("-----------");

  delay(30000);
  
  if (!ExternalTemperature.publish(ext_temp)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
  if (!Pressure01.publish(pres01)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
  if (!Temperature01.publish(temp01)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
  if (!Temperature02.publish(temp02)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
  if(!Gas.publish(value))
   if (!Pressure02.publish(pres02)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    }

}

void connect()
{
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void GASLevel() {
  int value = analogRead(sensor);
  value = map(value, 0, 4095, 0, 399);

  Serial.print("GAS Level :");
  Serial.println(value);
}
*/
/*
#include "DHT.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "AdafruitIO_WiFi.h"

#define IO_SERVER      "io.adafruit.com" 
#define IO_SERVERPORT  1883
#define IO_USERNAME "brunobinelli"
#define IO_KEY "aio_ltsc31zuEU41e0uMLbCfbo9ARn1x"
WiFiClient client;

#define WIFI_SSID "aKira"
#define WIFI_PASS "12345678"

#if defined(USE_AIRLIFT) || defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) ||         \
    defined(ADAFRUIT_PYPORTAL)
#if !defined(SPIWIFI_SS) // if the wifi definition isnt in the board 
#define SPIWIFI SPI
#define SPIWIFI_SS 10 // Chip select pin
#define NINA_ACK 9    // a.k.a BUSY or READY pin
#define NINA_RESETN 6 // Reset pin
#define NINA_GPIO0 -1 // Not connected
#endif
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, SPIWIFI_SS,
                   NINA_ACK, NINA_RESETN, NINA_GPIO0, &SPIWIFI);
#else
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
#endif

Adafruit_MQTT_Client mqtt(&client, IO_SERVER, IO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Humidity");

// digital pin 5
#define RELAY_PIN 14

// set up the 'digital' feed
AdafruitIO_Feed *valve = io.feed("valve");

#define DHTPIN 4       // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11  // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  
  pinMode(RELAY_PIN, OUTPUT);
  
  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // set up a message handler for the 'digital' feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  valve->onMessage(handleMessage);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  valve->get();

}

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  float hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temp = dht.readTemperature();
  
   if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.println("-----------");
  //Serial.println(esp_random());
  Serial.print("Humidade: ");
  Serial.print(hum, 3);
  Serial.print("% | Temperatura: ");
  Serial.print(temp, 3);
  Serial.print("°C");

  delay(10000);
  
  if (! Temperature.publish(temp)) {               //Publish Temperature data to Adafruit
      Serial.println(F("Failed"));
    }
       if (! Humidity.publish(hum)) {               //Publish Humidity data to Adafruit
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    }
}

// this function is called whenever an 'digital' feed message
// is received from Adafruit IO. it was attached to
// the 'digital' feed in the setup() function above.
void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");

  if(data->toPinLevel() == HIGH)
    Serial.println("HIGH");
  else
    Serial.println("LOW");


  digitalWrite(RELAY_PIN, data->toPinLevel());
}
*/

// ********** Bibliotecas ***********

#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "AdafruitIO_WiFi.h"

// ***********************************

// ***** Config. Wifi e Adafruit *****

#define WLAN_SSID       "your_ssid"
#define WLAN_PASS       "your_pass"

#define AIO_SERVER      "io.adafruit.com"  
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "your_usarname"
#define AIO_KEY         "your_key"
WiFiClient client;

//**************************************
 
//*********** Config. MQTT *************

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish ExternalTemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ExternalTemperature");
Adafruit_MQTT_Publish Pressure01 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure01");
Adafruit_MQTT_Publish Temperature01 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature01");
Adafruit_MQTT_Publish Gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas");
Adafruit_MQTT_Publish Pressure02 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure02");
Adafruit_MQTT_Publish Temperature02 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature02");
Adafruit_MQTT_Subscribe valve = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/valve");

//**************************************

//DHT11 Definitions
#define DHTPIN 4 
#define DHTTYPE DHT11 

//MQ2 Definitions
#define Board ("ESP-32")
#define Pin (35)  
#define Type ("MQ-2") 
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (12) 
#define RatioMQ2CleanAir (9.83) //RS / R0 = 9.83 ppm
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//RELAY Defintions
#define RELAY 14
int RELAY_state=0;

//DHT11 Creation
DHT DHT(DHTPIN, DHTTYPE);
float DHT_Temperature=0;

//MQ2 Creation
float MQ2_Concentration=0;

//BMP280 Creation
Adafruit_BMP280 BMP1;
Adafruit_BMP280 BMP2;
float BMP1_Temperature=0;
float BMP2_Temperature=0;
float BMP1_Pressure=0;
float BMP2_Pressure=0;

const int v_valve = 14; 

void connect()
{
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
  
}

void setup() {
  Serial.begin(115200);

  delay(1000);

  //Initiate DHT11
  DHT.begin();

  delay(1000);

  //Initiate MQ2
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25); 
  MQ2.setB(-2.222); 
  MQ2.init(); 
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); 
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
  }
  MQ2.setR0(calcR0/10);
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}

  //Initiate RELAY
  pinMode(RELAY, OUTPUT);

  pinMode(v_valve, OUTPUT);

  mqtt.subscribe(&valve); 

  //Iniate First BMP280
  unsigned status1;
  status1 = BMP1.begin(0x76);
  if (!status1) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(BMP1.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  BMP1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Iniate Second BMP280
  unsigned status2;
  status2 = BMP2.begin(0x77);
  if (!status2) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(BMP2.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  BMP2.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Connecting to Adafruit IO
  Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  connect();
}

void loop() {

  //Ping Adafruit IO a few times to make sure we remain connected
  if(!mqtt.ping(3))
  {
    //Reconnect to Adafruit IO
    if(!mqtt.connected())
    {
      connect();
    }
  }

  delay(1000);

  //DHT11 Reading and Printing
  DHT_Temperature=DHT.readTemperature();
  Serial.print(F("Outside Temperature: "));
  Serial.print(DHT_Temperature);
  Serial.println(" °C ");
  Serial.println();

  delay(1000);

  //MQ2 Reading and Printing
  MQ2.update();
  MQ2_Concentration=MQ2.readSensor();
  Serial.print(MQ2_Concentration);
  Serial.println(" PPM");
  Serial.println();

  delay(1000);

  //BMP280(First) Reading and Printing
  BMP1_Temperature=BMP1.readTemperature();
  BMP1_Pressure=BMP1.readPressure()/100000;
  Serial.print(F("Inside Temperature Tank 01 = "));
  Serial.print(BMP1_Temperature);
  Serial.println(" °C");
  Serial.print(F("Inside Pressure Tank 01 = "));
  Serial.print(BMP1_Pressure);
  Serial.println(" Bar");
  Serial.println();

  delay(1000);

  //BMP280(Second) Reading and Printing
  BMP2_Temperature=BMP2.readTemperature();
  BMP2_Pressure=BMP2.readPressure()/100000;
  Serial.print(F("Inside Temperature Tank 02 = "));
  Serial.print(BMP2_Temperature);
  Serial.println(" °C");
  Serial.print(F("Inside Pressure Tank 02 = "));
  Serial.print(BMP2_Pressure);
  Serial.println(" Bar");
  Serial.println();

  //RELAY Condition
  RELAY_state = digitalRead(RELAY);
  if(BMP1_Pressure >= 1.01)
  {
    digitalWrite(RELAY, LOW);  
    delay(1000);                       
  }
  else
  {
    digitalWrite(RELAY, HIGH);  
    delay(1000);    
  }
  

  //Publishing on Adafruit
  if(!ExternalTemperature.publish(DHT_Temperature)) 
  {            
    Serial.println(F("Failed"));
  }
  if(!Temperature01.publish(BMP1_Temperature)) 
  {            
    Serial.println(F("Failed"));
  }
  if(!Temperature02.publish(BMP2_Temperature)) 
  {            
    Serial.println(F("Failed"));
  }
  if(!Pressure01.publish(BMP1_Pressure))
  {               
    Serial.println(F("Failed"));
  }
  if(!Pressure02.publish(BMP2_Pressure))
  {               
    Serial.println(F("Failed"));
  }
  if(!Gas.publish(MQ2_Concentration)) 
  {               
    Serial.println(F("Failed"));
  }
  else
  {
    Serial.println(F("Sent!"));
  }

  Adafruit_MQTT_Subscribe *subscription; 
  while ((subscription = mqtt.readSubscription(5000))) { 
      
    if (subscription == &valve) {      
      
      if (strcmp((char *)valve.lastread, "ON") == 0) { 
         Serial.println(F("LIGOU!"));
         Serial.print(v_valve);
         digitalWrite(v_valve, LOW);                        
      }
      if (strcmp((char *)valve.lastread, "OFF") == 0) { 
        Serial.println(F("DESLIGOU!"));
         Serial.print(v_valve);
        digitalWrite(v_valve, HIGH);                           
      }
      
    }
  }

  delay(20000);


}