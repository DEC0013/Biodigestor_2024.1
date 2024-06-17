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