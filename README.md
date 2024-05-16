
# Biodigestor com ESP32 e Sensores

Este projeto implementa um sistema de biodigestão utilizando um microcontrolador ESP32 e diversos sensores para monitoramento ambiental. O biodigestor converte resíduos orgânicos em biogás, e os sensores ajudam a monitorar parâmetros como temperatura, umidade, pressão atmosférica e qualidade do ar no ambiente do biodigestor.


## Problema Resolvido

O projeto visa facilitar o monitoramento de sistemas de biodigestão, fornecendo dados precisos em tempo real sobre as condições do ambiente, o que pode ser crucial para o desempenho eficiente do processo de biodigestão.

## Componentes Utilizados

 - ESP32 (microcontrolador)
 - DHT11 (sensor de temperatura e umidade)
 - BMP280 (sensor de pressão atmosférica)
 - MQ-2 (sensor de gás)
 - Relé (simular a válvula)

## MQ-2

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

## BMP_280

#include <Wire.h>
#include <Adafruit_BMP280.h>
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
Adafruit_MQTT_Publish Pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure01");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");

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
    //              Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    //              Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    //              Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    //              Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
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

## DHT11

#include "DHT.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
 
// WiFi parameters
#define WLAN_SSID       ""
#define WLAN_PASS       ""

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"   // Adafruit IO Cloud Platform server for IoT
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""
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



## Arquitetura do Projeto

O projeto segue uma arquitetura onde o ESP32 coleta dados dos sensores e os envia para a plataforma Adafruit via protocolo MQTT. A plataforma Adafruit recebe os dados e os exibe em um dashboard online, permitindo o monitoramento remoto do sistema de biodigestão.
