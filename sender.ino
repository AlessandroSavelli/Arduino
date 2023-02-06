#include <VirtualWire.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
SoftwareSerial MySerial(2, 3); // RX, TX


#define DHTPIN 5  
#define DHTTYPE DHT22 


const int led_pin = 13;
const int transmit_pin = 12;


  float dht_temp = 0.0;
  float temp = 0.0;

  
  float humidity = 0.0;
  float pioggia = 0.0;
  float pres = 0.0;

float altitudine_tivoli_casa = 298.0; // metri su liv. mare
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


DHT dht(DHTPIN, DHTTYPE);

void setup()
{


Serial.begin(9600);
  MySerial.begin(9600);
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.begin();
  dht.begin();
    pinMode(led_pin, OUTPUT);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */  
}



void loop()
{
  int sensorValuepioggia = analogRead(A3);
  
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  readSensor();
  

  temp = (temp_event.temperature+dht_temp)/2;
  pres = pressure_event.pressure / pow(1.0 - (altitudine_tivoli_casa/44330.0), 5.255);
  
  
  Serial.print(temp);
  Serial.println(" °C");

  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print(pioggia);
  Serial.println(" mm");

  Serial.print(pres);
  Serial.println(" hPa");
  
 Serial.println("");      // ci mette 11 secondi per scrivere sul seriale ogni volta 5000+5000+err

  

    MySerial.println(temp);
    MySerial.println(humidity);
    MySerial.println(pioggia);
    MySerial.println(pres);
    


  delay(15000);
 
}

void readSensor()
{
 
 humidity = dht.readHumidity();
 dht_temp = dht.readTemperature();
 pioggia = analogRead(A3);
 

}
