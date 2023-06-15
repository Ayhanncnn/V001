#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS.h>
#include "LoRa_E32.h"

#define SEALEVELPRESSURE_HPA (1013.25)




  struct Signal  {
     float uv;
     float nem;
     float basinc;
     float sicaklik;
     float yukseklik;
     float konum_enlem;
     float konum_boylam;
     float konum_yukseklik;
    } data;

    int UVOUT = 27; //Output from the sensor
    int REF_3V3 = 14; //3.3V power on the Arduino board
    float enlem, boylam ;
    unsigned long time11,time22;//lora millis



      Adafruit_BME280 bme; 
      TinyGPS gps;
      //LoRa_E32 e32ttl(&Serial);


float referans=0;//bme offset



void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  bme.begin();
  //e32ttl.begin();

  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  referans = bme.readAltitude(SEALEVELPRESSURE_HPA);

}

void loop() {

  data.uv = UV();
  data.nem = bme.readHumidity();
  data.basinc = bme.readPressure() / 100.0F;
  data.sicaklik = bme.readTemperature();
  data.yukseklik = bme.readAltitude(SEALEVELPRESSURE_HPA)-referans;
  data.konum_enlem = konum_enlem_oku();
  data.konum_boylam = konum_boylam_oku();
  data.konum_yukseklik = konum_yukseklik_oku();

  /*time11 = millis(); 
  if (time11-time22 > 250){ 
  ResponseStatus rs = e32ttl.sendFixedMessage(0, 2, 10, &data, sizeof(Signal));// addres channel
  time22 = time11;
  }*/


  
  Serial.print(" uv = ");
  Serial.print( data.uv);
  Serial.print(" basinc =");
  Serial.print(data.basinc );
  Serial.print(" sicaklik =");
  Serial.print(data.sicaklik);
  Serial.print(" yukseklik =");
  Serial.print(data.yukseklik);
  Serial.println(" nem ");
  Serial.println(data.nem);
  delay(100);

}

  float UV(){
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  return(uvIntensity);
  }



  int averageAnalogRead(int pinToRead)
    {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);
    }
 
  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }



  float konum_enlem_oku(){
  smartdelay(100);
  gps.f_get_position(&enlem, &boylam);
  return enlem;
  }


  float konum_boylam_oku(){
  smartdelay(100);
  gps.f_get_position(&enlem, &boylam);  
  return boylam;
  }


  float konum_yukseklik_oku(){
  smartdelay(100);
  int alt = gps.f_altitude();
  return alt;
  }


  static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);}

