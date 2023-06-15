#include <SoftwareSerial.h>
#include "LoRa_E32.h"
SoftwareSerial ss(25, 26);

  LoRa_E32 e32ttl(&Serial2);
 
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
 
void setup() { 
  ss.begin(9600);  
  Serial.begin(9600); 
  e32ttl.begin();
  delay(500);
}
 
void loop() {

  while (e32ttl.available()  > 1) {

    
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    struct Signal data = *(Signal*) rsc.data;

    ss.print("t10.txt="); 
    ss.write(0x22);
    ss.print(data.yukseklik);
    ss.print(" m");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t11.txt=");
    ss.write(0x22);
    ss.print(data.basinc*100);
    ss.print(" Pa");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);

    
    ss.print("t13.txt=");
    ss.write(0x22);
    ss.print(data.nem);
    ss.print(" %");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);

    ss.print("t12.txt=");
    ss.write(0x22);
    ss.print(data.sicaklik);
    ss.println(" C");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t14.txt=");
    ss.write(0x22);
    ss.print(data.uv);
    ss.print(" mW/cm^2");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t15.txt=");
    ss.write(0x22);
    ss.print(data.konum_enlem,6);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t16.txt=");
    ss.write(0x22);
    ss.print(data.konum_boylam,6);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t17.txt=");
    ss.write(0x22);
    ss.print(data.konum_yukseklik);
    ss.print(" m");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


  delay(100);// smart
  Serial.print(data.konum_yukseklik);
  Serial.print(" ");
  Serial.print(data.konum_enlem);
  Serial.print(" ");
  Serial.print(data.konum_boylam);
  Serial.print(" ");






  }
}