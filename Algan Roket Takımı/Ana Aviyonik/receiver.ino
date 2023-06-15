#include <SoftwareSerial.h>
#include "LoRa_E32.h"


SoftwareSerial ss(0, 4);

  LoRa_E32 e32ttl(&Serial2);  
 
  struct Signal  {
     float irtifa;
     float hiz;
     float pitch;
     float xacisi;
     float yacisi;
     float ivme;
     float konum_enlem;
     float konum_boylam;
     float konum_yukseklik;
     bool anaParasut=0;
     bool suruklenmeParasut=0;
    } data;
 
void setup() { 
  ss.begin(9600);  
  //Serial.begin(9600); 
  e32ttl.begin();
  delay(500);
}
 
void loop() {

  while (e32ttl.available()  > 1) {
    
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    struct Signal data = *(Signal*) rsc.data;
    
    ss.print("t11.txt="); 
    ss.write(0x22);
    ss.print(data.irtifa);
    ss.print(" m");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t21.txt=");
    ss.write(0x22);
    ss.print(data.hiz);
    ss.print(" m/s");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);

    
    ss.print("t12.txt=");
    ss.write(0x22);
    ss.print(data.xacisi);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);

    ss.print("t13.txt=");
    ss.write(0x22);
    ss.print(data.yacisi);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t14.txt=");
    ss.write(0x22);
    ss.print(data.pitch);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t15.txt=");
    ss.write(0x22);
    ss.print(data.ivme);
    ss.print(" g");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t17.txt=");
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


    ss.print("t18.txt=");
    ss.write(0x22);
    ss.print(data.konum_yukseklik);
    ss.print(" m");
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);


    ss.print("t19.txt=");
    ss.write(0x22);
    ss.print(data.suruklenmeParasut);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);

    ss.print("t20.txt=");
    ss.write(0x22);
    ss.print(data.anaParasut);
    ss.write(0x22);
    ss.write(0xff);
    ss.write(0xff);
    ss.write(0xff);




  delay(100);// smart
  /*Serial.print(data.irtifa);
  Serial.print(" ");
  Serial.print(data.konum_enlem);
  Serial.print(" ");
  Serial.print(data.konum_yukseklik);
  Serial.print(" ");
  Serial.print(data.konum_boylam);
  Serial.print(" ");
  Serial.println(data.xacisi);
  Serial.print(" ");*/

}




}
