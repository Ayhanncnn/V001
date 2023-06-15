#include <Adafruit_BMP280.h>
#include <MPU6050_tockn.h>
#include <ADXL345_WE.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include "EEPROM.h"
#include "LoRa_E32.h"

  struct Signal  {
     float irtifa1;
     float hiz1;
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

  float ilk_irtifa_1,son_irtifa_1,hesaplanan_hiz_1=0;//hız1 hesapla fonk
  unsigned long eskiZaman1=0;//hiz1 hesapla fonk içerisinde kullanılan milis için
  unsigned long yeniZaman1;
  unsigned long time11,time22;

  float referans1 =0.000, irtifa_1 =0.000; //bmp offset değerleri için
  float enlem, boylam ;
  int time2,time1;// sürüklenme paraşütü delay

  int a,b,c,d;
  bool roket_ateslendi_mi = false;
  bool suruklenme_parasutu_acildi_mi = false;
  bool ana_parasut_acildi_mi = false;
  bool atesleme1 = false;
  bool atesleme2 = false;

  bool hiz_durum = false;


  //eeprom
    /*int eepitch = 0;
      int eeyacisi = 800000;
      int eexacisi = 1600000;
      int eeirtifa = 2400000;
      int eehiz = 3200000;*/




  //Kalman delayleri//

  SimpleKalmanFilter irtifa_1KalmanFilter(1, 1, 0.01); 
  SimpleKalmanFilter pitchKalmanFilter(1, 1, 0.01);
  SimpleKalmanFilter xacisiKalmanFilter(1, 1, 0.01);
  SimpleKalmanFilter yacisiKalmanFilter(1, 1, 0.01);
  SimpleKalmanFilter ivmeKalmanFilter(1, 1, 0.01);




      #define buzzer 12
      #define ADXL345_I2CADDR 0x53
      ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);
      Adafruit_BMP280 bmp; 
      MPU6050 mpu6050(Wire);
      TinyGPSPlus gps;
      

      LoRa_E32 e32ttl(&Serial);

void setup() {
  //gate tetikleme
  pinMode(33,OUTPUT);//Ana
  pinMode(32,OUTPUT);//Tepe Noktası

  pinMode(buzzer,OUTPUT);//buzzer sistemin açıldığı bilgisini verecek
  for(int z=0;z<26;z++){
  digitalWrite(buzzer,HIGH);
  delay(50);
  digitalWrite(buzzer,LOW);
  delay(50);
  }
      Wire.begin();

      //bmp
       bmp.begin();
      referans1=bmp.readAltitude(1013.25);
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
        Adafruit_BMP280::FILTER_X8,      /* Filtering.*/
        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */ 

      // mpu 
      mpu6050.begin();
      mpu6050.calcGyroOffsets(true);

      //ADXL
      myAcc.init();
      myAcc.setCorrFactors(-266.0, 285.0, -268.0, 278.0, -291.0, 214.0);
      delay(2000);
      myAcc.measureAngleOffsets();

      // gps uart
       Serial2.begin(9600);
       //Serial.begin(9600);

      //lora
      e32ttl.begin();
      delay(500);

      //eeprom
      //EEPROM.begin(4000000);

       /// loopa girdiğini anlamak için
      digitalWrite(buzzer,HIGH);
      delay(1000);
      digitalWrite(buzzer,LOW);
      delay(100);

}




 
void loop() {


  data.irtifa1= irtifa1_oku();//
  data.hiz1 = hiz1_oku();//
  data.xacisi = gyro_oku_x();//
  data.yacisi = gyro_oku_y();//
  data.pitch = pitch_oku();//
  data.ivme = ivme_oku(); 
  data.konum_enlem = konum_enlem_oku();
  data.konum_boylam = konum_boylam_oku();
  data.konum_yukseklik = konum_yukseklik_oku();


      
       /*EEPROM.writeFloat(eepitch, data.pitch);
      eepitch += sizeof(float);

      EEPROM.writeFloat(eeyacisi, data.yacisi);
      eeyacisi += sizeof(float);

      EEPROM.writeFloat(eexacisi, data.xacisi);
      eexacisi += sizeof(float);

      EEPROM.writeFloat(eeirtifa, data.irtifa1);
      eeirtifa += sizeof(float);

      EEPROM.writeFloat(eehiz, data.hiz1);
      eehiz += sizeof(float);*/               


  /*Serial.print("Irtifa: ");
  Serial.println(data.irtifa1);

  Serial.print("Hiz: ");
  Serial.println(data.hiz1);

  Serial.print("Gyro x: ");
  Serial.println(data.xacisi);

  Serial.print("Gyro y: ");
  Serial.println(data.yacisi);

  Serial.print("Pitch: ");
  Serial.println(data.pitch);

  Serial.print("Ivme: ");
  Serial.println(data.ivme);

  Serial.print("Enlem: ");
  Serial.println(data.konum_enlem,6);

  Serial.print("Boylam: ");
  Serial.println(data.konum_boylam,6);

  Serial.print("GPS Yukseklik: ");
  Serial.println(data.konum_yukseklik);

  Serial.println("///////////////////////////////////////////////////////");*/



  hiz_okey(data.hiz1);
  suruklenme_parasut( data.xacisi,data.yacisi, data.hiz1, data.pitch,data.irtifa1); 
  ana_parasut(data.irtifa1, data.suruklenmeParasut);
  

  time11 = millis(); 
  if (time11-time22 > 1000){ 
  ResponseStatus rs = e32ttl.sendFixedMessage(0, 20, 23, &data, sizeof(Signal));
  time22 = time11;
      }



}


  float irtifa1_oku(){
    irtifa_1 =bmp.readAltitude(1013.25)-referans1;  
    float filtreli_irtifa_1 = irtifa_1KalmanFilter.updateEstimate(irtifa_1);  
  return filtreli_irtifa_1; 
  }


  float hiz1_oku(){
    yeniZaman1 = millis(); 
    ilk_irtifa_1 =irtifa1_oku(); 
    if (yeniZaman1-eskiZaman1 > 100){ 
      son_irtifa_1 = irtifa1_oku(); 
      eskiZaman1 = yeniZaman1;
      }
    hesaplanan_hiz_1 = (son_irtifa_1 - ilk_irtifa_1)*10; 
  return hesaplanan_hiz_1; 
  }


  float gyro_oku_x(){
  mpu6050.update();
  float filtreli_acix =  xacisiKalmanFilter.updateEstimate(mpu6050.getAngleX());
  return filtreli_acix;  
  }

  float gyro_oku_y(){
  mpu6050.update();
  float filtreli_aciy =  -1*yacisiKalmanFilter.updateEstimate(mpu6050.getAngleY());
  return filtreli_aciy;  
  }


  float pitch_oku(){
  float pitch = myAcc.getPitch();  
  float filtreli_pitch = -1* pitchKalmanFilter.updateEstimate(pitch);
  return filtreli_pitch;
  }


  float ivme_oku(){  
  xyzFloat g = myAcc.getGValues();
  float ivme_x = g.x;
  float filtreli_ivme_x =  ivmeKalmanFilter.updateEstimate(ivme_x);
   return filtreli_ivme_x;
  }
  

  float konum_enlem_oku(){
    smartdelay(100);
    float enlem  = gps.location.lat();
  return enlem;
  }


  float konum_boylam_oku(){
    smartdelay(100);
    float boylam = gps.location.lng(); 
 
  return boylam;
  }


  float konum_yukseklik_oku(){
    smartdelay(100);
    int alt = gps.altitude.meters();
  return alt;
  }


  static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);}

  float hiz_okey(float olculen_hiz){
    if(!hiz_durum){
      if(olculen_hiz > 30)
       hiz_durum = true;
    }
  } 
    

  float suruklenme_parasut(float hesaplanan_aci_x,float hesaplanan_aci_y, float hesaplanan_hiz,float hesaplanan_pitch,float hesaplanan_irtifa){ 
     if(hiz_durum){
     if(hesaplanan_irtifa > 2500  ){

       ///Test esnasında kullanılan kısım
       ///digitalWrite(12,HIGH);  
       ///delay(100);
       ///digitalWrite(12,LOW);
       ///delay(100);
    if(!suruklenme_parasutu_acildi_mi){
  

   if(hesaplanan_aci_y < 20 ||hesaplanan_aci_y > 130  ){ //-10 map yapılabilir
        a=1; 
    }
    
   if(hesaplanan_aci_x > 55||hesaplanan_aci_x < -40 ){ // konuşulsun
        b=1; 
    }
   if(irtifa1_oku()>=2500){
      if(hesaplanan_hiz <= 0){   
        c=1;
      }
    }
 
   if(hesaplanan_pitch < 30){ //-30 olabilir//20 olabilir
        d=1;
    }
      if( a+b+c+d >= 2 ){
        
       if (!atesleme1) {
       digitalWrite(32,HIGH);
       delay(1000);
       digitalWrite(32,LOW);
       delay(100);
       atesleme1 = true;
       data.suruklenmeParasut = true;
       //Serial.print("suruklenme parasutu açıldı");
       }   
      
  return data.suruklenmeParasut;
  }

  }}}}

  float ana_parasut(float okunan_irtifa1,bool suruklenme){
    if(suruklenme){
    if(!ana_parasut_acildi_mi){
         if(okunan_irtifa1 <600 ){
      data.anaParasut = true;
      if (!atesleme2) {
        digitalWrite(33,HIGH);
        delay(1500);
        digitalWrite(33,LOW);
        delay(100);
       atesleme2 = true;
       
        
       }   
        }
      
  
    return data.anaParasut;
}
}
}