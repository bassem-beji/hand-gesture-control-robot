#include "MPU9250.h"
#include "Arduino.h"
#include "BluetoothSerial.h"
#include "string.h"
#include <wire.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t receiverAddress[] = {0x0C,0xB8,0x15,0xC1,0x1C,0xB8};  //adresse mac de la carte ESP32 receiver

 
#define ENCODER_DO_NOT_USE_INTERRUPTS
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


BluetoothSerial SerialBT;

typedef struct PacketData 
{
  float xAxisValue;
  float yAxisValue;           //les angles el 7achetna bihom 
  float zAxisValue;  
}PacketData;
PacketData data;     //el variable data hia eli bech nab3atha lel esp32 receiver






// Functions
void getdata();
void onDataSent();
// mpu object
MPU9250 mpu;
// angles of the robot
float pitch, yaw, roll ,z ;
int k=0 ;




/*  
float  rollavr=0 ,pitchavr=0 ,yawavr=0;                                     //les fonctions de houssem 
void MPU_Angles_Avr(){
for (int i = 0; i < 2000; i++)
{
mpu.update();
float x_angle = mpu.getRoll();
float y_angle = mpu.getPitch();
float z_angle = mpu.getYaw();
rollavr += (x_angle/2000);
pitchavr += (y_angle/2000);
yawavr += (z_angle/2000);
delay(1);
}}

void Measured_Roll_Pitch_Yaw(){
if (mpu.update()) {
roll = mpu.getRoll() - rollavr -2;
pitch = mpu.getPitch() - pitchavr +2;
yaw = mpu.getYaw() - yawavr;

Serial.print("Roll: |");
Serial.print(roll);
Serial.print("Pitch: |");
Serial.print(pitch);
Serial.print(" Yaw:  | ");
Serial.print(yaw);
Serial.print("\n");

}
}*/


// implementation des fonctions 

/*
void getinitz(){
  zInitiale = mpu.getYaw();
  k=k+1;
}*/

void getdata() {
  /*
  if (k<30)
  {
       getinitz();
       delay(5);
  }
  static unsigned long lastET = millis(); 
  if ((millis() - lastET) > 5000) {
  zInitiale = mpu.getYaw();
  }
  yaw = mpu.getYaw(zInitiale);

  */
 
 z=mpu.getGyroZ();
     if(z<-20) {   
      while (true)
      {
        
      yaw = -70+z;
      static unsigned long l = millis(); 
      if ((millis() - l) > 50){
        break;
      }}}

      else if(z>20) {   
      while (true)
      {
      yaw = 70+z;
      static unsigned long ls = millis(); 
      if ((millis() - ls) > 50){
        break;
      }}
     }
     else {
      yaw =0 ;
     }
/*
      z=mpu.getGyroZ();
     if(z<-10) {  
      k=0;
        if (k<10){ 
            yaw = -80+z;
            k++;
            delay(5);
      }}

      else if(z>10) {   
       */
  roll = mpu.getRoll();
  pitch = mpu.getPitch();
  Serial.print(" roll pitch yaw  ");
  Serial.print(roll);
  Serial.print(" | ");
  Serial.print(pitch);
  Serial.print(" | ");
  Serial.println(yaw);
   Serial.print(" | ");
 
   delay(5);
   
}






void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS){
    Serial.println("livraision OK");
  }
  else{
    Serial.println("probleme de livraision");
  }

}

esp_now_peer_info_t peerInfo;
 
void setup() {
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
    Wire.begin();
   


  while (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    delay(500);
  }
  //a5er ezyada
  
  if (esp_now_init() == ESP_OK) 
  {
    Serial.println("Inistialisation ESP_NOW OK");
  }
  else
  {
    Serial.println("Erreur d'inistialisaton ESP_NOW");
    ESP.restart();
  }
 
  
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
       
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK)
  {
    Serial.println("fonction peer OK");
  }
  else
  {
    Serial.println("Erreur fonction Peer");
  }   

  esp_now_register_send_cb(OnDataSent);
  
}

void loop() { 
  if (mpu.update()) {
  static unsigned long lastEventTime = millis(); 
  if ((millis() - lastEventTime) > 25) {
   
    
    lastEventTime = millis();
      /*
    x= mpu.getGyroX() ;
     y=mpu.getGyroY() ;
     z=mpu.getGyroZ();
     if(z<-5 || z>5) {
      delay(5);
     }
     z2=mpu.getGyroBiasZ();
     Serial.print(x);
  Serial.print(" | ");
  Serial.print(y);
  Serial.print(" | ");
  Serial.println(z);
  Serial.print(" | ");
  Serial.println(z2);*/
      getdata();
       data.xAxisValue =  roll;
       data.yAxisValue =  pitch;
       data.zAxisValue = -yaw ;
         
       
  
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &data, sizeof(data));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }  
  }}
  }