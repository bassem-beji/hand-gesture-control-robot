#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <stdio.h>
#include<vector>
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0
#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3
uint8_t masterMacAddress[] = {0xFC,0xF5,0xC4,0x2F,0xA3,0x04}; 

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};

std::vector<MOTOR_PINS> motorPins = 
{
  {16, 17, 22, 4},  //BACK_RIGHT_MOTOR
  {18, 19, 23, 5},  //BACK_LEFT_MOTOR
  {26, 27, 14, 6},  //FRONT_RIGHT_MOTOR
  {33, 25, 32, 7},  //FRONT_LEFT_MOTOR   
};

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
 // This is signal timeout in milli seconds. We will reset the data if no signal
#define SIGNAL_TIMEOUT 1000 
unsigned long lastRecvTime = 0;

typedef struct PacketData
{
   float xAxisValue;
  float yAxisValue;           
  float zAxisValue; 
}PacketData;

PacketData receiverData;  



void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}




// les fonctions 
 void OnDataRecv();       
 void printMAC();
 void rotateMotor();

void rotateMotor(int motorNumber, int motorSpeed)
{
  if (motorSpeed < 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);    
  }
  else if (motorSpeed > 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);      
  }
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}




void processCarMovement(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:   
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);                  
      break;
    case BACKWARD:    
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
    case LEFT:      
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
    case RIGHT:          
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
    case FORWARD_LEFT:  
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break; 
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  }}


void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);

  memcpy(&receiverData, incomingData, sizeof(receiverData));
  Serial.println("donnees recues");  
  Serial.print("X");
  Serial.println(receiverData.xAxisValue);
  Serial.print("Y: ");
  Serial.println(receiverData.yAxisValue);
  Serial.print("Z: ");
  Serial.println(receiverData.zAxisValue);
  Serial.println();

/*
  if ( receiverData.xAxisValue < 35 && receiverData.yAxisValue < 35)
  {
    processCarMovement(FORWARD_LEFT);    
  }
  else if ( receiverData.xAxisValue > 65 && receiverData.yAxisValue < 35)
  {
    processCarMovement(FORWARD_RIGHT);    
  } 
  else if ( receiverData.xAxisValue < 35 && receiverData.yAxisValue > 65)
  {
    processCarMovement(BACKWARD_LEFT);    
  }
  else if ( receiverData.xAxisValue > 65 && receiverData.yAxisValue > 65)
  {
    processCarMovement(BACKWARD_RIGHT);    
  }  
  
  else if (receiverData.zAxisValue > 65)
  {
    processCarMovement(TURN_RIGHT);
  }
  else if (receiverData.zAxisValue < 35)
  {
    processCarMovement(TURN_LEFT);
  }




  else if (receiverData.yAxisValue < 35)
  {
    processCarMovement(FORWARD);  
  }
  else if (receiverData.yAxisValue > 65)
  {
    processCarMovement(BACKWARD);     
  }
  else if (receiverData.xAxisValue >65)
  {
    processCarMovement(RIGHT);   
  }
  else if (receiverData.xAxisValue < 35)
  {
    processCarMovement(LEFT);    
  } 
  else
  {
    processCarMovement(STOP);     
  }

*/
if (receiverData.yAxisValue < -30)
  {
    processCarMovement(FORWARD);  
  }
 else if (receiverData.yAxisValue > 30)
  {
    processCarMovement(BACKWARD);     
  }
    else if (receiverData.zAxisValue >10 && -20<receiverData.yAxisValue<20 && -20<receiverData.xAxisValue<20)
  {
    processCarMovement(TURN_RIGHT);  
  }  
  else if (receiverData.zAxisValue < -10  && -20<receiverData.yAxisValue<20 && -20<receiverData.xAxisValue<20)
  {
    processCarMovement(TURN_LEFT);    
  } 
 
  /*
  if ( receiverData.xAxisValue < -30 && receiverData.yAxisValue < -30)
  {
    processCarMovement(FORWARD_LEFT);    
  }
  else if ( receiverData.xAxisValue > 30 && receiverData.yAxisValue < -30)
  {
    processCarMovement(FORWARD_RIGHT);    
  } 
  else if ( receiverData.xAxisValue < -30 && receiverData.yAxisValue > 30)
  {
    processCarMovement(BACKWARD_LEFT);    
  }
  else if ( receiverData.xAxisValue > 30 && receiverData.yAxisValue > 30)
  {
    processCarMovement(BACKWARD_RIGHT);    
  }  */
  
  
  
   else if (receiverData.xAxisValue > 30)
  {
    processCarMovement(RIGHT); 
  }
  else if (receiverData.xAxisValue < -30)
  {
    processCarMovement(LEFT); 
  }
  //STOP
 else
  {
    processCarMovement(STOP);     
  }
  
  lastRecvTime = millis();   

}



void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);  
    //Set up PWM for motor speed
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);  
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);     
    rotateMotor(i, STOP);  
  }
}



esp_now_peer_info_t peerInfo;

void setup() {
  setUpPinModes();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  
  if (esp_now_init() == ESP_OK) 
  {
    Serial.println("Inistialisation ESP_NOW OK");
  }
  else
  {
    Serial.println("Erreur d'inistialisaton ESP_NOW");
    ESP.restart();
  }
 

  
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
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

  
  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
  
  //Check Signal lost.
  
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    processCarMovement(STOP); 
  }
}
