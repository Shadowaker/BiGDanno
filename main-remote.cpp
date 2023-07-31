#ifdef REMOTE

#include <string>  
#include <iostream> 
#include <sstream> 

#include <math.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <WiFi.h>
#include <batteryMonitor.h>
#include <ledUtility.h>
#include "esp_log.h"
#include "mac.h"

# define DEBUG_ENABLED false
//# define DEBUG_PRINTS


static const char *TAG = "MAIN";
//------------ turn on generic serial printing

//data that will be sent to the receiver

typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t packetArg1;     //  [button, value] || 0 -> 102030
  int16_t packetArg2;     //  lever value
  int16_t packetArg3;     //  misc
}
packet_t;


packet_t sentData;
packet_t recData;


//---------------------------------------ESP_NOW Variables


String success;
esp_now_peer_info_t peerInfo;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recData, incomingData, sizeof(recData));
}




//---------------------------------------HARDWARE DEPENDANT Variables
// one ifdef case per hardware to speed up modularity of the code

//RAC standard remote
const int steerPot = 7;
const int accPot = 10;
const int leverPot = 8;
//const int trimPot = 39;

const int rightBtn = 2;
const int leftBtn = 4;
const int topBtn = 5;
//const int lowSwitch = 32;
//const int topSwitch = 25;
LedUtility Led(21);

//customisable vars
int analogRes = 10;
int analogReadMax = (1 << analogRes)-1;


//variables for the sketch
int leverValue = 0;

unsigned long current_time = 0;


void setup() {
  //store_values(); // uncomment only to initialize mem
  analogReadResolution(analogRes);
  analogSetAttenuation(ADC_11db);
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(topBtn, INPUT_PULLUP);
  //pinMode(lowSwitch, INPUT_PULLUP);
  //pinMode(topSwitch, INPUT_PULLUP);
  Led.init();
  Led.setBlinks(1,150);
  delay(2000);
#ifdef DEBUG_PRINTS
  Serial.begin(115200);
  Serial.println("RAC GENERIC BOT");
#endif


  //---------------------------------------ESP NOW setup
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  char macStr[18];
  Serial.print("Packet from: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           peerInfo.peer_addr[0], peerInfo.peer_addr[1], peerInfo.peer_addr[2], peerInfo.peer_addr[3], peerInfo.peer_addr[4], peerInfo.peer_addr[5]);
  Serial.print("sending to: ");
  Serial.println(macStr);
  esp_now_register_recv_cb(OnDataRecv);
  Led.setBlinks(0);
  Led.ledOn();
}

void loop() {
  //read pots values
  int strValue = analogRead(steerPot);
  delay(3);
  int accValue = analogRead(accPot);
  delay(3);
  int leverValue = analogRead(leverPot);
  delay(3);
  current_time = millis(); 
  bool leftValue = !digitalRead(rightBtn);
  bool rightValue = !digitalRead(leftBtn);
  bool topValue = !digitalRead(topBtn);
  
  // vvvvvv CODE vvvvvv // UwU

  static int  i = 0;   
  static int  t = 0;
  static int  j = 0;

  /*
  int normAcc = 0;
  if (accValue <= 360)
    normAcc = map(accValue, 0, 360, -512, 0); 
  else if (accValue >= 560)
    normAcc = map(accValue, 560, 1023, 0, 512);
  int normStr = 0;
  if (strValue <= 360)
    normStr = map(strValue, 0, 360, -512, 0); 
  else if (strValue >= 560)
    normStr = map(strValue, 560, 1023, 0, 512);

  int ray = sqrt(normAcc * normAcc + normStr * normStr);  */

  int newLeverValue = map(leverValue, 700, 300, 930, 100);
  
  if (DEBUG_ENABLED)
  {
    if (i == 85) {
      Serial.println("-------------------------------------\n");
      Serial.printf("Steer: %d\n", strValue);
      Serial.printf("Acc: %d\n", accValue);
      //Serial.printf("normSteer: %d\n", normStr);
      //Serial.printf("normAcc: %d\n", normAcc);
      //Serial.printf("ray: %d\n", ray);
      Serial.printf("leverValue: %d\n", leverValue);
      Serial.printf("newLeverValue: %d\n", newLeverValue);
      Serial.printf("rValue: %d - lValue: %d - topValue: %d\n",  rightValue, leftValue, topValue);
      i = 0;
    }
    else {
      i++;
    }
  }
  if (t == 1)
  {
    if (j == 12)
    {
      j = 0;
      t = 0;
    }
    else {
      j++;
    }
  }

  // Stop range 360 - 560
  sentData.packetArg1 = topValue;
  sentData.packetArg2 = newLeverValue;
  sentData.packetArg3 = 0;
  sentData.speedmotorLeft = 0;
  sentData.speedmotorRight = 0;
  // 650 300

/*  
  if (ray > 512)
    ray = 512;
  if (ray < -512)
    ray = -512;

  if (normAcc > 0) {
    if (normStr > 0) {
      sentData.speedmotorLeft = ray;
      sentData.speedmotorRight = ray - (ray / 2);
    }
    else {
      sentData.speedmotorLeft = -ray + (ray / 2);
      sentData.speedmotorRight = ray;
    }
  }
  else if (normAcc < 0) {
    if (normStr < 0) {
      sentData.speedmotorLeft = -ray;
      sentData.speedmotorRight = ray - (ray / 2);
    }
    else {
      sentData.speedmotorLeft = -ray + (ray / 2);
      sentData.speedmotorRight = ray;
    }
  }
  else {
    sentData.speedmotorLeft = 0;
    sentData.speedmotorRight = 0;
  }
*/

  if (accValue == 0)
  {
    if (rightBtn) {
      sentData.speedmotorLeft = -500;
      sentData.speedmotorRight = 500;
    }
    else {
      sentData.speedmotorLeft = -400;
      sentData.speedmotorRight = 400;
    }
  }
  else if (accValue == 1023)
  {
    if (rightBtn) {
      sentData.speedmotorLeft = 500;
      sentData.speedmotorRight = -500;
    }
    else {
      sentData.speedmotorLeft = 400;
      sentData.speedmotorRight = -400;
    }
  }

  if (strValue == 0)
  {
    if (rightBtn) {
      sentData.speedmotorLeft = -500;
      sentData.speedmotorRight = -500;
    }
    else {
      sentData.speedmotorLeft = -400;
      sentData.speedmotorRight = -400;
    }
  }
  else if (strValue == 1023)
  {
    if (rightBtn) {
      sentData.speedmotorLeft = 500;
      sentData.speedmotorRight = 500;
    }
    else {
      sentData.speedmotorLeft = 400;
      sentData.speedmotorRight = 400;
    }
  }
  

  if (leftValue) {
    t = 1;
  }

  if (t == 1)
  {
    sentData.speedmotorLeft = 512;
    sentData.speedmotorRight = 512;
  }
  
  // Send data to robot

  
  // -------------------------------------------- //
  esp_err_t result = -1;
  result = esp_now_send(robotAddress, (uint8_t *) &sentData, sizeof(sentData));
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
  delay(10);
}
#endif