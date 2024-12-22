/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <NewPing.h>

#define TRIGGER_PIN 21
#define ECHO 20
#define MAX_DISTANCE 400

const int time_between_pings = 50;
const int time_between_sends = 1000; //in ms
const int threshold = 10;
int distance;
unsigned long trigger_millis;
unsigned long send_millis;
unsigned long curr_millis;

NewPing sonar(TRIGGER_PIN, ECHO, MAX_DISTANCE);

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x24, 0xec, 0x4a, 0x07, 0x5b, 0x4c};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int sonar_value;
  int overfill_value;
} struct_message;

// Create a struct_message called myData
struct_message bin_level;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {

  //overfill value
  bin_level.overfill_value = 20; //cm from the top of the bin

  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  curr_millis = millis();
  if (curr_millis - trigger_millis >= time_between_pings){
    distance = sonar.ping_cm();
    trigger_millis = curr_millis;
  }
  // Set values to send
  bin_level.sonar_value = distance;

  // Send message via ESP-NOW
  if (curr_millis - send_millis >= time_between_sends){
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &bin_level, sizeof(bin_level));
    send_millis = curr_millis;
    if (result == ESP_OK) {
    Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  delay(1000);
}