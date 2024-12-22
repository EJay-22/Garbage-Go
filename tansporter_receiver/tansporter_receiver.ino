/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

#define RightMotor 2  // Relay control pin
#define LeftMotor 4   // Relay control pin

#define leftIR 7   // Left IR sensor
#define rightIR 8  // Right IR sensor

bool overfill = 0;
bool moving = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int sonar_value;
  int overfill_value;
} struct_message;

// Create a struct_message called myData
struct_message bin_level;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&bin_level, incomingData, sizeof(bin_level));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Sonar Value: ");
  Serial.println(bin_level.sonar_value);
  Serial.print("overfill Value: ");
  Serial.println(bin_level.overfill_value);
  Serial.print("Overfill?: ");
  bool overfill = bin_level.sonar_value <= bin_level.overfill_value;
  Serial.println(overfill);
  if (overfill){
    moving = 1;
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  
  pinMode(RightMotor, OUTPUT);      // Relay output (right)
  pinMode(LeftMotor, OUTPUT);       // Relay output (left)

  pinMode(leftIR, INPUT);           // IR sensors as input
  pinMode(rightIR, INPUT);          // IR sensors as input

  digitalWrite(RightMotor, LOW);    // Motors OFF initially
  digitalWrite(LeftMotor, LOW);     // Motors OFF initially

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

}

 
void loop() {
  if (overfill || moving){
    trace();
  }
}
void right() {
  moving = 1;
  digitalWrite(RightMotor, HIGH);
  digitalWrite(LeftMotor, LOW);
}

void left() {
  moving = 1;
  digitalWrite(RightMotor, LOW);
  digitalWrite(LeftMotor, HIGH);
}

void forward() {
  moving = 1;
  digitalWrite(RightMotor, HIGH);
  digitalWrite(LeftMotor, HIGH);
}

void stop() {
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);
  moving = 0;
}

void trace() {
  // Read IR sensor values
  int left_sense = digitalRead(leftIR);
  int right_sense = digitalRead(rightIR);

  // Debugging - print sensor values
  Serial.print("Left: ");
  Serial.print(left_sense);
  Serial.print(" | Right: ");
  Serial.println(right_sense);

  // Logic for line-following based on Active LOW sensors
  if (right_sense == LOW && left_sense == HIGH) {   // Right sensor detects black (LOW)
    right();
  } 
  else if (left_sense == LOW && right_sense == HIGH) {  // Left sensor detects black (LOW)
    left();
  } 
  else if (left_sense == LOW && right_sense == LOW) {   // Both sensors detect black (LOW)
    forward();
  } 
  else {  // Both sensors on white (HIGH)
    stop();
  }
}