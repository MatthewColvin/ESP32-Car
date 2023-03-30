/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0xE0, 0x5A, 0x1B, 0x5F, 0x5D, 0x44 };

// Define remote buttons and analog inputs
const byte joystickXPin = 39;    // Used as an ADC
const byte joystickYPin = 34;    // Used as an ADC
const byte joystickBtnPin = 36;  // Used as a digital input. These buttons are active-high
const byte leftBtnPin = 35;      // Used as a digital input.

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int a;   // X axis
  int b;   // Y axis
  bool c;  // Right button
  bool d;  // Left button
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Define X and Y axis deadband.
const int xAxisDeadband = 200;
const int yAxisDeadband = 200;

int xAxisOffset = 0;
int yAxisOffset = 0;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  //set the ADC resolution to 12 bits (0-4096)
  analogReadResolution(12);
  // Initialize buttons
  pinMode(36, INPUT);  // Right button
  pinMode(35, INPUT);  // Left button

  // Measure the center of the joystick to use as a zero offset.
  xAxisOffset = analogRead(39);
  yAxisOffset = analogRead(34);


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  int xAxisMapped = 0;
  int yAxisMapped = 0;

  int xAxisJoy = analogRead(39) - xAxisOffset;
  int yAxisJoy = analogRead(34) - yAxisOffset;

  if (xAxisJoy > xAxisDeadband) // Scale x-axis inputs to +/- 100 and remove the deadband in the middle.
  {
    xAxisMapped = map(xAxisJoy, 200, 2048, 0, 100);
  } 
  else if (xAxisJoy < -xAxisDeadband) 
  {
    xAxisMapped = map(xAxisJoy, -200, -2048, 0, -100);
  } 
  else xAxisMapped = 0;


  if (yAxisJoy > yAxisDeadband)  // Scale y-axis inputs to +/- 100 and remove the deadband in the middle.
  {
    yAxisMapped = map(yAxisJoy, 200, 2048, 0, 100);
  } 
  else if (yAxisJoy < -yAxisDeadband) 
  {
    yAxisMapped = map(yAxisJoy, -200, -2048, 0, -100);
  } 
  else yAxisMapped = 0;

  Serial.print("X-axis: ");
  Serial.print(analogRead(39));
  Serial.print("\t");
  Serial.print(xAxisJoy);
  Serial.print("\t");
  Serial.print(xAxisMapped);
  Serial.println("\t");

  Serial.print("Y-axis: ");
  Serial.print(analogRead(34));
  Serial.print("\t");
  Serial.print(yAxisJoy);
  Serial.print("\t");
  Serial.println(yAxisMapped);
  Serial.println("\t");

  // Set values to send
  myData.a = xAxisMapped;
  myData.b = yAxisMapped;
  myData.c = digitalRead(36);
  myData.d = digitalRead(35);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
  delay(100);
}