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

const byte rightMotorAPin = 15;  // Define motor pins. Each A/B pair controls the direction of one motor
const byte rightMotorBPin = 2;   // PWM on one of the A/B pins will control the "speed"
const byte leftMotorAPin = 16;
const byte leftMotorBPin = 17;

const byte rightMotorAChannel = 0;  // PWM channels (timers) used by the motors
const byte rightMotorBChannel = 1;
const byte leftMotorAChannel = 2;
const byte leftMotorBChannel = 3;

const byte redLED = 19;  // Pins the LED are connected to. Active-high.
const byte greenLED = 18;
const byte blueLED = 5;

const byte servoPin = 13;  // Pin the servo is connected to.

const int freq = 500;      // Set timer to run at 500Hz
const int resolution = 8;  // Set timer to 8-bit

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int a;   // X axis
  int b;   // Y axis
  bool c;  // Right button
  bool d;  // Left button
} struct_message;

int rightMotor = 0;
int leftMotor = 0;


// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("X-axis: ");
  // Serial.println(myData.a);
  // Serial.print("Y-axis: ");
  // Serial.println(myData.b);
  // Serial.print("Right Button: ");
  // Serial.println(myData.c);
  // Serial.print("Left Button: ");
  // Serial.println(myData.d);
  // Serial.println();

  rightMotor = constrain((myData.b - myData.a), -127, 127);  // Mix forward/backward and left/right inputs.
  leftMotor = constrain((myData.b + myData.a), -127, 127); // These need to be constrained to avoid overflowing the PWM timer. 

  Serial.print("Right Motor: ");
  Serial.println(rightMotor);
  Serial.print("Left Motor: ");
  Serial.println(leftMotor);
  Serial.println();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(rightMotorAPin, OUTPUT);  // Configure motor output pins
  pinMode(rightMotorBPin, OUTPUT);
  pinMode(leftMotorAPin, OUTPUT);
  pinMode(leftMotorBPin, OUTPUT);

  pinMode(redLED, OUTPUT);  // Initialize LED pins as outputs
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  digitalWrite(rightMotorAPin, LOW);  // Initialize motor output states
  digitalWrite(rightMotorBPin, LOW);
  digitalWrite(leftMotorAPin, LOW);
  digitalWrite(leftMotorBPin, LOW);

  ledcSetup(rightMotorAChannel, freq, resolution);  // Configure PWM channels (timers) to the specified frequency and resolution
  ledcSetup(rightMotorBChannel, freq, resolution);
  ledcSetup(leftMotorAChannel, freq, resolution);
  ledcSetup(leftMotorBChannel, freq, resolution);

  ledcAttachPin(rightMotorAPin, rightMotorAChannel);  // Attach a pin to a PWM channel
  ledcAttachPin(rightMotorBPin, rightMotorBChannel);
  ledcAttachPin(leftMotorAPin, leftMotorAChannel);
  ledcAttachPin(leftMotorBPin, leftMotorBChannel);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  if (rightMotor > 0) {
    ledcWrite(rightMotorAChannel, abs(rightMotor)*2);  // PWM channel
    ledcWrite(rightMotorBChannel, 0);                // Hold this channel low
  } 
  else if (rightMotor < 0) {
    ledcWrite(rightMotorAChannel, 0);                // Hold this channel low
    ledcWrite(rightMotorBChannel, abs(rightMotor)*2);  // PWM channel
  } 
  else {
    ledcWrite(rightMotorAChannel, 0);  // Stop the motor. Hold both sides low.
    ledcWrite(rightMotorBChannel, 0);
  }

  if (leftMotor > 0) {
    ledcWrite(leftMotorAChannel, abs(leftMotor)*2);  // PWM channel
    ledcWrite(leftMotorBChannel, 0);                // Hold this channel low
  } 
  else if (leftMotor < 0) {
    ledcWrite(leftMotorAChannel, 0);                // Hold this channel low
    ledcWrite(leftMotorBChannel, abs(leftMotor)*2);  // PWM channel
  } 
  else {
    ledcWrite(leftMotorAChannel, 0);  // Stop the motor. Hold both sides low.
    ledcWrite(leftMotorBChannel, 0);
  }

  
}
