
const byte rightMotorAPin = 15;
const byte rightMotorBPin = 2;
const byte leftMotorAPin = 16;
const byte leftMotorBPin = 17;

const byte rightMotorAChannel = 0;
const byte rightMotorBChannel = 1;
const byte leftMotorAChannel = 2;
const byte leftMotorBChannel = 3;

const byte redLED = 19;
const byte greenLED = 18;
const byte blueLED = 5;

const byte servoPin = 13;

const int freq = 500;     // Set timer to run at 500Hz
const int resolution = 8;  // Set timer to 8-bit

void setup() {
  pinMode(rightMotorAPin, OUTPUT); // Configure motor output pins
  pinMode(rightMotorBPin, OUTPUT);
  pinMode(leftMotorAPin, OUTPUT);
  pinMode(leftMotorBPin, OUTPUT);

  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  digitalWrite(rightMotorAPin, LOW); // Initialize motor output states
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
}

 void loop() {

digitalWrite(redLED, HIGH);
ledcWrite(rightMotorAChannel,115); // Right Forward
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel,127); // Left Forward
ledcWrite(leftMotorBChannel, 0);
delay(2000);
digitalWrite(greenLED, HIGH);
ledcWrite(rightMotorAChannel, 0);
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel, 0);
ledcWrite(leftMotorBChannel, 0);
delay(500);
ledcWrite(rightMotorAChannel,127); // Right Forward
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel, 0); // left Reverse
ledcWrite(leftMotorBChannel, 127);
delay(425);
digitalWrite(blueLED, HIGH);
ledcWrite(rightMotorAChannel, 0);
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel, 0);
ledcWrite(leftMotorBChannel, 0);
delay(500);
ledcWrite(rightMotorAChannel,127); // Right Forward
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel, 0); // left Reverse
ledcWrite(leftMotorBChannel, 127);
delay(425);
digitalWrite(redLED, LOW);
digitalWrite(greenLED, LOW);
digitalWrite(blueLED, LOW);
ledcWrite(rightMotorAChannel, 0);
ledcWrite(rightMotorBChannel, 0);
ledcWrite(leftMotorAChannel, 0);
ledcWrite(leftMotorBChannel, 0);
delay(500);


// ledcWrite(leftMotorAChannel, 0); // left Reverse
// ledcWrite(leftMotorBChannel, 127);
// delay(500);
// ledcWrite(leftMotorAChannel, 0);
// ledcWrite(leftMotorBChannel, 0);
// delay(500);
// ledcWrite(leftMotorAChannel,127); // Left Forward
// ledcWrite(leftMotorBChannel, 0);
// delay(500);
// ledcWrite(leftMotorAChannel, 0);
// ledcWrite(leftMotorBChannel, 0);
// delay(500);


   
//   // increase the LED brightness
//   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
//     // changing the LED brightness with PWM
//     ledcWrite(rightMotorBChannel, dutyCycle);
//     ledcWrite(leftMotorAChannel, dutyCycle);
//     delay(3);
//   }

//   // decrease the LED brightness
//   for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
//     // changing the LED brightness with PWM
//     ledcWrite(rightMotorBChannel, dutyCycle);
//     ledcWrite(leftMotorAChannel, dutyCycle);
//     delay(3);
//   }
}
