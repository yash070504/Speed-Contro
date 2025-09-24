#include <Encoder.h>
#include <LiquidCrystal.h>

const int rs = 13, en = 12, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int potPin = A2;    
// Define motor control pins
const int motorPin1 = 9;
int setpoint = 0;
const int motorPin2 = 10;
const int motorPWMPin = 11;  // PWM for speed control

int i = 0; 
int d = 0;
int laste = 0;

// Define encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;
Encoder encoder(encoderPinA, encoderPinB);

long previousEncoderPosition = 0;
unsigned long previousMillis = 0;
float currentRPM = 0;

// Maximum RPM and motor control variables
const int maxRPM = 210;  // Example max RPM
float referenceRPM = 0;    // RPM set by DIP switch / potentiometer
int motorSpeed = 0;        // Motor speed in PWM (0-255)

// Encoder details
const int encoderTicksPerRevolution = 360; // Adjust according to your encoder specs

void setup() {
  lcd.begin(16, 2); // Initialize the LCD
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // Set up serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Set motor direction (forward)
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  int potValue = analogRead(potPin);
  setpoint = map(potValue, 0, 1023, 0, 360);
  referenceRPM = setpoint;

  // Measure the speed using the encoder
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis) / 1000.0;  // Convert to seconds
  
  long currentEncoderPosition = encoder.read();
  long encoderTicks = currentEncoderPosition - previousEncoderPosition;
  
  // Calculate RPM
  currentRPM = abs(encoderTicks / (float)encoderTicksPerRevolution) / (elapsedTime / 60.0);
  
  previousEncoderPosition = currentEncoderPosition;
  previousMillis = currentMillis;

  // PID-like control
  float error = referenceRPM - currentRPM;
  i += error * 0.1; 
  d = (error - laste) / 0.1;
  laste = error;

  motorSpeed += (int)(error * 0.5 + i * 0.00 + d * 0.0);
  motorSpeed = constrain(motorSpeed, 0, 255);
  
  analogWrite(motorPWMPin, motorSpeed);

  // Print debugging info
  Serial.print(referenceRPM);
  Serial.print(",");
  Serial.println(currentRPM);

  // Display desired and current RPM on LCD
  lcd.setCursor(0, 0);
  lcd.print("Des: ");
  lcd.print(referenceRPM);

  lcd.setCursor(0, 1);
  lcd.print("Cur: ");
  lcd.print(currentRPM);
  lcd.print("   ");  // Clear trailing chars

  delay(1000);
}
