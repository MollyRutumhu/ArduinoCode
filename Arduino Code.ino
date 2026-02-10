

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set LCD address and size (16 columns, 2 rows)

// Define input pins
const int buttonPin = 2;             // Push button pin for menu control
const int pidInputPin = A0;          // Analog pin for PID sensor feedback

// Define temperature sensor and LEDs pins
const int tempSensorPin = A1;        // Analog pin connected to TMP36 temp sensor
const int greenLed = 4;              // Green LED pin for low temp indication
const int yellowLed = 5;             // Yellow LED pin for medium temp indication
const int redLed = 8;                // Red LED pin for high temp warning

// Define ultrasonic sensor pins
const int trigPin = 6;               // Ultrasonic sensor trigger pin
const int echoPin = 7;               // Ultrasonic sensor echo pin

// Define motor and op-amp control pins
const int motorPin = 10;             // PWM pin to control motor speed
const int opAmpPin = 9;              // Additional PWM pin (not used here for output)

// Define timing variables to schedule tasks without delay()
unsigned long lastSensorTime = 0;
unsigned long lastPidTime = 0;
unsigned long lastTempTime = 0;
unsigned long lastLedBlinkTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastLogTime = 0;

// Define time intervals for each task (in milliseconds)
const unsigned long sensorInterval = 100;      // Ultrasonic sensor read interval
const unsigned long pidInterval = 10;          // PID update interval
const unsigned long tempInterval = 300;        // Temperature read interval
const unsigned long ledBlinkInterval = 50;     // Red LED blink interval for high temp
const unsigned long lcdInterval = 200;         // LCD update interval
const unsigned long logInterval = 500;         // Serial logging interval

// PID parameters tuning constants
const float Kp = 0.65918;  // Proportional gain
const float Ki = 28.2811;  // Integral gain
const float Kd = 0.0038411; // Derivative gain
const float sampleTime = 0.01; // Sample time in seconds

// Variables for PID controller calculations
float integral = 0;         // Integral term accumulator
float lastError = 0;        // Previous error value for derivative calculation
float pidOutput = 0;        // PID output value to control motor

// Variables for ultrasonic sensor processing
long pulseDuration;         // Duration of ultrasonic echo pulse
int distance = -1;          // Measured distance in centimeters
float setpointVoltage = 0;  // Setpoint voltage derived from distance for PID

int motorDuty = 0;          // PWM duty cycle value for motor (0-1023)
int redLedState = LOW;      // State for blinking red LED during over-temperature warning

// Variables for button states and debounce
int buttonState = HIGH;     // Current button state (using INPUT_PULLUP)
int lastButtonState = HIGH; // Previous button state for debounce logic
unsigned long lastButtonChange = 0; // Last time button input changed
const unsigned long debounceDelay = 10; // Debounce delay in milliseconds

void setup() {
  Serial.begin(9600);               // Start serial monitor at baud rate 9600

  // Set pin modes:
  pinMode(buttonPin, INPUT_PULLUP); // Button input with internal pull-up resistor
  pinMode(greenLed, OUTPUT);       // Green LED output
  pinMode(yellowLed, OUTPUT);      // Yellow LED output
  pinMode(redLed, OUTPUT);         // Red LED output
  pinMode(trigPin, OUTPUT);        // Ultrasonic trigger pin as output
  pinMode(echoPin, INPUT);         // Ultrasonic echo pin as input
  pinMode(motorPin, OUTPUT);       // Motor control as PWM output
  pinMode(opAmpPin, OUTPUT);       // Op-amp control pin (PWM output)

  lcd.init();                      // Initialize the LCD
  lcd.backlight();                 // Turn on LCD backlight

  // Show splash screen on LCD for 3 seconds
  lcd.setCursor(0, 0);
  lcd.print("PID CONTROL");
  lcd.setCursor(0, 1);
  lcd.print("TEMP + MOTOR");
  delay(3000);
  lcd.clear();

  // Configure Timer1 for fast PWM mode with no prescaler for max frequency PWM signal (pins 9 and 10)
  TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM12) | (1 << CS10);
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current system time in milliseconds

  // Read Ultrasonic Sensor every sensorInterval ms
  if (currentMillis - lastSensorTime >= sensorInterval) {
    lastSensorTime = currentMillis;

    // Trigger ultrasonic sensor: send 10us HIGH pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse duration (in microseconds), timeout after 30ms
    pulseDuration = pulseIn(echoPin, HIGH, 30000);

    if (pulseDuration > 0) {
      // Calculate distance in cm (speed of sound ~0.034 cm/us)
      distance = (pulseDuration * 0.034) / 2;
      distance = constrain(distance, 0, 30);

      // Calculate voltage setpoint for PID controller based on distance
      setpointVoltage = distance / 10.0;

      // Map distance to PWM duty cycle range (0-1023)
      motorDuty = map(distance, 0, 30, 0, 1023);
      motorDuty = constrain(motorDuty, 0, 1023);

      // Apply PWM signal to motor pin
      analogWrite(motorPin, motorDuty);
    }
    else {
      // If no echo, reset distance and stop motor
      distance = -1;
      setpointVoltage = 0;
      analogWrite(motorPin, 0);
    }
  }

  // PID controller calculations every pidInterval ms
  if (currentMillis - lastPidTime >= pidInterval) {
    lastPidTime = currentMillis;

    // Read current process variable from analog input
    int input = analogRead(pidInputPin);

    // Calculate setpoint in same voltage count scale as ADC
    float setpoint = (setpointVoltage * 1023) / 5.0;

    // Compute current error between setpoint and feedback
    float error = setpoint - input;

    // Calculate PID output terms
    float P = Kp * error;
    integral += Ki * sampleTime * error;
    integral = constrain(integral, 0, 1023);
    float D = Kd * (error - lastError) / sampleTime;
    lastError = error;

    // Total PID output constrained to PWM limits
    pidOutput = P + integral + D;
    pidOutput = constrain(pidOutput, 0, 1023);

    // Output PID value to hardware PWM register OCR1A (pin 9)
    OCR1A = (int)pidOutput;
  }

  // Temperature reading and LED indication every tempInterval ms
  if (currentMillis - lastTempTime >= tempInterval) {
    lastTempTime = currentMillis;

    int sensorVal = analogRead(tempSensorPin);
    float voltage = sensorVal * (5.0 / 1023.0);
    float temperature = (voltage - 0.5) * 100;  // Convert voltage to °C

    // Turn on different LEDs depending on temperature range
    if (temperature < 28) {
      digitalWrite(greenLed, HIGH);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, LOW);
    }
    else if (temperature < 32) {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, LOW);
    }
    else if (temperature <= 35) {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, HIGH);
    }
    else {
      // Blink red LED if temperature is above 35°C
      if (currentMillis - lastLedBlinkTime >= ledBlinkInterval) {
        lastLedBlinkTime = currentMillis;
        redLedState = !redLedState;  // Toggle LED state
        digitalWrite(redLed, redLedState);
      }
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, LOW);
    }
  }

  // Button input reading with debounce logic
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) lastButtonChange = currentMillis;
  if ((currentMillis - lastButtonChange) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {  // On button press
        menu++;
        if (menu > 3) menu = 0;  // Cycle menus 0-3
        lcd.clear();             // Clear LCD when menu changes
      }
    }
  }
  lastButtonState = reading;

  // Update LCD display every lcdInterval ms
  if (currentMillis - lastLcdTime >= lcdInterval) {
    lastLcdTime = currentMillis;

    switch (menu) { //switch case fot the menu 
      case 0:  // Show distance and setpoint
        lcd.setCursor(0, 0);
        if (distance >= 0) {
          lcd.print("Dist: ");
          lcd.print(distance);
          lcd.print(" cm   ");
          lcd.setCursor(0, 1);
          lcd.print("S: ");
          lcd.print(setpointVoltage, 1);
          lcd.print(" V   ");
        }
        else {
          lcd.print("No echo detected");
          lcd.setCursor(0, 1);
          lcd.print("Motor stopped   ");
        }
        break;

      case 1:  // Show temperature (Celsius and Fahrenheit)
        int tempVal = analogRead(tempSensorPin);
        float tempV = tempVal * (5.0 / 1023.0);
        float tempC = (tempV - 0.5) * 100;
        float tempF = tempC * 9 / 5 + 32;

        lcd.setCursor(0, 0);
        lcd.print("Temp:            ");
        lcd.setCursor(0, 1);
        lcd.print(tempC, 1);
        lcd.print((char)223);
        lcd.print("C ");
        lcd.print(tempF, 0);
        lcd.print((char)223);
        lcd.print("F ");
        break;

      case 2:  // Show PID constants
        lcd.setCursor(0, 0);
        lcd.print("Kp:");
        lcd.print(Kp, 3);
        lcd.print(" Ki:");
        lcd.print(Ki, 3);
        lcd.setCursor(0, 1);
        lcd.print("Kd:");
        lcd.print(Kd, 3);
        lcd.print("         ");
        break;

      case 3:  // Show motor duty cycle (%)
        float dutyPercent = ((float)motorDuty / 1023) * 100;
        lcd.setCursor(0, 0);
        lcd.print("Motor Duty:");
        lcd.setCursor(0, 1);
        lcd.print((int)dutyPercent);
        lcd.print("%    ");
        break;
    }
  }

  // Serial CSV logging every logInterval ms
  if (currentMillis - lastLogTime >= logInterval) {
    lastLogTime = currentMillis;

    // Convert uptime from ms to HH:MM:SS format
    unsigned long uptimeSeconds = currentMillis / 1000;
    int hours = uptimeSeconds / 3600;
    int minutes = (uptimeSeconds % 3600) / 60;
    int seconds = uptimeSeconds % 60;
    char uptimeStr[10];
    sprintf(uptimeStr, "%02d:%02d:%02d", hours, minutes, seconds);

    // Read temperature again for logging
    int tempSensorVal = analogRead(tempSensorPin);
    float tempVolts = tempSensorVal * (5.0 / 1023.0);
    float tempCelcius = (tempVolts - 0.5) * 100;

    // Calculate motor duty cycle percentage and PID output voltage
    float motorDutyPercent = ((float)motorDuty / 1023) * 100;
    float pidVoltOut = (pidOutput / 1023) * 5;

    // Print CSV line with uptime, measurements, and outputs
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(uptimeStr);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(tempCelcius, 2);
    Serial.print(",");
    Serial.print(motorDutyPercent, 2);
    Serial.print(",");
    Serial.print(setpointVoltage, 2);
    Serial.print(",");
    Serial.println(pidVoltOut, 2);
  }
}
