// Sensor parameters
const int sensorPin = PB1;         // Analog pin PB1 for pressure sensor
const float VCC = 3.3;            // STM32 ADC reference voltage (3.3V)
const float ADC_RES = 4095.0;     // 12-bit ADC resolution (0-4095)
const float SENSOR_MIN_VOLTAGE = 0.5;  // Min output voltage (0 psi) at 5V supply
const float SENSOR_MAX_VOLTAGE = 4.5;  // Max output voltage (700 kPa) at 5V supply
const float MAX_PRESSURE_KPA = 700.0;  // Max pressure in kPa
const float KPA_TO_PSI = 0.1450377;    // Conversion factor (1 kPa = 0.145 psi)
const float VOLTAGE_DIVIDER_RATIO = 3.0; // 10k:20k divider (adjust if different)

// Servo pin definition
const int servoPin = PB9;         // PWM pin (specify your pin here, e.g., PB9)

// PWM settings
const int pwmFreq = 200;          // 200 Hz

// Duty cycle values (0-255 range)
const int closedDuty = 71;        // 20% duty cycle (0.20 * 255)
const int openDuty = 112;         // 50% duty cycle (0.50 * 255)

// State and timing
bool isOpen = false;
unsigned long startTime;          // To track elapsed time
const float pressureThreshold = 80.0; // Open servo at 80 psi

void setup() {
  // Record start time
  startTime = millis();

  // Configure pins
  pinMode(sensorPin, INPUT_ANALOG);
  pinMode(servoPin, OUTPUT);

  // Configure PWM for servo
  analogWriteFrequency(pwmFreq);  // Set PWM frequency to 200 Hz
  analogWrite(servoPin, closedDuty); // Start with servo closed
}

void loop() {
  // Read voltage from the sensor (after voltage divider)
  int sensorValue = analogRead(sensorPin);
  float voltageAtADC = (sensorValue * VCC) / ADC_RES; // Voltage at PB1 (0-3.3V)
  float sensorVoltage = voltageAtADC * VOLTAGE_DIVIDER_RATIO; // Actual sensor voltage (0.5-4.5V)
  
  // Convert voltage to pressure in kPa
  float pressure_kPa = ((sensorVoltage - SENSOR_MIN_VOLTAGE) / 
                       (SENSOR_MAX_VOLTAGE - SENSOR_MIN_VOLTAGE)) * MAX_PRESSURE_KPA;
  
  // Convert kPa to psi
  float pressure_psi = pressure_kPa * KPA_TO_PSI;
  float fixed_pressure_psi = 1.6 * pressure_psi;
  
  // Calculate elapsed time in seconds
  float elapsedTime = (millis() - startTime) / 1000.0;
  
  // Control servo based on pressure
  controlServo(fixed_pressure_psi);

  // Delay for stability (100ms)
  delay(100);
}

// Function to control servo based on pressure
void controlServo(float pressure) {
  if (pressure >= pressureThreshold && !isOpen) {
    // Open the servo
    analogWrite(servoPin, openDuty);
    isOpen = true;
  } else if (pressure < pressureThreshold && isOpen) {
    // Close the servo
    analogWrite(servoPin, closedDuty);
    isOpen = false;
  }
}