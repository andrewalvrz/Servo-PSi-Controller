// Sensor parameters
const int sensorPin = A0;         // Analog pin A0 for pressure sensor
const float VCC = 5.0;            // Nano ADC reference voltage (5V default)
const float ADC_RES = 1023.0;     // 10-bit ADC resolution (0-1023)
const float SENSOR_MIN_VOLTAGE = 0.3425;  // Min output voltage (0 psi) at 5V supply
const float SENSOR_MAX_VOLTAGE = 3.0825;  // Max output voltage (700 kPa) at 5V supply
const float MAX_PRESSURE_KPA = 700.0;     // Max pressure in kPa
const float KPA_TO_PSI = 0.1450377;       // Conversion factor (1 kPa = 0.145 psi)
const float VOLTAGE_DIVIDER_RATIO = 2.174; // 1.37k/6.30k, R2=1.37k divider

// Servo pin definition
const int servoPin = 9;           // PWM pin D9

// Duty cycle values (0-255 range)
const int closedDuty = 51;        // ~20% duty cycle (0.20 * 255)
const int openDuty = 127;         // ~50% duty cycle (0.50 * 255)

// State and timing
bool isOpen = false;
unsigned long startTime;          // To track elapsed time
const float pressureThreshold = 80.0; // Open servo at 80 psi

void setup() {
  // Record start time
  startTime = millis();

  // Configure pins
  pinMode(sensorPin, INPUT);      // Analog input for sensor
  pinMode(servoPin, OUTPUT);      // PWM output for servo

  // Start with servo closed
  analogWrite(servoPin, closedDuty);
}

void loop() {
  // Read voltage from the sensor (after voltage divider)
  int sensorValue = analogRead(sensorPin);
  float voltageAtADC = (sensorValue * VCC) / ADC_RES; // Voltage at A0 (0-5V)
  float sensorVoltage = voltageAtADC * VOLTAGE_DIVIDER_RATIO; // Actual sensor voltage (0.3425-3.0825V)
  
  // Convert voltage to pressure in kPa
  float pressure_kPa = ((sensorVoltage - SENSOR_MIN_VOLTAGE) / 
                       (SENSOR_MAX_VOLTAGE - SENSOR_MIN_VOLTAGE)) * MAX_PRESSURE_KPA;
  
  // Convert kPa to psi
  float pressure_psi = pressure_kPa * KPA_TO_PSI;
  float fixed_pressure_psi = 1.6 * pressure_psi; // Apply correction factor
  
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