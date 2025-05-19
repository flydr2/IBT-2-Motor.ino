/*
 * Ported to use ITB-2 and ESP32 in language I can understand. 
 * contact: Marc flydr2@gmail.com
 * There is room for improvement. 
 * 
 * !!!!!! Use at your own risk !!!!!
 * /*
 * Ported to use ITB-2 and ESP32 in language I can understand. 
 * contact: Marc flydr2@gmail.com
 * There is room for improvement. 
 * 
 * !!!!!! Use at your own risk !!!!!
 * 
 * There are 3 versions (that I know about) of the IBT-2.
   The difference is the IS pins output voltage per mw... Be sure never to send over 3.3v to the IS_PINS 34 

 * The adjustments in pypilot can be made to use the max amp as a limit stop... 
 * I intend to make an other version which will have a backup pilot built-in and managed from wifi AP mode in the esp32
 * 
 * Ported partially (To my needs not everything is ported. (Enough for my needs)
 * Limit current, limit switches, Slew, and some faults work... 
 * If anyone improves it... Please send me a link to your version.
 * added a restart when CRC fails
 */

#include <Arduino.h>
#include <HardwareSerial.h> // ESP32-specific serial library
#include "crc.h" // Original crc.h for CRC calculations



// Pin definitions for ESP32
#define RPWM 25  // PWM pin for right (starboard) motor direction
#define LPWM 26  // PWM pin for left (port) motor direction
#define REN  27  // Enable pin for motor driver (REN and LEN tied together)
#define IS_PINS 34 // Analog input for current sensing (ADC1_6) Wire the 2 IS pins together to a 680Ω resistor to ground for a 0–3.3V (1k for 5v) acts as a voltage divider
#define Port_Switch_pin 32 // Rudder limit switch (port)
#define Starboard_Switch_pin 33 // Rudder limit switch (starboard)

//I will have extra defines for the selfcontained autopilot here later.

// PWM settings for ESP32
#define PWM_FREQ 10000   // PWM frequency (Hz)
#define PWM_RESOLUTION 8 // 8-bit resolution (0–255)

// Serial port for pypilot communication
HardwareSerial SerialPypilot(2); // Use UART2
#define RX_PIN 16 // RX pin for UART2
#define TX_PIN 17 // TX pin for UART2

// Command and telemetry codes
#define DISENGAGE_CODE 0x68
#define ENGAGE_CODE 0x37
#define COMMAND_CODE 0xC7
#define CONTROLLER_TEMP_CODE 0xf9
#define MAX_CURRENT_CODE 0x1E
#define CURRENT_CODE 0x1C
#define VOLTAGE_CODE 0xB3
#define FLAGS_CODE 0x8F
#define MAX_MOTOR_TEMP_CODE 0x5A
#define RUDDER_MIN_CODE 0x2B
#define RUDDER_MAX_CODE 0x4D
#define RUDDER_SENSE_CODE 0xa7
#define MOTOR_TEMP_CODE 0x48
#define MAX_SLEW_CODE 0x71
#define EEPROM_READ_CODE 0x91
#define MAX_CONTROLLER_TEMP_CODE 0xA4
#define RUDDER_RANGE_CODE 0xB6
#define EEPROM_WRITE_CODE 0x53
#define CLUTCH_PWM_AND_BRAKE_CODE 0x36
#define RESET_CODE 0xE7
#define REPROGRAM_CODE 0x19
#define EEPROM_VALUE_CODE 0x9a
#define OVERCURRENT_FAULT 4
#define SYNC 1
#define ENGAGED 8
#define INVALID 16
#define PORT_PIN_FAULT 32
#define STARBOARD_PIN_FAULT 64
#define BADVOLTAGE 128
#define MIN_RUDDER_FAULT 256
#define MAX_RUDDER_FAULT 512
#define CURRENT_RANGE 1024
#define BAD_FUSES 2048
#define REBOOTED 32768
#define HANDSHAKE_CODE 0x5A


// Global variables
int speed = 1000;
float currentAmps = 0.0;
float voltage = 12.0;
int currentLimit = 4096; // Adjusted for 12-bit ADC (calibrate)
int maxMotorTemp = 100;
int maxSlewRate = 20;
int max_slew_slow = 20;
bool motorEngaged = false;
bool debugMode = false; // Enable for debugging

uint16_t flags = SYNC;
bool isSynced = true;
int port_starboard_direction;
int starboard_overcurrent = 0;
int port_overcurrent = 0;
int PortValue = 0;
int StarboardValue = 0;
static int16_t rudderSense = 0;
float controllerTemp = 25.0;
float motorTemp = 30.0;
#define ROLLING_AVG_SIZE 100 //may not need this 
float currentAmpsHistory[ROLLING_AVG_SIZE] = {0.0};
int currentAmpsIndex = 0;
float rollingAverageCurrent = 0.0;
int currentSenseRaw = 0;


// Custom floatMap function (adjusted for 12-bit ADC)
float floatMap(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) return outMin;
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void debugLog(const char* message) {
  if (debugMode) {
    Serial.println(message);
  }
}

void stopMotor() {
  ledcWrite(RPWM, 0); // Use pin directly in v3.0
  ledcWrite(LPWM, 0);
  speed = 1000;
  motorEngaged = false;
  debugLog("Motor stopped");
}

void stop_port() {
  stopMotor();
  port_overcurrent = 1;
  Serial.println("Port overCurrent");
}

void stop_starboard() {
  stopMotor();
  starboard_overcurrent = 1;
  Serial.println("Starboard overCurrent");
}

void resetFaults() {
  port_overcurrent = 0;
  starboard_overcurrent = 0;
  flags &= ~(OVERCURRENT_FAULT | PORT_PIN_FAULT | STARBOARD_PIN_FAULT | MIN_RUDDER_FAULT | MAX_RUDDER_FAULT);

  
  if (debugMode) {
    Serial.println("All faults have been reset.");
  }
}



int calculateSlewRate(int targetSpeed, int lastSpeed, int maxSlewSpeed, int maxSlewSlow) {
  int speedDifference = targetSpeed - lastSpeed;
  if (speedDifference > maxSlewSpeed) {
    return lastSpeed + maxSlewSpeed;
  } else if (speedDifference < -maxSlewSlow) {
    return lastSpeed - maxSlewSlow;
  }
  return targetSpeed;
}

void setMotorSpeed(int speed) {
  static int lastSpeed = 0;
   
      if ((speed == 1000) || (speed == 254)) {
    stopMotor();
    lastSpeed = 0;
    return;
  }

  int targetSpeed = (speed < 1000)
                    ? map(speed, 999, 0, 0, 255)
                    : map(speed, 1001, 2000, 0, 255);

  if (targetSpeed != lastSpeed) {
    targetSpeed = calculateSlewRate(targetSpeed, lastSpeed, maxSlewRate, max_slew_slow);
    lastSpeed = targetSpeed;
  }

  motorEngaged = true;

  if (speed < 1000) { // STARBOARD
    port_starboard_direction = 2;
    port_overcurrent = 0;
    if (starboard_overcurrent == 0 && StarboardValue != LOW) {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, targetSpeed);
   //   if (debugMode) {
        Serial.print("Motor engaged, STARBOARD speed (0-255): ");
        Serial.println(targetSpeed);
        Serial.print("Max current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
   //   }
    } else {
      stop_starboard();
    }
  } else if (speed > 1000) { // PORT
    port_starboard_direction = 1;
    starboard_overcurrent = 0;
    if (port_overcurrent == 0 && PortValue != LOW) {
      ledcWrite(LPWM, 0);
      ledcWrite(RPWM, targetSpeed);
    ///  if (debugMode) {
        Serial.print("Motor engaged, PORT speed (0-255): ");
        Serial.println(targetSpeed);
        Serial.print("Max current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
    //  }
    } else {
      stop_port();
    }
  }
}

void sendFeedback() {
  if (SerialPypilot.availableForWrite() < 32) return;
 

  uint8_t feedback[4];

  // Current Feedback (units: 10mA)
  uint16_t currentAmpsInt = static_cast<uint16_t>(rollingAverageCurrent);
  feedback[0] = CURRENT_CODE;
  feedback[1] = currentAmpsInt & 0xFF;
  feedback[2] = (currentAmpsInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Voltage Feedback (units: 10mV)
  uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);
  feedback[0] = VOLTAGE_CODE;
  feedback[1] = voltageInt & 0xFF;
  feedback[2] = (voltageInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Controller Temperature Feedback
  uint16_t controllerTempInt = static_cast<uint16_t>(controllerTemp);
  feedback[0] = CONTROLLER_TEMP_CODE;
  feedback[1] = controllerTempInt & 0xFF;
  feedback[2] = (controllerTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Motor Temperature Feedback
  uint16_t motorTempInt = static_cast<uint16_t>(motorTemp);
  feedback[0] = MOTOR_TEMP_CODE;
  feedback[1] = motorTempInt & 0xFF;
  feedback[2] = (motorTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Rudder Feedback
  feedback[0] = RUDDER_SENSE_CODE;
  feedback[1] = rudderSense & 0xFF;
  feedback[2] = (rudderSense >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Flags Feedback
  uint16_t faultFlags = flags;
  if (rollingAverageCurrent > currentLimit) {
    faultFlags |= OVERCURRENT_FAULT;
  }
  if (motorEngaged) {
    faultFlags |= ENGAGED;
  }
//  if (port_overcurrent || PortValue == LOW) {
//    faultFlags |= MAX_RUDDER_FAULT;
//  }
//  if (starboard_overcurrent || StarboardValue == LOW) {
//    faultFlags |= MIN_RUDDER_FAULT;
//  }
  feedback[0] = FLAGS_CODE;
  feedback[1] = faultFlags & 0xFF;
  feedback[2] = (faultFlags >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  if (debugMode && (faultFlags & OVERCURRENT_FAULT)) {
    Serial.println("Overcurrent fault detected!");
  }
}

void sendHandshake() {
  uint8_t handshake[4];
  handshake[0] = HANDSHAKE_CODE;
  handshake[1] = 0x00;
  handshake[2] = 0x00;
  handshake[3] = crc8(handshake, 3);
  SerialPypilot.write(handshake, 4);
}

void parseCommand(uint8_t *command) {
  static unsigned long lastCommandTime = 0;
  uint8_t cmd = command[0];
  uint16_t value = command[1] | (command[2] << 8);

  isSynced = true;
  lastCommandTime = millis();

  if (debugMode) {
    Serial.print("Received packet: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      Serial.print(command[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("Command: 0x");
    Serial.print(cmd, HEX);
    Serial.print(", value: ");
    Serial.println(value);
  }

  uint8_t feedback[4];
  switch (cmd) {
    case HANDSHAKE_CODE:
      feedback[0] = HANDSHAKE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("Sent handshake response");
      break;
    case DISENGAGE_CODE:
      stopMotor();
      break;
    case RESET_CODE:
      resetFaults();
      break;
    case ENGAGE_CODE:
      motorEngaged = true;
      break;
    case COMMAND_CODE:
      setMotorSpeed(value);
      break;
    case MAX_CURRENT_CODE:
      currentLimit = value;
      if (debugMode) {
        Serial.print("Max current set to: ");
        Serial.println(currentLimit * 0.01, 2);
        Serial.print("Current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
      }
      break;
    case MAX_SLEW_CODE:
      maxSlewRate = command[1];
      max_slew_slow = command[2];
      if (maxSlewRate > 250) maxSlewRate = 250;
      if (max_slew_slow > 250) max_slew_slow = 250;
      if (maxSlewRate < 1) maxSlewRate = 1;
      if (max_slew_slow < 1) max_slew_slow = 1;
      if (debugMode) {
        Serial.print("Max slew rate: ");
        Serial.println(maxSlewRate);
        Serial.print("Max slew slow: ");
        Serial.println(max_slew_slow);
      }
      break;
    case EEPROM_READ_CODE:
      {
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = currentLimit & 0xFF;
        feedback[2] = (currentLimit >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 0");

        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = 0x01;
        feedback[2] = 0x00;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 2");

        uint16_t range = 4000; // Rudder range
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = range & 0xFF;
        feedback[2] = (range >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 4");
      }
      break;
    case EEPROM_WRITE_CODE:
      feedback[0] = EEPROM_VALUE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("EEPROM write acknowledged");
      break;
    case RUDDER_RANGE_CODE:
      {
        uint16_t range = 4000;
        feedback[0] = RUDDER_RANGE_CODE;
        feedback[1] = range & 0xFF;
        feedback[2] = (range >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        if (debugMode) Serial.println("Sent rudder range");
      }
      break;
    case REPROGRAM_CODE:
      feedback[0] = REPROGRAM_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("Reprogram command acknowledged");
      break;
    default:
      if (debugMode) {
        Serial.print("Unknown command: 0x");
        Serial.println(cmd, HEX);
      }
 
      break;
  }

  if (millis() - lastCommandTime > 100) {
    isSynced = false;
    flags &= ~SYNC;
  }
}

bool verifyCRC(uint8_t *data) {
  return crc8(data, 3) == data[3];
}

void setup() {
  Serial.begin(115200);
  SerialPypilot.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Configure PWM using ESP32 core v3.0 LEDC API
  ledcAttach(RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Pin modes
  pinMode(REN, OUTPUT);
  pinMode(Port_Switch_pin, INPUT_PULLUP);
  pinMode(Starboard_Switch_pin, INPUT_PULLUP);
  pinMode(IS_PINS, INPUT);

  // Initialize motor
  digitalWrite(REN, HIGH);
  stopMotor();

 // Serial.println("ESP32 motor controller initialized");
  sendHandshake();

}

void loop() {
  static uint8_t buffer[4];
  static uint8_t bufferIndex = 0;
  static uint8_t invalidCrcCount = 0;
  const uint8_t MAX_INVALID_CRC = 5;

  // Process serial data
  if (SerialPypilot.available() > 0) {
    if (bufferIndex >= 4) {
      if (debugMode) Serial.println("Buffer overflow, resetting");
      bufferIndex = 0;
      invalidCrcCount++;



      
    } else {
      buffer[bufferIndex++] = SerialPypilot.read();
      if (bufferIndex == 4) {
        if (verifyCRC(buffer)) {
          parseCommand(buffer);
          invalidCrcCount = 0;
        } else {
          invalidCrcCount++;
          if (debugMode) {
            Serial.print("Invalid CRC received, count: ");
            Serial.println(invalidCrcCount);
          }
          
        }
        bufferIndex = 0;
      }
    }
    if (invalidCrcCount >= MAX_INVALID_CRC) {
      while (SerialPypilot.available() > 0) SerialPypilot.read();
      invalidCrcCount = 0;
      
      if (debugMode) Serial.println("Flushed SerialPypilot due to excessive invalid CRCs");
      sendHandshake();
      ESP.restart();
    }
  }

  // Read current sense (12-bit ADC)
   currentSenseRaw = analogRead(IS_PINS);
  currentAmps = floatMap(currentSenseRaw, 0, 4095, 0, 8500); // Map to 0–43A (10mA units) Still need to check this it's wrong

  // Update rolling average
  currentAmpsHistory[currentAmpsIndex] = currentAmps;
  currentAmpsIndex = (currentAmpsIndex + 1) % ROLLING_AVG_SIZE;
  rollingAverageCurrent = 0.0;
  for (int i = 0; i < ROLLING_AVG_SIZE; i++) {
    rollingAverageCurrent += currentAmpsHistory[i];
  }
  rollingAverageCurrent /= ROLLING_AVG_SIZE;

  // Overcurrent and limit switch checks
  if (rollingAverageCurrent > currentLimit) {
    if (port_starboard_direction == 1) {
      stop_port();
    } else if (port_starboard_direction == 2) {
      stop_starboard();
    }
    if (debugMode) {
      Serial.println("Motor stopped due to overcurrent!");
      Serial.print("Max current set to: ");
      Serial.println(currentLimit * 0.01, 2);
      Serial.print("Current: ");
      Serial.println(rollingAverageCurrent * 0.01, 2);
      
    }
  }

  // Rudder limit switch checks
  PortValue = digitalRead(Port_Switch_pin);
  if (port_starboard_direction == 1 && PortValue == LOW) {
    stop_port();
  }

  StarboardValue = digitalRead(Starboard_Switch_pin);
  if (port_starboard_direction == 2 && StarboardValue == LOW) {
    stop_starboard();
  }

  // Fault pin checks
  if (!digitalRead(Port_Switch_pin)) {
    stop_port();
    flags |= PORT_PIN_FAULT;
  } else {
    flags &= ~PORT_PIN_FAULT;
  }

  if (!digitalRead(Starboard_Switch_pin)) {
    stop_starboard();
    flags |= STARBOARD_PIN_FAULT;
  } else {
    flags &= ~STARBOARD_PIN_FAULT;
  }

  // Periodic tasks
  static unsigned long lastFeedbackTime = 0;
  static unsigned long lastHandshakeTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastFeedbackTime > 100) {
    sendFeedback();
    lastFeedbackTime = currentMillis;
  }

  if (currentMillis - lastHandshakeTime > 1000) {

      
    sendHandshake();
    lastHandshakeTime = currentMillis;
  }
}
 * There are 3 versions (that I know about) of the IBT-2.
   The difference is the IS pins output... Be sure never to send over 3.3v to the IS_PINS 34 

 * The adjustments in pypilot can be made to use the max amp as a limit stop... 
 * I intend to make an other version which will have a backup pilot built-in and managed from wifi AP mode in the esp32
 * 
 * Ported partially (To my needs not everything is ported. (Enough for my needs)
 * Limit current, limit switches, Slew, and some faults work... 
 * If anyone improves it... Please send me a link to your version.
 */

#include <Arduino.h>
#include <HardwareSerial.h> // ESP32-specific serial library
#include "crc.h" // Original crc.h for CRC calculations



// Pin definitions for ESP32
#define RPWM 25  // PWM pin for right (starboard) motor direction
#define LPWM 26  // PWM pin for left (port) motor direction
#define REN  27  // Enable pin for motor driver (REN and LEN tied together)
#define IS_PINS 34 // Analog input for current sensing (ADC1_6) Wire the 2 IS pins together to a 680Ω resistor to ground for a 0–3.3V (1k for 5v) acts as a voltage divider
#define Port_Switch_pin 32 // Rudder limit switch (port)
#define Starboard_Switch_pin 33 // Rudder limit switch (starboard)

//I will have extra defines for the selfcontained autopilot here later.

// PWM settings for ESP32
#define PWM_FREQ 10000   // PWM frequency (Hz)
#define PWM_RESOLUTION 8 // 8-bit resolution (0–255)

// Serial port for pypilot communication
HardwareSerial SerialPypilot(2); // Use UART2
#define RX_PIN 16 // RX pin for UART2
#define TX_PIN 17 // TX pin for UART2

// Command and telemetry codes
#define DISENGAGE_CODE 0x68
#define ENGAGE_CODE 0x37
#define COMMAND_CODE 0xC7
#define CONTROLLER_TEMP_CODE 0xf9
#define MAX_CURRENT_CODE 0x1E
#define CURRENT_CODE 0x1C
#define VOLTAGE_CODE 0xB3
#define FLAGS_CODE 0x8F
#define MAX_MOTOR_TEMP_CODE 0x5A
#define RUDDER_MIN_CODE 0x2B
#define RUDDER_MAX_CODE 0x4D
#define RUDDER_SENSE_CODE 0xa7
#define MOTOR_TEMP_CODE 0x48
#define MAX_SLEW_CODE 0x71
#define EEPROM_READ_CODE 0x91
#define MAX_CONTROLLER_TEMP_CODE 0xA4
#define RUDDER_RANGE_CODE 0xB6
#define EEPROM_WRITE_CODE 0x53
#define CLUTCH_PWM_AND_BRAKE_CODE 0x36
#define RESET_CODE 0xE7
#define REPROGRAM_CODE 0x19
#define EEPROM_VALUE_CODE 0x9a
#define OVERCURRENT_FAULT 4
#define SYNC 1
#define ENGAGED 8
#define INVALID 16
#define PORT_PIN_FAULT 32
#define STARBOARD_PIN_FAULT 64
#define BADVOLTAGE 128
#define MIN_RUDDER_FAULT 256
#define MAX_RUDDER_FAULT 512
#define CURRENT_RANGE 1024
#define BAD_FUSES 2048
#define REBOOTED 32768
#define HANDSHAKE_CODE 0x5A

// Global variables
int speed = 1000;
float currentAmps = 0.0;
float voltage = 12.0;
int currentLimit = 4096; // Adjusted for 12-bit ADC (calibrate)
int maxMotorTemp = 100;
int maxSlewRate = 20;
int max_slew_slow = 20;
bool motorEngaged = false;
bool debugMode = false; // Enable for debugging

uint16_t flags = SYNC;
bool isSynced = true;
int port_starboard_direction;
int starboard_overcurrent = 0;
int port_overcurrent = 0;
int PortValue = 0;
int StarboardValue = 0;
static int16_t rudderSense = 0;
float controllerTemp = 25.0;
float motorTemp = 30.0;
#define ROLLING_AVG_SIZE 10
float currentAmpsHistory[ROLLING_AVG_SIZE] = {0.0};
int currentAmpsIndex = 0;
float rollingAverageCurrent = 0.0;
int currentSenseRaw = 0;


// Custom floatMap function (adjusted for 12-bit ADC)
float floatMap(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) return outMin;
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void debugLog(const char* message) {
  if (debugMode) {
    Serial.println(message);
  }
}

void stopMotor() {
  ledcWrite(RPWM, 0); // Use pin directly in v3.0
  ledcWrite(LPWM, 0);
  speed = 1000;
  motorEngaged = false;
  debugLog("Motor stopped");
}

void stop_port() {
  stopMotor();
  port_overcurrent = 1;
//  rudderSense = 3000; // Indicate port limit
//  flags |= MAX_RUDDER_FAULT;
 // flags &= ~(MIN_RUDDER_FAULT);
  Serial.println("Port overCurrent");
}

void stop_starboard() {
  stopMotor();
  starboard_overcurrent = 1;
//  rudderSense = -3000; // Indicate starboard limit
 // flags |= MIN_RUDDER_FAULT;
 // flags &= ~(MAX_RUDDER_FAULT);
  Serial.println("Starboard overCurrent");
}

void resetFaults() {
  port_overcurrent = 0;
  starboard_overcurrent = 0;
  flags &= ~(OVERCURRENT_FAULT | PORT_PIN_FAULT | STARBOARD_PIN_FAULT | MIN_RUDDER_FAULT | MAX_RUDDER_FAULT);
  if (debugMode) {
    Serial.println("All faults have been reset.");
  }
}



int calculateSlewRate(int targetSpeed, int lastSpeed, int maxSlewSpeed, int maxSlewSlow) {
  int speedDifference = targetSpeed - lastSpeed;
  if (speedDifference > maxSlewSpeed) {
    return lastSpeed + maxSlewSpeed;
  } else if (speedDifference < -maxSlewSlow) {
    return lastSpeed - maxSlewSlow;
  }
  return targetSpeed;
}

void setMotorSpeed(int speed) {
  static int lastSpeed = 0;


  if ((speed == 1000) || (speed == 254)) {
    stopMotor();
    lastSpeed = 0;
    return;
  }

  int targetSpeed = (speed < 1000)
                    ? map(speed, 999, 0, 0, 255)
                    : map(speed, 1001, 2000, 0, 255);

  if (targetSpeed != lastSpeed) {
    targetSpeed = calculateSlewRate(targetSpeed, lastSpeed, maxSlewRate, max_slew_slow);
    lastSpeed = targetSpeed;
  }

  motorEngaged = true;

  if (speed < 1000) { // STARBOARD
    port_starboard_direction = 2;
    port_overcurrent = 0;
    if (starboard_overcurrent == 0 && StarboardValue != LOW) {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, targetSpeed);
     // if (debugMode) {
        Serial.print("Motor engaged, STARBOARD speed (0-255): ");
        Serial.println(targetSpeed);
        Serial.print("Max current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
     // }
    } else {
      stop_starboard();
    }
  } else if (speed > 1000) { // PORT
    port_starboard_direction = 1;
    starboard_overcurrent = 0;
    if (port_overcurrent == 0 && PortValue != LOW) {
      ledcWrite(LPWM, 0);
      ledcWrite(RPWM, targetSpeed);
   //   if (debugMode) {
        Serial.print("Motor engaged, PORT speed (0-255): ");
        Serial.println(targetSpeed);
        Serial.print("Max current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
     // }
    } else {
      stop_port();
    }
  }
}

void sendFeedback() {
  if (SerialPypilot.availableForWrite() < 32) return;

  uint8_t feedback[4];

  // Current Feedback (units: 10mA)
  uint16_t currentAmpsInt = static_cast<uint16_t>(rollingAverageCurrent);
  feedback[0] = CURRENT_CODE;
  feedback[1] = currentAmpsInt & 0xFF;
  feedback[2] = (currentAmpsInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Voltage Feedback (units: 10mV)
  uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);
  feedback[0] = VOLTAGE_CODE;
  feedback[1] = voltageInt & 0xFF;
  feedback[2] = (voltageInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Controller Temperature Feedback
  uint16_t controllerTempInt = static_cast<uint16_t>(controllerTemp);
  feedback[0] = CONTROLLER_TEMP_CODE;
  feedback[1] = controllerTempInt & 0xFF;
  feedback[2] = (controllerTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Motor Temperature Feedback
  uint16_t motorTempInt = static_cast<uint16_t>(motorTemp);
  feedback[0] = MOTOR_TEMP_CODE;
  feedback[1] = motorTempInt & 0xFF;
  feedback[2] = (motorTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Rudder Feedback
  feedback[0] = RUDDER_SENSE_CODE;
  feedback[1] = rudderSense & 0xFF;
  feedback[2] = (rudderSense >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  // Flags Feedback
  uint16_t faultFlags = flags;
  if (rollingAverageCurrent > currentLimit) {
    faultFlags |= OVERCURRENT_FAULT;
  }
  if (motorEngaged) {
    faultFlags |= ENGAGED;
  }
//  if (port_overcurrent || PortValue == LOW) {
//    faultFlags |= MAX_RUDDER_FAULT;
//  }
//  if (starboard_overcurrent || StarboardValue == LOW) {
//    faultFlags |= MIN_RUDDER_FAULT;
//  }
  feedback[0] = FLAGS_CODE;
  feedback[1] = faultFlags & 0xFF;
  feedback[2] = (faultFlags >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  SerialPypilot.write(feedback, 4);

  if (debugMode && (faultFlags & OVERCURRENT_FAULT)) {
    Serial.println("Overcurrent fault detected!");
  }
}

void sendHandshake() {
  uint8_t handshake[4];
  handshake[0] = HANDSHAKE_CODE;
  handshake[1] = 0x00;
  handshake[2] = 0x00;
  handshake[3] = crc8(handshake, 3);
  SerialPypilot.write(handshake, 4);
}

void parseCommand(uint8_t *command) {
  static unsigned long lastCommandTime = 0;
  uint8_t cmd = command[0];
  uint16_t value = command[1] | (command[2] << 8);

  isSynced = true;
  lastCommandTime = millis();

  if (debugMode) {
    Serial.print("Received packet: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      Serial.print(command[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("Command: 0x");
    Serial.print(cmd, HEX);
    Serial.print(", value: ");
    Serial.println(value);
  }

  uint8_t feedback[4];
  switch (cmd) {
    case HANDSHAKE_CODE:
      feedback[0] = HANDSHAKE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("Sent handshake response");
      break;
    case DISENGAGE_CODE:
      stopMotor();
      break;
    case RESET_CODE:
      resetFaults();
      break;
    case ENGAGE_CODE:
      motorEngaged = true;
      break;
    case COMMAND_CODE:
      setMotorSpeed(value);
      break;
    case MAX_CURRENT_CODE:
      currentLimit = value;
      if (debugMode) {
        Serial.print("Max current set to: ");
        Serial.println(currentLimit * 0.01, 2);
        Serial.print("Current: ");
        Serial.println(rollingAverageCurrent * 0.01, 2);
      }
      break;
    case MAX_SLEW_CODE:
      maxSlewRate = command[1];
      max_slew_slow = command[2];
      if (maxSlewRate > 250) maxSlewRate = 250;
      if (max_slew_slow > 250) max_slew_slow = 250;
      if (maxSlewRate < 1) maxSlewRate = 1;
      if (max_slew_slow < 1) max_slew_slow = 1;
      if (debugMode) {
        Serial.print("Max slew rate: ");
        Serial.println(maxSlewRate);
        Serial.print("Max slew slow: ");
        Serial.println(max_slew_slow);
      }
      break;
    case EEPROM_READ_CODE:
      {
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = currentLimit & 0xFF;
        feedback[2] = (currentLimit >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 0");

        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = 0x01;
        feedback[2] = 0x00;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 2");

        uint16_t range = 4000; // Rudder range
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = range & 0xFF;
        feedback[2] = (range >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 4");
      }
      break;
    case EEPROM_WRITE_CODE:
      feedback[0] = EEPROM_VALUE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("EEPROM write acknowledged");
      break;
    case RUDDER_RANGE_CODE:
      {
        uint16_t range = 4000;
        feedback[0] = RUDDER_RANGE_CODE;
        feedback[1] = range & 0xFF;
        feedback[2] = (range >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        SerialPypilot.write(feedback, 4);
        if (debugMode) Serial.println("Sent rudder range");
      }
      break;
    case REPROGRAM_CODE:
      feedback[0] = REPROGRAM_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      SerialPypilot.write(feedback, 4);
      if (debugMode) Serial.println("Reprogram command acknowledged");
      break;
    default:
      if (debugMode) {
        Serial.print("Unknown command: 0x");
        Serial.println(cmd, HEX);
      }
      break;
  }

  if (millis() - lastCommandTime > 2000) {
    isSynced = false;
    flags &= ~SYNC;
  }
}

bool verifyCRC(uint8_t *data) {
  return crc8(data, 3) == data[3];
}

void setup() {
  Serial.begin(115200);
  SerialPypilot.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Configure PWM using ESP32 core v3.0 LEDC API
  ledcAttach(RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Pin modes
  pinMode(REN, OUTPUT);
  pinMode(Port_Switch_pin, INPUT_PULLUP);
  pinMode(Starboard_Switch_pin, INPUT_PULLUP);
  pinMode(IS_PINS, INPUT);

  // Initialize motor
  digitalWrite(REN, HIGH);
  stopMotor();

  Serial.println("ESP32 motor controller initialized");
  sendHandshake();
}

void loop() {
  static uint8_t buffer[4];
  static uint8_t bufferIndex = 0;
  static uint8_t invalidCrcCount = 0;
  const uint8_t MAX_INVALID_CRC = 5;

  // Process serial data
  if (SerialPypilot.available() > 0) {
    if (bufferIndex >= 4) {
      if (debugMode) Serial.println("Buffer overflow, resetting");
      bufferIndex = 0;
      invalidCrcCount++;
    } else {
      buffer[bufferIndex++] = SerialPypilot.read();
      if (bufferIndex == 4) {
        if (verifyCRC(buffer)) {
          parseCommand(buffer);
          invalidCrcCount = 0;
        } else {
          invalidCrcCount++;
          if (debugMode) {
            Serial.print("Invalid CRC received, count: ");
            Serial.println(invalidCrcCount);
          }
        }
        bufferIndex = 0;
      }
    }
    if (invalidCrcCount >= MAX_INVALID_CRC) {
      while (SerialPypilot.available() > 0) SerialPypilot.read();
      invalidCrcCount = 0;
      if (debugMode) Serial.println("Flushed SerialPypilot due to excessive invalid CRCs");
      sendHandshake();
    }
  }

  // Read current sense (12-bit ADC)
   currentSenseRaw = analogRead(IS_PINS);
  currentAmps = floatMap(currentSenseRaw, 0, 4095, 0, 8500); // Map to 0–43A (10mA units) Still need to check this

  // Update rolling average
  currentAmpsHistory[currentAmpsIndex] = currentAmps;
  currentAmpsIndex = (currentAmpsIndex + 1) % ROLLING_AVG_SIZE;
  rollingAverageCurrent = 0.0;
  for (int i = 0; i < ROLLING_AVG_SIZE; i++) {
    rollingAverageCurrent += currentAmpsHistory[i];
  }
  rollingAverageCurrent /= ROLLING_AVG_SIZE;

  // Overcurrent and limit switch checks
  if (rollingAverageCurrent > currentLimit) {
    if (port_starboard_direction == 1) {
      stop_port();
    } else if (port_starboard_direction == 2) {
      stop_starboard();
    }
    if (debugMode) {
      Serial.println("Motor stopped due to overcurrent!");
      Serial.print("Max current set to: ");
      Serial.println(currentLimit * 0.01, 2);
      Serial.print("Current: ");
      Serial.println(rollingAverageCurrent * 0.01, 2);
    }
  }

  // Rudder limit switch checks
  PortValue = digitalRead(Port_Switch_pin);
  if (port_starboard_direction == 1 && PortValue == LOW) {
    stop_port();
  }

  StarboardValue = digitalRead(Starboard_Switch_pin);
  if (port_starboard_direction == 2 && StarboardValue == LOW) {
    stop_starboard();
  }

  // Fault pin checks
  if (!digitalRead(Port_Switch_pin)) {
    stop_port();
    flags |= PORT_PIN_FAULT;
  } else {
    flags &= ~PORT_PIN_FAULT;
  }

  if (!digitalRead(Starboard_Switch_pin)) {
    stop_starboard();
    flags |= STARBOARD_PIN_FAULT;
  } else {
    flags &= ~STARBOARD_PIN_FAULT;
  }

  // Periodic tasks
  static unsigned long lastFeedbackTime = 0;
  static unsigned long lastHandshakeTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastFeedbackTime > 100) {
    sendFeedback();
    lastFeedbackTime = currentMillis;
  }

  if (currentMillis - lastHandshakeTime > 1000) {
    sendHandshake();
    lastHandshakeTime = currentMillis;
  }
}
