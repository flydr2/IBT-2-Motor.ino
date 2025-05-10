// This is a very crude but working script
// The other file is more complete.
// For use on arduino Mega serial port 1

#include <Arduino.h>
#include "crc.h" // Ensure crc.h is available for CRC calculations

// Pin definitions
#define RPWM 9
#define LPWM 10
#define REN 4 // for now these are wired to 5v
#define LEN 3
#define L_IS A1 // not up to the right amps
#define R_IS A1

// Command and telemetry codes
#define DISENGAGE_CODE 0x68
#define ENGAGE_CODE 0x36
#define COMMAND_CODE 0xC7
#define MAX_CURRENT_CODE 0x1E
#define CURRENT_CODE 0x1C
#define VOLTAGE_CODE 0xB3
#define FLAGS_CODE 0x8F
#define MAX_MOTOR_TEMP_CODE 0x5A
#define RUDDER_MIN_CODE 0x2B
#define RUDDER_MAX_CODE 0x4D
#define MAX_SLEW_CODE 0x71
#define EEPROM_READ_CODE 0x91
#define MAX_CONTROLLER_TEMP_CODE 0xA4
#define RUDDER_RANGE_CODE 0xB6
#define EEPROM_WRITE_CODE 0x53
#define CLUTCH_PWM_AND_BRAKE_CODE 0x36
#define RESET_CODE 0xE7
#define REPROGRAM_CODE 0x19

// Handshake and telemetry codes for Pypilot detection
#define HANDSHAKE_CODE 0x5A

// Global variables
float currentAmps = 0.0;
float voltage = 12.0;
int currentLimit = 166;
int maxMotorTemp = 100; // Example value for max motor temp
int rudderMin = 0;      // Example rudder min
int rudderMax = 255;    // Example rudder max
int maxSlewRate = 10;   // Example max slew rate
bool motorEngaged = false;
bool debugMode = true;  // Toggle debug mode

// Custom floatMap function
float floatMap(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) return outMin; // Avoid division by zero
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void debugLog(const char* message) {
  if (debugMode) {
    Serial.println(message);
  }
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  motorEngaged = false;
  debugLog("Motor stopped");
}

void setMotorSpeed(int speed) {
  if (speed == 1000) {
    stopMotor();
    return;
  }

  motorEngaged = true;
  if (speed < 1000) { // Reverse
    int reverseSpeed = floatMap(speed, 999, 0, 0, 255);
    analogWrite(LPWM, reverseSpeed);
    analogWrite(RPWM, 0);
    if (debugMode) {
      Serial.print("..................Motor engaged, reverse speed: ");
      Serial.println(reverseSpeed);
    }
  } else if (speed > 1000) { // Forward
    int forwardSpeed = floatMap(speed, 1001, 2000, 0, 255);
    analogWrite(RPWM, forwardSpeed);
    analogWrite(LPWM, 0);
    if (debugMode) {
      Serial.print("..................Motor engaged, forward speed: ");
      Serial.println(forwardSpeed);
    }
  }
}

void sendFeedback() {
  static uint8_t feedbackType = 0; // Rotate between feedback types
  uint8_t feedback[4];

  uint16_t currentAmpsInt = static_cast<uint16_t>(currentAmps * 100);
  uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);

  switch (feedbackType) {
    case 0: // Current feedback
      feedback[0] = CURRENT_CODE;
      feedback[1] = currentAmpsInt & 0xFF;
      feedback[2] = (currentAmpsInt >> 8) & 0xFF;
      break;

    case 1: // Voltage feedback
      feedback[0] = VOLTAGE_CODE;
      feedback[1] = voltageInt & 0xFF;
      feedback[2] = (voltageInt >> 8) & 0xFF;
      break;

    case 2: // Flags feedback
      feedback[0] = FLAGS_CODE;
      feedback[1] = motorEngaged ? 1 : 0;
      feedback[2] = 0;
      break;

    default:
      feedbackType = 0;
      return;
  }

  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4); // Send feedback packet

  if (debugMode) {
    Serial.print("Sent feedback: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(feedback[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  feedbackType = (feedbackType + 1) % 3; // Rotate feedback type
}

void sendHandshake() {
  uint8_t handshake[4];
  handshake[0] = HANDSHAKE_CODE;
  handshake[1] = 0x00; // Reserved byte
  handshake[2] = 0x00; // Reserved byte
  handshake[3] = crc8(handshake, 3); // Calculate CRC

  Serial1.write(handshake, 4); // Send handshake packet

  if (debugMode) {
    Serial.print("Sent handshake: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(handshake[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void parseCommand(uint8_t *command) {
  uint8_t cmd = command[0];
  uint16_t value = command[1] | (command[2] << 8);

  if (debugMode) {
    Serial.print("Received command: ");
    Serial.print(cmd, HEX);
    Serial.print(", value: ");
    Serial.println(value);
  }

  switch (cmd) {
    case DISENGAGE_CODE:
      stopMotor();
      break;

    case ENGAGE_CODE:
    case COMMAND_CODE: // Includes speed and direction control
      setMotorSpeed(value);
      break;

    case MAX_CONTROLLER_TEMP_CODE:
      if (debugMode) {
        Serial.print("Max controller temperature set to: ");
        Serial.println(value);
      }
      break;

    case MAX_CURRENT_CODE:
      currentLimit = value;
      if (debugMode) {
        Serial.print("Max current set to: ");
        Serial.println(currentLimit);
      }
      break;

    case MAX_MOTOR_TEMP_CODE:
      maxMotorTemp = value;
      if (debugMode) {
        Serial.print("Max motor temperature set to: ");
        Serial.println(maxMotorTemp);
      }
      break;

    case RUDDER_MIN_CODE:
      rudderMin = value;
      if (debugMode) {
        Serial.print("Rudder minimum set to: ");
        Serial.println(rudderMin);
      }
      break;

    case RUDDER_MAX_CODE:
      rudderMax = value;
      if (debugMode) {
        Serial.print("Rudder maximum set to: ");
        Serial.println(rudderMax);
      }
      break;

    case MAX_SLEW_CODE:
      maxSlewRate = value;
      if (debugMode) {
        Serial.print("Max slew rate set to: ");
        Serial.println(maxSlewRate);
      }
      break;

    case EEPROM_READ_CODE:
      if (debugMode) {
        Serial.print("EEPROM read request received for address: ");
        Serial.println(value);
      }
      // Add EEPROM read functionality here
      break;

    default:
      if (debugMode) {
        Serial.print("........................Unknown command received: ");
        Serial.println(cmd, HEX);
      }
      break;
  }
}

bool verifyCRC(uint8_t *data) {
  uint8_t crc = crc8(data, 3);
  return crc == data[3];
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(L_IS, INPUT);
  pinMode(R_IS, INPUT);

  stopMotor();
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  Serial.println("Arduino motor controller initialized");

  // Send initial handshake for Pypilot detection
  sendHandshake();
}

void loop() {
  static uint8_t buffer[4];
  static uint8_t bufferIndex = 0;

  while (Serial1.available() > 0) {
    uint8_t receivedByte = Serial1.read();
    buffer[bufferIndex++] = receivedByte;

    if (bufferIndex == 4) {
      if (verifyCRC(buffer)) {
        parseCommand(buffer);
      } else if (debugMode) {
        Serial.println("Invalid CRC received");
      }
      bufferIndex = 0;
    }
  }

  int currentSenseRaw = analogRead(L_IS);
  currentAmps = floatMap(currentSenseRaw, 0, 1023, 0, 10);

  static unsigned long lastFeedbackTime = 0;
  static unsigned long lastHandshakeTime = 0;
  unsigned long currentMillis = millis();

  // Send telemetry feedback every 100ms
  if (currentMillis - lastFeedbackTime > 100) {
    sendFeedback();
    lastFeedbackTime = currentMillis;
  }

  // Send handshake every 1000ms to ensure detection
  if (currentMillis - lastHandshakeTime > 1000) {
    sendHandshake();
    lastHandshakeTime = currentMillis;
  }
}
