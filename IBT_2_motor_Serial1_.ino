// I'm currently working on this draft...
//adding a rudder sensor only based on overcurrent... It works and careful settings of the amps in pypilot is required.
// Make sure that the voltage at pin A1 does not go over 5v... and that 5v is 43a or rescale it in the map function
// added current faults
//'********** This uses Serial1 on arduino Mega.************
// Added rudder limit switches (should ground them if not used)

//Todo... Register correctly to pypilot and add Faults as required

#include <Arduino.h>
#include "crc.h" // Ensure crc.h is available for CRC calculations


// Pin definitions
#define RPWM 10 //change these to your directions port/starboard
#define LPWM 9
#define REN 4 // (wire the 2 pins REN/LEN together to 1 arduino pin)
#define IS_PINS A1 //  both L and R tied together lowers the IBT "IS pins" voltage output... use a resitor to gnd to lower more (need to calculate)
#define Port_Switch_pin 4
#define Starboard_Switch_pin 5 // rudder limit switches... 


// Command and telemetry codes
#define DISENGAGE_CODE 0x68
#define ENGAGE_CODE 0x37
#define COMMAND_CODE 0xC7
#define CONTROLLER_TEMP_CODE 0xf9
#define MAX_CURRENT_CODE 0x1E // current in units of 10mA
#define CURRENT_CODE 0x1C
#define VOLTAGE_CODE 0xB3 // voltage in 10mV increments
#define FLAGS_CODE 0x8F
#define MAX_MOTOR_TEMP_CODE 0x5A        //not used here
#define RUDDER_MIN_CODE 0x2B
#define RUDDER_MAX_CODE 0x4D
#define RUDDER_SENSE_CODE 0xa7
#define MOTOR_TEMP_CODE 0x48
#define MAX_SLEW_CODE 0x71
#define EEPROM_READ_CODE 0x91           //not used here
#define MAX_CONTROLLER_TEMP_CODE 0xA4   //not used here
#define RUDDER_RANGE_CODE 0xB6
#define EEPROM_WRITE_CODE 0x53          //not used here
#define CLUTCH_PWM_AND_BRAKE_CODE 0x36  //not used here
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
#define MIN_RUDDER 256
#define MAX_RUDDER 512
#define CURRENT_RANGE 1024
#define BAD_FUSES 2048
#define REBOOTED 32768

// Handshake and telemetry codes for Pypilot detection
#define HANDSHAKE_CODE 0x5A


// Global variables
int speed = 1000;
float currentAmps = 0.0;
float voltage = 12.0;
int currentLimit = 1024;// just a number at this point
int maxMotorTemp = 100; // Example value for max motor temp
int maxSlewRate = 20;   // Example max slew rate 1 to 100
int max_slew_slow = 20;  // Example max slew rate 1 to 100
bool motorEngaged = false;
bool debugMode = false;  // Toggle debug mode
uint16_t flags = SYNC; // Initialize with SYNC and REBOOTED
bool isSynced = true; // Ensure SYNC is set from start
int port_starboard_direction; //Sets the present motor direction 1=port 2=strarboard
int starboard_overcurrent = 0; // Flag
int port_overcurrent = 0;
int PortValue = 0;
int StarboardValue = 0;
static int16_t rudderSense = 0; // Simulated rudder position

// Define variables for feedback
float controllerTemp = 25.0; // Example starting temperature for controller
float motorTemp = 30.0;      // Example starting temperature for motor

#define ROLLING_AVG_SIZE 30
float currentAmpsHistory[ROLLING_AVG_SIZE] = {0.0};
int currentAmpsIndex = 0;
float rollingAverageCurrent = 0.0;


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

void stop_port() {
  stopMotor();
  port_overcurrent = 1;
  rudderSense = 3000; // Max port position
  Serial.println("Port overCurrent");
}

void stop_starboard() {
  stopMotor();
  starboard_overcurrent = 1;
  rudderSense = -3000; // Max starboard position
  Serial.println("Starboard overCurrent");
}


void resetFaults() {
  port_overcurrent = false;
//  starboard_overcurrent = false;
//  portFaultTime = 0;
//  starboardFaultTime = 0;
//  port_starboard_direction = 0;
//  stopMotor();
//  rudderSense = 0; // Reset rudder position
  flags &= ~(OVERCURRENT_FAULT | PORT_PIN_FAULT | STARBOARD_PIN_FAULT | MIN_RUDDER | MAX_RUDDER);
 // if (debugMode) 
  Serial.println("Faults reset");
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  speed = 1000;
  motorEngaged = false;
  debugLog("Motor stopped");
}

int calculateSlewRate(int targetSpeed, int lastSpeed, int maxSlewSpeed, int maxSlewSlow) {
  int speedDifference = targetSpeed - lastSpeed;

  if (speedDifference > maxSlewSpeed) {
    return lastSpeed + maxSlewSpeed; // Accelerate up to the max slew speed
  } else if (speedDifference < -maxSlewSlow) {
    return lastSpeed - maxSlewSlow; // Decelerate up to the max slew slow
  }

  return targetSpeed; // No slew limitation needed
}

void setMotorSpeed(int speed) {

  static int lastSpeed = 0; // Track the last speed sent to the motor (range 0-255)
  // Serial.println(speed);
  // If the speed is neutral (1000), stop the motor or 254 no new code
  if ((speed == 1000) || (speed == 254)) {

    stopMotor();
    lastSpeed = 0; // Reset lastSpeed to 0 when the motor stops
    return;
  }

  // Map the input speed (1001–2000 or 0–999) to the 0–255 range
  int targetSpeed = (speed < 1000)
                    ? map(speed, 999, 0, 0, 250) // Reverse range
                    : map(speed, 1001, 2000, 0, 250); // Forward range
  //Serial.print("targetSpeedFRESH :"); Serial.println(targetSpeed);

  // Only apply slew adjustment if the speed has changed
  if (targetSpeed != lastSpeed) {
    // Calculate the new speed using the slew rate
    targetSpeed = calculateSlewRate(targetSpeed, lastSpeed, maxSlewRate, max_slew_slow);
    lastSpeed = targetSpeed; // Update the last speed
  }

  motorEngaged = true;

  // Apply the adjusted speed to the motor
  if (debugMode) {
    Serial.print("Port overcurrent status = ");
    Serial.println(port_overcurrent);
    Serial.print("Starboard overcurrent status = ");
    Serial.println(starboard_overcurrent);
  }
  if (speed < 1000) { // STARBOARD TURN
    port_starboard_direction = 2;// 2 is starboard direction
    port_overcurrent = 0; // reset the overcurrent starboard fault because we go in the opposite direction
    if (starboard_overcurrent == 0) {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, targetSpeed);
      Serial.print("..Motor engaged, STARBOARD speed (0-255): ");
      Serial.println(targetSpeed);
      Serial.print("Max current: ");
      Serial.println(rollingAverageCurrent * .01, 2); // back to amps
    } else {
      stop_starboard();
    }
    if (debugMode) {
      Serial.print("..Motor engaged, STARBOARD speed (0-255): ");
      Serial.println(targetSpeed);
      Serial.print("Max current: ");
      Serial.println(rollingAverageCurrent * .01, 2); // back to amps
    }

  } else if (speed > 1000) { // TURNING TO PORT
    port_starboard_direction = 1; // 1 is port
    starboard_overcurrent = 0; // reset the overcurrent port fault because we go in the opposite direction
    if (port_overcurrent == 0) {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, targetSpeed);
      Serial.print("..Motor engaged, PORT speed (0-255): ");
      Serial.println(targetSpeed);
      Serial.print("Max current: ");
      Serial.println(rollingAverageCurrent * .01, 2); // back to amps

    } else {
      stop_port();
    }
    if (debugMode) {
      Serial.print("..Motor engaged, PORT speed (0-255): ");
      Serial.println(targetSpeed);
    }
  }
}


void sendFeedback() {
  if (Serial1.availableForWrite() < 32) return; // Skip if buffer nearly full

  uint8_t feedback[4];

//  // Update rudder position based on motor direction
//  if (port_starboard_direction == 1 && motorEngaged && !port_overcurrent && PortValue != LOW) {
//   // rudderSense += 5; // Port
//  } else if (port_starboard_direction == 2 && motorEngaged && !starboard_overcurrent && StarboardValue != LOW) {
//  //  rudderSense -= 5; // Starboard
//  }
//  if (rudderSense > 1000) rudderSense = 1000;
//  if (rudderSense < -1000) rudderSense = -1000;

  // Current Feedback (units: 10mA)
  uint16_t currentAmpsInt = static_cast<uint16_t>(rollingAverageCurrent);
  feedback[0] = CURRENT_CODE;
  feedback[1] = currentAmpsInt & 0xFF;
  feedback[2] = (currentAmpsInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  Serial1.write(feedback, 4);

  // Voltage Feedback (units: 10mV)
  uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);
  feedback[0] = VOLTAGE_CODE;
  feedback[1] = voltageInt & 0xFF;
  feedback[2] = (voltageInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  Serial1.write(feedback, 4);

  // Controller Temperature Feedback (units: °C)
  uint16_t controllerTempInt = static_cast<uint16_t>(controllerTemp);
  feedback[0] = CONTROLLER_TEMP_CODE;
  feedback[1] = controllerTempInt & 0xFF;
  feedback[2] = (controllerTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  Serial1.write(feedback, 4);

  // Motor Temperature Feedback (units: °C)
  uint16_t motorTempInt = static_cast<uint16_t>(motorTemp);
  feedback[0] = MOTOR_TEMP_CODE;
  feedback[1] = motorTempInt & 0xFF;
  feedback[2] = (motorTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  Serial1.write(feedback, 4);

//  // Rudder Feedback
//  feedback[0] = RUDDER_SENSE_CODE;
//  feedback[1] = rudderSense & 0xFF;
//  feedback[2] = (rudderSense >> 8) & 0xFF;
//  feedback[3] = crc8(feedback, 3);
//  Serial1.write(feedback, 4);

  // Flags Feedback
  uint16_t faultFlags = flags; // Start with global flags (SYNC, REBOOTED)
  if (rollingAverageCurrent > currentLimit) {
    faultFlags |= OVERCURRENT_FAULT;
  }
  if (motorEngaged) {
    faultFlags |= ENGAGED;
  }
//  if (port_overcurrent || PortValue == LOW) {
//    faultFlags |= PORT_PIN_FAULT | MIN_RUDDER;
//  }
//  if (starboard_overcurrent || StarboardValue == LOW) {
//    faultFlags |= STARBOARD_PIN_FAULT | MAX_RUDDER;
//  }
  feedback[0] = FLAGS_CODE;
  feedback[1] = faultFlags & 0xFF;
  feedback[2] = (faultFlags >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3);
  Serial1.write(feedback, 4);

  if (debugMode && (faultFlags & OVERCURRENT_FAULT)) {
    Serial.println("Overcurrent fault detected!");
  }
}



void sendHandshake() {
  uint8_t handshake[4];
  handshake[0] = HANDSHAKE_CODE;   // Handshake code (e.g., 0x5A)
  handshake[1] = 0x00;            // Reserved byte
  handshake[2] = 0x00;            // Reserved byte
  handshake[3] = crc8(handshake, 3); // Calculate CRC for the first 3 bytes

  Serial1.write(handshake, 4);    // Send the 4-byte handshake packet
}


void parseCommand(uint8_t *command) {
  static unsigned long lastCommandTime = 0; // Track last command time
  uint8_t cmd = command[0];
  uint16_t value = command[1] | (command[2] << 8);

  isSynced = true;
  lastCommandTime = millis();

  // Log raw packet for debugging
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
    case HANDSHAKE_CODE: // 0x5A
      feedback[0] = HANDSHAKE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      Serial1.write(feedback, 4);
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
      break;
    case MAX_SLEW_CODE:
      maxSlewRate = command[1];
      max_slew_slow = command[2];
      break;
    case EEPROM_READ_CODE:
      {
        uint16_t address = value;
        // Send multiple EEPROM responses
        // Address 0: Max current
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = currentLimit & 0xFF;
        feedback[2] = (currentLimit >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        Serial1.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 0");

        // Address 2: Config flags
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = 0x01; // Indicate basic motor type
        feedback[2] = 0x00;
        feedback[3] = crc8(feedback, 3);
        Serial1.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 2");

        // Address 4: Rudder range
        uint16_t range = 4000; // Try larger range
        feedback[0] = EEPROM_VALUE_CODE;
        feedback[1] = range & 0xFF;
        feedback[2] = (range >> 8) & 0xFF;
        feedback[3] = crc8(feedback, 3);
        Serial1.write(feedback, 4);
        delay(10);
        if (debugMode) Serial.println("Sent EEPROM response for address 4");
      }
      break;
    case EEPROM_WRITE_CODE:
      feedback[0] = EEPROM_VALUE_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      Serial1.write(feedback, 4);
      break;
    case RUDDER_RANGE_CODE:
      feedback[0] = RUDDER_RANGE_CODE;
      uint16_t range = 4000; // Match EEPROM range
      feedback[1] = range & 0xFF;
      feedback[2] = (range >> 8) & 0xFF;
      feedback[3] = crc8(feedback, 3);
      Serial1.write(feedback, 4);
      break;
    case REPROGRAM_CODE:
      feedback[0] = REPROGRAM_CODE;
      feedback[1] = 0x00;
      feedback[2] = 0x00;
      feedback[3] = crc8(feedback, 3);
      Serial1.write(feedback, 4);
      break;
    default:
      if (debugMode) {
        Serial.print("Unknown command: 0x");
        Serial.println(cmd, HEX);
      }
      break;
  }

  // Reset SYNC if no commands for 2 seconds
  if (millis() - lastCommandTime > 2000) {
    isSynced = false;
    flags &= ~SYNC; // Clear SYNC flag
  }
}

bool verifyCRC(uint8_t *data) {
  uint8_t crc = crc8(data, 3);
  return crc == data[3];
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);
 // Serial1.setRxBufferSize(256); // Increase buffer

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(Port_Switch_pin, INPUT_PULLUP);
  pinMode(Starboard_Switch_pin, INPUT_PULLUP);
  pinMode(IS_PINS, INPUT);

  speed = 1000;
  stopMotor();
  digitalWrite(REN, HIGH);

  Serial.println("Arduino motor controller initialized");

  // Send initial handshake
  sendHandshake();
}

void loop() {
static uint8_t buffer[4];
static uint8_t bufferIndex = 0;
static uint8_t invalidCrcCount = 0;
const uint8_t MAX_INVALID_CRC = 5; // Retry threshold

// Process one byte per loop iteration
if (Serial1.available() > 0) {
  if (bufferIndex >= 4) {
    if (debugMode) Serial.println("Buffer overflow, resetting");
    bufferIndex = 0;
    invalidCrcCount++;
  } else {
    uint8_t receivedByte = Serial1.read();
    buffer[bufferIndex++] = receivedByte;
    if (bufferIndex == 4) {
      if (verifyCRC(buffer)) {
        parseCommand(buffer);
        invalidCrcCount = 0; // Reset on valid packet
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
  // Flush serial buffer and retry after too many invalid packets
  if (invalidCrcCount >= MAX_INVALID_CRC) {
    while (Serial1.available() > 0) Serial1.read(); // Clear buffer
    invalidCrcCount = 0;
    if (debugMode) Serial.println("Flushed Serial1 due to excessive invalid CRCs");
    sendHandshake(); // Re-initiate communication
  }
}

  // Read the current sense value
  int currentSenseRaw = analogRead(IS_PINS);
  currentAmps = floatMap(currentSenseRaw, 0, 1023, 0, 4300); // Units: 10mA

  // Update rolling average
  currentAmpsHistory[currentAmpsIndex] = currentAmps;
  currentAmpsIndex = (currentAmpsIndex + 1) % ROLLING_AVG_SIZE;
  rollingAverageCurrent = 0.0;
  for (int i = 0; i < ROLLING_AVG_SIZE; i++) {
    rollingAverageCurrent += currentAmpsHistory[i];
  }
  rollingAverageCurrent /= ROLLING_AVG_SIZE;

  // Stop motor if rolling average exceeds current limit
  if (rollingAverageCurrent > currentLimit) {
    if (port_starboard_direction == 1) {
      stop_port();
    }
    if (port_starboard_direction == 2) {
      stop_starboard();
    }
    if (debugMode) {
      Serial.println("Motor stopped due to overcurrent!");
      Serial.print("Max current set to: ");
      Serial.println(currentLimit);
      Serial.print("Max current: ");
      Serial.println(rollingAverageCurrent * 0.01, 2);
    }
  }

  PortValue = digitalRead(Port_Switch_pin);
  if (port_starboard_direction == 1 && PortValue == LOW) {
    stop_port();
  }

  StarboardValue = digitalRead(Starboard_Switch_pin);
  if (port_starboard_direction == 2 && StarboardValue == LOW) {
    stop_starboard();
  }

  // Periodic tasks
  static unsigned long lastFeedbackTime = 0;
  static unsigned long lastHandshakeTime = 0;
  unsigned long currentMillis = millis();

  // Send telemetry feedback every 100ms
  if (currentMillis - lastFeedbackTime > 100) {
    sendFeedback();
    lastFeedbackTime = currentMillis;
  }

  // Send handshake every 1000ms
  if (currentMillis - lastHandshakeTime > 1000) {
    sendHandshake();
    lastHandshakeTime = currentMillis;
  }
}
