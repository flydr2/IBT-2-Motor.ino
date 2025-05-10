// I'm currently working on this draft... Not ready for use
// in the process of adding a rudder sensor only based on overcurrent

// added current faults


//The enum are for my references
//enum commands {COMMAND_CODE=0xc7, RESET_CODE=0xe7, MAX_CURRENT_CODE=0x1e, MAX_CONTROLLER_TEMP_CODE=0xa4, MAX_MOTOR_TEMP_CODE=0x5a, RUDDER_RANGE_CODE=0xb6, RUDDER_MIN_CODE=0x2b, RUDDER_MAX_CODE=0x4d, REPROGRAM_CODE=0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71, EEPROM_READ_CODE=0x91, EEPROM_WRITE_CODE=0x53, CLUTCH_PWM_AND_BRAKE_CODE=0x36};

//enum results {CURRENT_CODE=0x1c, VOLTAGE_CODE=0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f, EEPROM_VALUE_CODE=0x9a};

//enum {SYNC = 1, OVERTEMP_FAULT = 2, OVERCURRENT_FAULT = 4, ENGAGED = 8, INVALID = 16, PORT_PIN_FAULT = 32, STARBOARD_PIN_FAULT = 64, BADVOLTAGE_FAULT = 128, MIN_RUDDER_FAULT = 256, MAX_RUDDER_FAULT = 512, CURRENT_RANGE = 1024, BAD_FUSES = 2048, /* PORT_FAULT=4096  STARBOARD_FAULT=8192 */ REBOOTED = 32768};


#include <Arduino.h>
#include "crc.h" // Ensure crc.h is available for CRC calculations
//#include <avr/boot.h> // Include for microcontroller signature identification

// Pin definitions
#define RPWM 10 //change these to your directions port/starboard
#define LPWM 9
#define REN 4 // for now these are wired to 5v (wire the 2 pins REN/LEN together to 1 arduino pin)
#define IS_PINS A1 // both L and R tied together lowers the IBT "IS pins" voltage output... use a resitor to gnd to lower more (need to calculate)


// Command and telemetry codes
#define DISENGAGE_CODE 0x68
#define ENGAGE_CODE 0x36
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


// Handshake and telemetry codes for Pypilot detection
#define HANDSHAKE_CODE 0x5A


// Global variables
int speed = 1000;
float currentAmps = 0.0;
float voltage = 12.0;
int currentLimit = 1024;// just a number at this point
int maxMotorTemp = 100; // Example value for max motor temp
int maxSlewRate = 20;   // Example max slew rate 1 to 255
int max_slew_slow = 20;
bool motorEngaged = false;
bool debugMode = false;  // Toggle debug mode
uint16_t flags;
int port_starboard_direction; //Set the present motor direction 0=no direction 1=port 2=strarboard
int starboard_overcurrent = 0;
int port_overcurrent = 0;
// Define variables for feedback
float controllerTemp = 25.0; // Example starting temperature for controller
float motorTemp = 30.0;      // Example starting temperature for motor



#define ROLLING_AVG_SIZE 10
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

void stop_port()
{
  stopMotor();
  port_overcurrent = 1;
  Serial.println("Port overCurrent");

}

void stop_starboard()
{
  stopMotor();
  starboard_overcurrent = 1;
  Serial.println("Starboard overCurrent");

}


void resetFaults() {
  // Reset all fault flags
  flags &= ~(OVERCURRENT_FAULT);//  add fauld to reset to the list here

  // Debug output for confirmation
  if (debugMode) {
    Serial.println("All faults have been reset.");
  }
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
 // if (debugMode) {
    Serial.print("Port overcurrent status = ");
    Serial.println(port_overcurrent);
    Serial.print("Starboard overcurrent status = ");
    Serial.println(starboard_overcurrent);
 // }
  if (speed < 1000) { // STARBOARD TURN
    port_starboard_direction = 2;// 2 is starboard direction
    port_overcurrent = 0; // reset the overcurrent starboard fault because we go in the opposite direction
    if (starboard_overcurrent == 0) {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, targetSpeed);
      Serial.print("..................Motor engaged, STARBOARD speed (0-255): ");
      Serial.println(targetSpeed);
    } else {
      stop_starboard();
    }
    if (debugMode) {
      Serial.print("..................Motor engaged, STARBOARD speed (0-255): ");
      Serial.println(targetSpeed);
    }

  } else if (speed > 1000) { // PORT
    port_starboard_direction = 1; // 1 is port
    starboard_overcurrent = 0; // reset the overcurrent port fault because we go in the opposite direction
    if (port_overcurrent == 0) {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, targetSpeed);
  //    Serial.print("..................Motor engaged, PORT speed (0-255): ");
  //    Serial.println(targetSpeed);

    } else {
      stop_port();
    }
    if (debugMode) {
      Serial.print("..................Motor engaged, PORT speed (0-255): ");
      Serial.println(targetSpeed);
    }
  }
}
void sendFeedback() {
  uint8_t feedback[4];

  // Send Current Feedback
  uint16_t currentAmpsInt = static_cast<uint16_t>(currentAmps); // current in units of 10mA
  feedback[0] = CURRENT_CODE;
  feedback[1] = currentAmpsInt & 0xFF;
  feedback[2] = (currentAmpsInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4);

  // Send Voltage Feedback
  uint16_t voltageInt = static_cast<uint16_t>(voltage * 100); // voltage in 10mV increments
  feedback[0] = VOLTAGE_CODE;
  feedback[1] = voltageInt & 0xFF;
  feedback[2] = (voltageInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4);

  // Send Controller Temperature Feedback
  uint16_t controllerTempInt = static_cast<uint16_t>(controllerTemp); // Replace with actual controller temperature value
  feedback[0] = CONTROLLER_TEMP_CODE;
  feedback[1] = controllerTempInt & 0xFF;
  feedback[2] = (controllerTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4);

  // Send Motor Temperature Feedback
  uint16_t motorTempInt = static_cast<uint16_t>(motorTemp); // Replace with actual motor temperature value
  feedback[0] = MOTOR_TEMP_CODE;
  feedback[1] = motorTempInt & 0xFF;
  feedback[2] = (motorTempInt >> 8) & 0xFF;
  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4);


  // Send Flags Feedback (including faults)
  uint8_t faultFlags = 0;

  // Check for overcurrent fault
  if (rollingAverageCurrent > currentLimit) {
    faultFlags |= OVERCURRENT_FAULT; // Set overcurrent fault flag
  }


  flags = faultFlags;
  feedback[0] = FLAGS_CODE;
  feedback[1] = faultFlags; // Send fault flags (overcurrent, rudder faults)
  feedback[2] = 0; // Reserved byte
  feedback[3] = crc8(feedback, 3); // Calculate CRC
  Serial1.write(feedback, 4);

  // Debug Mode Output
  if (debugMode) {
    Serial.println("All feedback sent.");
    if (faultFlags & OVERCURRENT_FAULT) {
      Serial.println("Overcurrent fault detected!");
    }

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
      if (debugMode) {
        Serial.print("DISENGAGE CODE: ");
        Serial.println(value);
      }

      break;

    case RESET_CODE:
      resetFaults();
      if (debugMode) {
        Serial.println("RESET command received. Faults reset.");
      }
      break;

    case ENGAGE_CODE: //
      //setMotorSpeed(value);
      if (debugMode) {
        Serial.print("ENGAGE CODE: ");
        Serial.println(value);
      }
      break;

    case COMMAND_CODE: // Includes speed and direction control
      setMotorSpeed(value);
      if (debugMode) {
        Serial.print("MOTOR SPEED CODE: ");
        Serial.println(value);
      }
      break;

    case MAX_CONTROLLER_TEMP_CODE:
      if (debugMode) {
        Serial.print("Max controller temperature set to: ");
        Serial.println(value);
      }
      break;

    case MAX_CURRENT_CODE:// current in units of 10mA
      currentLimit = value;
      if (debugMode) {
        Serial.print("Max current set to: ");
        Serial.println(currentLimit);
        Serial.print("Max current: ");
        Serial.println(currentAmps);
      }
      break;

    case MAX_MOTOR_TEMP_CODE: //not used here
      maxMotorTemp = value;
      //      if (debugMode) {
      //        Serial.print("Max motor temperature set to: ");
      //        Serial.println(maxMotorTemp);
      //      }
      break;


    case MAX_SLEW_CODE:
      // Ensure buffer has enough data
      maxSlewRate = command[1];
      max_slew_slow = command[2];

      // if set at the end of range (up to 255)  no slew limit
      if (maxSlewRate > 250)
        maxSlewRate = 255;
      if (max_slew_slow > 250)
        max_slew_slow = 255;
      // must have some slew
      if (maxSlewRate < 1)
        maxSlewRate = 1;
      if (max_slew_slow < 1)
        max_slew_slow = 1;


      if (debugMode) {
        //        Serial.print("Max slew rate set to: ");
        //        Serial.println(maxSlewRate);
        //        Serial.print("Max slew SLOW set to: ");
        //        Serial.println(max_slew_slow);

      }
      break;

    case EEPROM_READ_CODE: //not used here
      //      if (debugMode) {
      //        Serial.print("EEPROM read request received for address: ");
      //        Serial.println(value);
      //      }
      //      // Add EEPROM read functionality here
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

  pinMode(IS_PINS, INPUT);

  speed = 1000;
  stopMotor();
  digitalWrite(REN, HIGH);


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
      } else {
        Serial.println("Invalid CRC received");
      }
      bufferIndex = 0;
    }
  }

  // Read the current sense value
  int currentSenseRaw = analogRead(IS_PINS);
  currentAmps = floatMap(currentSenseRaw, 0, 1023, 0, 4300); // Amps to be determined. Must read the datasheet

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
    if (port_starboard_direction == 1) { // Chect which side to stop
      stop_port();
    }
    if (port_starboard_direction == 2) {
      stop_starboard();
    }
    Serial.println("Motor stopped due to overcurrent!");
    Serial.print("Max current set to: ");
    Serial.println(currentLimit);
    Serial.print("Max current: ");
    Serial.println(currentAmps);

  }

  static unsigned long lastFeedbackTime = 0;
  static unsigned long lastHandshakeTime = 0;
  static unsigned long lastResetTime = 0;
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
  // Reset port starboard faults every 10sec to ensure not stuck
  //  if ((starboard_overcurrent != 0) || (port_overcurrent != 0)) {
  //
  //    if (currentMillis - lastResetTime > 10000) {
  //      starboard_overcurrent = 0;
  //      port_overcurrent = 0;
  //      lastResetTime = currentMillis;
  //    }
  //
  //  } else {
  //    lastResetTime = currentMillis;
  //  }
  //
}// End Loop
