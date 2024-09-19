#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// Define stepper motor connections and steps per revolution
#define dirPin 5
#define stepPin 6
#define stepsPerRevolution 400  // Typical value for steps per revolution

// Initialize the stepper library on the pins you want to use
AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);

// Define maximum speed and acceleration
#define ACCELERATION 4000  // Adjusted for practical acceleration
#define MAX_SPEED 5000     // Adjusted for practical speed

// Define digital pins for sensors
#define LIMIT_SENSOR_MINUS 2    // Pin for sensor at -46 degrees (Digital pin 2)
#define LIMIT_SENSOR_PLUS 3     // Pin for sensor at +30 degrees (Digital pin 3)
#define LIMIT_SENSOR_0 4        // Pin for sensor at 0 degrees (Digital pin 4 polled)

// Define states for calibration
enum CalibrationStep {
  CALIBRATION_IDLE,
  MOVE_TO_POSITIVE_LIMIT,
  WAIT_FOR_POSITIVE_LIMIT,
  MOVE_TO_NEGATIVE_LIMIT,
  WAIT_FOR_NEGATIVE_LIMIT,
  CALIBRATION_DONE
};
CalibrationStep calibrationStep = CALIBRATION_IDLE;

// Variables for state change detection (pin 4)
int lastSensor0State = HIGH;  // Start with the assumption that the sensor is HIGH (not triggered)
bool sensor0Triggered = false;  // Flag to know if sensor has been triggered

// Define global variables
bool step_adjustment_multiplier = 1;
float steps_per_incline_degree = 0.0;
int eepromAddress = 0;  // Starting address in EEPROM to store the value

// EEPROM addresses for position and validity flag
const int EEPROM_STEPS_PER_DEGREE_ADDR = 0;      // 4 bytes
const int EEPROM_POSITIVE_LIMIT_ADDR = 4;        // 4 bytes
const int EEPROM_NEGATIVE_LIMIT_ADDR = 8;        // 4 bytes
const int EEPROM_CURRENT_POSITION_ADDR = 12;     // 4 bytes
const int EEPROM_POSITION_VALIDITY_ADDR = 16;    // 1 byte

// Define variables to track previous values displayed on the LCD
int lastSpeed = -1;
int lastInclineSetpoint = -1;

int speed = 0;             // Speed in mm/min
int inclineSetpoint = 0;   // Incline angle in degrees
int inclineAngle = 0;
bool movingToSetpoint = false;  // Flag to check if moving
bool paused = false;

// Define a flag to store the previous button state
int lastButtonState = -1;  // -1 means no button pressed
unsigned long lastDebounceTime = 0;  // Time the button was last toggled
const unsigned long debounceDelay = 50;  // Debounce time (in milliseconds)

// Variables for continuous button hold
unsigned long lastButtonHoldTime = 0;  // Time of the last button hold action
const unsigned long holdInterval = 200;  // Interval to increment or decrement while holding the button

// Limit Switch Variables
volatile bool limit_negative_triggered = false;
volatile bool limit_positive_triggered = false;
bool limit_0 = false;

// Debounce variables for limit switches
const unsigned long limitSwitchDebounceDelay = 50;  // 50 ms debounce time
unsigned long limitNegativeDebounceTime = 0;
unsigned long limitPositiveDebounceTime = 0;
int lastLimitNegativeState = HIGH;
int lastLimitPositiveState = HIGH;

// Variables to store limit switch positions
long pos_negative = 0;   // Position at -46 degrees
long pos_positive = 0;   // Position at +30 degrees
bool calibrating = false;  // Flag to check if in calibration mode

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);

  // Set the maximum speed and acceleration
  myStepper.setMaxSpeed(MAX_SPEED);
  myStepper.setAcceleration(ACCELERATION);

  // Initialize the LCD with 16 columns and 2 rows
  lcd.begin(16, 2);

  // Turn on the backlight
  lcd.backlight();

  // Print a welcome message on the LCD
  lcd.setCursor(0, 0);
  lcd.print("Zazuwall SA");
  lcd.setCursor(0, 1);
  lcd.print("Let's Climb!");

  delay(2000);

  // Clear the screen after the welcome message
  lcd.clear();

  // Set the sensor pins as inputs with pull-up resistors
  pinMode(LIMIT_SENSOR_0, INPUT_PULLUP);
  pinMode(LIMIT_SENSOR_MINUS, INPUT_PULLUP);
  pinMode(LIMIT_SENSOR_PLUS, INPUT_PULLUP);

  // Read steps per degree from EEPROM
  steps_per_incline_degree = readFloatFromEEPROM(EEPROM_STEPS_PER_DEGREE_ADDR);
  Serial.print("Loaded Steps per Degree from EEPROM: ");
  Serial.println(steps_per_incline_degree);

  // Read the saved position validity flag
  byte positionValid = EEPROM.read(EEPROM_POSITION_VALIDITY_ADDR);

  if (steps_per_incline_degree <= 0.0) {
    Serial.println("Running auto-calibration because stored value is invalid...");
    run_auto_calibration();
  } else if (positionValid == 1) {
    // Read the saved current position
    long savedPosition = readLongFromEEPROM(EEPROM_CURRENT_POSITION_ADDR);
    myStepper.setCurrentPosition(savedPosition);
    Serial.print("Restored current position from EEPROM: ");
    Serial.println(savedPosition);

    // Read the saved limit positions
    pos_negative = readLongFromEEPROM(EEPROM_NEGATIVE_LIMIT_ADDR);
    pos_positive = readLongFromEEPROM(EEPROM_POSITIVE_LIMIT_ADDR);
    Serial.print("Restored negative limit position: ");
    Serial.println(pos_negative);
    Serial.print("Restored positive limit position: ");
    Serial.println(pos_positive);
  } else {
    // Position is not valid, prompt for calibration or manual positioning
    Serial.println("Position not valid. Please calibrate or move to known position.");
    lcd.setCursor(0, 0);
    lcd.print("Pos Invalid!");
    lcd.setCursor(0, 1);
    lcd.print("Calibrate (Btn1)");
  }
}

void loop() {
  static int currentButtonState = -1;

  // Read the analog value from A0 for button presses
  int analogValue = analogRead(A0);

  // Convert the analog value to voltage (assuming a 5V reference)
  float voltage = analogValue * (5.0 / 1023.0);

  if (movingToSetpoint) {
    // Continuously check the limit switches during movement
    handleLimitSwitchesDuringMovement();
    myStepper.run();
  }

  // Handle calibration steps
  if (calibrating) {
    handleCalibration();
  } else {
    // Determine which button is pressed based on voltage range
    int buttonState = -1;  // Default to -1 for no button pressed
    if (voltage > 4.8 && voltage <= 5.0) {
      buttonState = 0;  // No button pressed
    } 
    else if (voltage >= 0.0 && voltage <= 0.1) {
      buttonState = 1;  // Button 1 pressed (Calibration)
    }
    else if (voltage >= 0.6 && voltage <= 0.8) {
      buttonState = 2;  // Button 2 pressed (Increase Incline)
    }
    else if (voltage >= 1.5 && voltage <= 1.7) {
      buttonState = 3;  // Button 3 pressed (Decrease Incline)
    }
    else if (voltage >= 2.4 && voltage <= 2.6) {
      buttonState = 4;  // Button 4 pressed (Decrease Speed)
    }
    else if (voltage >= 3.5 && voltage <= 3.7) {
      buttonState = 5;  // Button 5 pressed (Set/Pause Movement)
    }

    // Check if the button state has changed
    if (buttonState != lastButtonState) {
      // Reset the debounce timer
      lastDebounceTime = millis();
      lastButtonHoldTime = millis();  // Reset the hold time
    }

    // Check if the debounce time has passed
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // Only act if the button state has changed or is held
      if (buttonState != currentButtonState || (millis() - lastButtonHoldTime >= holdInterval && buttonState != 0)) {
        currentButtonState = buttonState;
        lastButtonHoldTime = millis();  // Update the hold time for continuous action

        // Perform actions based on the button state
        switch (currentButtonState) {
          case 1:  // Button 1 pressed - start calibration
            run_auto_calibration();
            break;
          case 2:  // Button 2 pressed - increase incline angle
            if (!movingToSetpoint && !paused) {
              inclineSetpoint += 5;
              if (inclineSetpoint > 30) inclineSetpoint = 30;  // Limit to +30 degrees
              Serial.print("Setpoint increased to: ");
              Serial.println(inclineSetpoint);
            }
            break;
          case 3:  // Button 3 pressed - decrease incline angle
            if (!movingToSetpoint && !paused) {
              inclineSetpoint -= 5;
              if (inclineSetpoint < -46) inclineSetpoint = -46;  // Limit to -46 degrees
              Serial.print("Setpoint decreased to: ");
              Serial.println(inclineSetpoint);
            }
            break;
          case 4:  // Button 4 pressed - decrease speed
            speed -= 1;
            if (speed < 0) speed = 0;  // Prevent speed from going negative
            break;
          case 5:  // Button 5 pressed - Move to Setpoint or Pause Movement
            if (!paused) {
              if (!movingToSetpoint) {
                // Start moving to the setpoint
                Serial.print("Moving to setpoint: ");
                Serial.println(inclineSetpoint);
                moveToSetpoint(inclineSetpoint);
                movingToSetpoint = true;
              } else {
                // Pause the movement if already moving
                Serial.println("Movement paused");
                paused = true;
                myStepper.stop();
              }
            } else {
              // Resume movement
              Serial.println("Resuming movement to setpoint");
              paused = false;
                moveToSetpoint(inclineSetpoint);  // Resume moving to the setpoint
            }
            break;
        }
      }
    }

    // Update the lastButtonState
    lastButtonState = buttonState;

    // Check if the stepper has reached the current target position
    if (movingToSetpoint && myStepper.distanceToGo() == 0) {
      Serial.println("Reached setpoint");
      movingToSetpoint = false;

      // Save the current position to EEPROM
      saveCurrentPositionToEEPROM();
    }

    // Run the stepper to the new position if needed
    if (!paused && !movingToSetpoint) {
      myStepper.run();
    }

    // Call the updateLCD() function to refresh the screen only if needed
    updateLCD();
  }
}

void run_auto_calibration() {
  if (calibrating) {
    // Already calibrating
    return;
  }

  calibrating = true;
  calibrationStep = MOVE_TO_POSITIVE_LIMIT;

  // Display calibration info on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");

  Serial.println("Starting auto calibration...");

  // Reset limit flags
  limit_negative_triggered = false;
  limit_positive_triggered = false;

  pos_negative = 0;
  pos_positive = 0;

  // Start moving towards positive limit
  myStepper.setAcceleration(ACCELERATION);
  myStepper.setMaxSpeed(MAX_SPEED);
  myStepper.moveTo(1000000);  // Move towards positive limit (large positive number)
}

void handleCalibration() {
  switch (calibrationStep) {
    case MOVE_TO_POSITIVE_LIMIT:
      // Already started moving towards positive limit in run_auto_calibration()
      calibrationStep = WAIT_FOR_POSITIVE_LIMIT;
      break;

    case WAIT_FOR_POSITIVE_LIMIT:
      myStepper.run();
      {
        int currentLimitPositiveState = digitalRead(LIMIT_SENSOR_PLUS);
        if (currentLimitPositiveState != lastLimitPositiveState) {
          limitPositiveDebounceTime = millis(); // Reset debounce timer
        }

        if ((millis() - limitPositiveDebounceTime) > limitSwitchDebounceDelay) {
          if (currentLimitPositiveState == LOW) {
            // Limit switch has been triggered
            myStepper.stop();
            myStepper.run();  // Ensure the motor stops
            pos_positive = myStepper.currentPosition();
            myStepper.setCurrentPosition(pos_positive);  // Set current position
            Serial.print("Position at +30 degrees: ");
            Serial.println(pos_positive);
            lcd.setCursor(0, 1);
            lcd.print("Reached +30       ");
            delay(500);  // Short pause
            calibrationStep = MOVE_TO_NEGATIVE_LIMIT;
          }
        }
        lastLimitPositiveState = currentLimitPositiveState;
      }
      break;

    case MOVE_TO_NEGATIVE_LIMIT:
      // Start moving towards negative limit
      myStepper.moveTo(-1000000);  // Move towards negative limit (large negative number)
      calibrationStep = WAIT_FOR_NEGATIVE_LIMIT;
      break;

    case WAIT_FOR_NEGATIVE_LIMIT:
      myStepper.run();
      {
        int currentLimitNegativeState = digitalRead(LIMIT_SENSOR_MINUS);
        if (currentLimitNegativeState != lastLimitNegativeState) {
          limitNegativeDebounceTime = millis(); // Reset debounce timer
        }

        if ((millis() - limitNegativeDebounceTime) > limitSwitchDebounceDelay) {
          if (currentLimitNegativeState == LOW) {
            // Limit switch has been triggered
            myStepper.stop();
            myStepper.run();  // Ensure the motor stops
            pos_negative = myStepper.currentPosition();
            myStepper.setCurrentPosition(pos_negative);  // Set current position
            Serial.print("Position at -46 degrees: ");
            Serial.println(pos_negative);
            lcd.setCursor(0, 1);
            lcd.print("Reached -46       ");
            delay(500);  // Short pause

            // Move slightly away from the limit switch
            myStepper.moveTo(pos_negative + steps_per_incline_degree * 1);  // Move 1 degree away
            while (myStepper.distanceToGo() != 0) {
              myStepper.run();
            }

            calibrationStep = CALIBRATION_DONE;
          }
        }
        lastLimitNegativeState = currentLimitNegativeState;
      }
      break;

    case CALIBRATION_DONE:
      // Calculate the steps between -46 and +30 degrees
      long steps_between = pos_positive - pos_negative;
      Serial.print("Steps between -46 and +30 degrees: ");
      Serial.println(steps_between);

      // Prevent division by zero
      if (steps_between > 0) {
        steps_per_incline_degree = (float)steps_between / 76.0;  // 76 degrees total
        lcd.setCursor(0, 1);
        lcd.print("Calibrated!        ");
      } else {
        steps_per_incline_degree = 0;
        Serial.println("Error: Invalid steps between positions. Check limit switches.");
        lcd.setCursor(0, 1);
        lcd.print("Calib Error!       ");
      }

      // Save the calculated value to EEPROM
      saveFloatToEEPROM(EEPROM_STEPS_PER_DEGREE_ADDR, steps_per_incline_degree);
      saveLongToEEPROM(EEPROM_NEGATIVE_LIMIT_ADDR, pos_negative);
      saveLongToEEPROM(EEPROM_POSITIVE_LIMIT_ADDR, pos_positive);
      Serial.println("Calibration complete, values saved to EEPROM.");

      // Invalidate current position (since we are at a known position)
      myStepper.setCurrentPosition(pos_negative + steps_per_incline_degree);  // After moving away from limit
      saveCurrentPositionToEEPROM();

      // End calibration mode
      calibrating = false;
      calibrationStep = CALIBRATION_IDLE;

      // Calibration completed
      Serial.print("Steps per degree: ");
      Serial.println(steps_per_incline_degree);
      Serial.println("Calibration complete.");

      // Clear calibration info from the LCD after a short delay
      delay(2000);
      lcd.clear();
      updateLCD();  // Return to normal display
      break;

    default:
      break;
  }
}

void handleLimitSwitchesDuringMovement() {
  // Check limit switches
  int limitMinusState = digitalRead(LIMIT_SENSOR_MINUS);
  int limitPlusState = digitalRead(LIMIT_SENSOR_PLUS);

  // Debounce logic for limit switches
  if (limitMinusState != lastLimitNegativeState) {
    limitNegativeDebounceTime = millis(); // Reset debounce timer
  }
  if ((millis() - limitNegativeDebounceTime) > limitSwitchDebounceDelay) {
    if (limitMinusState == LOW) {
      // Limit switch is triggered
      if (myStepper.speed() < 0) {
        // Moving towards the limit switch
        myStepper.stop();
        myStepper.run();
        Serial.println("Reached -46 degrees limit during movement.");
        movingToSetpoint = false;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("At -46 degrees");
        lcd.setCursor(0, 1);
        lcd.print("Limit reached  ");

        // Save the current position to EEPROM
        saveCurrentPositionToEEPROM();
      } else if (myStepper.speed() > 0) {
        // Moving away from the limit switch, ignore
      }
    }
  }
  lastLimitNegativeState = limitMinusState;

  if (limitPlusState != lastLimitPositiveState) {
    limitPositiveDebounceTime = millis(); // Reset debounce timer
  }
  if ((millis() - limitPositiveDebounceTime) > limitSwitchDebounceDelay) {
    if (limitPlusState == LOW) {
      // Limit switch is triggered
      if (myStepper.speed() > 0) {
        // Moving towards the limit switch
        myStepper.stop();
        myStepper.run();
        Serial.println("Reached +30 degrees limit during movement.");
        movingToSetpoint = false;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("At +30 degrees");
        lcd.setCursor(0, 1);
        lcd.print("Limit reached  ");

        // Save the current position to EEPROM
        saveCurrentPositionToEEPROM();
      } else if (myStepper.speed() < 0) {
        // Moving away from the limit switch, ignore
      }
    }
  }
  lastLimitPositiveState = limitPlusState;
}

// Save float to EEPROM
void saveFloatToEEPROM(int address, float value) {
  byte* bytePointer = (byte*)(void*)&value;  // Cast float to byte pointer
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.update(address + i, bytePointer[i]);
  }
}

// Read float from EEPROM
float readFloatFromEEPROM(int address) {
  float value = 0.0;
  byte* bytePointer = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    bytePointer[i] = EEPROM.read(address + i);
  }
  return value;
}

// Save long to EEPROM
void saveLongToEEPROM(int address, long value) {
  byte* bytePointer = (byte*)(void*)&value;  // Cast long to byte pointer
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.update(address + i, bytePointer[i]);
  }
}

// Read long from EEPROM
long readLongFromEEPROM(int address) {
  long value = 0;
  byte* bytePointer = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    bytePointer[i] = EEPROM.read(address + i);
  }
  return value;
}

void saveCurrentPositionToEEPROM() {
  long currentPosition = myStepper.currentPosition();
  saveLongToEEPROM(EEPROM_CURRENT_POSITION_ADDR, currentPosition);
  EEPROM.update(EEPROM_POSITION_VALIDITY_ADDR, 1);  // Set position valid flag to 1
  Serial.print("Saved current position to EEPROM: ");
  Serial.println(currentPosition);
}

void invalidateCurrentPositionInEEPROM() {
  EEPROM.update(EEPROM_POSITION_VALIDITY_ADDR, 0);  // Set position valid flag to 0
}

void updateLCD() {
  // Update speed if it has changed
  if (speed != lastSpeed) {
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    lcd.print(speed);
    lcd.print(" mm/m   ");  // Add extra spaces to overwrite previous digits if speed decreases

    lastSpeed = speed;  // Store the current speed as the last speed
  }

  // Update incline setpoint if it has changed
  if (inclineSetpoint != lastInclineSetpoint) {
    lcd.setCursor(0, 1);
    lcd.print("Incline: ");
    lcd.print(inclineSetpoint);
    lcd.print(" deg   ");  // Add extra spaces to overwrite previous digits if incline decreases

    lastInclineSetpoint = inclineSetpoint;  // Store the current incline as the last incline
  }
}

void moveToSetpoint(int setpoint) {
  // Ensure the setpoint is within bounds (-46° to +30°)
  if (setpoint < -46) setpoint = -46;
  if (setpoint > 30) setpoint = 30;

  // Calculate the target position
  long targetPosition = (setpoint + 46) * steps_per_incline_degree + pos_negative;

  // Move the stepper motor to the calculated position
  myStepper.moveTo(targetPosition);

  // Print debug info
  Serial.print("Moving to incline setpoint: ");
  Serial.print(setpoint);
  Serial.print(" degrees. Target step position: ");
  Serial.println(targetPosition);

  // Start the stepper movement
  movingToSetpoint = true;
}

void handleLostSteps(int unexpectedPosition) {
  // Stop the motor immediately
  myStepper.stop();
  movingToSetpoint = false;

  // Log the error and notify the user via LCD
  Serial.print("Error: Limit switch triggered unexpectedly at position: ");
  Serial.println(unexpectedPosition);

  // Invalidate current position
  invalidateCurrentPositionInEEPROM();

  // Display error message on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Error: Lost Steps!");
  lcd.setCursor(0, 1);
  lcd.print("Recalibrating...");

  // Delay before recalibration to give time for the user to notice
  delay(5000);  // 5-second delay to show the error on LCD

  // Initiate recalibration
  Serial.println("Initiating recalibration...");
  run_auto_calibration();  // Recalibrate to realign the motor
}
