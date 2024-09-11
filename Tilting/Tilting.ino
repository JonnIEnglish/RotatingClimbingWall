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
#define ACCELERATION 10000  // Adjusted for practical acceleration
#define MAX_SPEED 1000     // Adjusted for practical speed

// Define digital pins for sensors
#define LIMIT_SENSOR_MINUS_45 2    // Pin for sensor at -45 degrees (Digital pin 2 with interrupt)
#define LIMIT_SENSOR_PLUS_15 3     // Pin for sensor at +15 degrees (Digital pin 3 with interrupt)
#define LIMIT_SENSOR_0 4           // Pin for sensor at 0 degrees (Digital pin 4 polled)

// Interrupt handler function declarations
void sensorMinus45Triggered();
void sensorPlus15Triggered();

// Variables for state change detection (pin 4)
int lastSensor0State = HIGH;  // Start with the assumption that the sensor is HIGH (not triggered)
bool sensor0Triggered = false;  // Flag to know if sensor has been triggered



// Define global variables
bool step_adjustment_multiplier = 1;
float steps_per_incline_degree = 0.0;
int eepromAddress = 0;  // Starting address in EEPROM to store the value

// Define variables to track previous values displayed on the LCD
int lastSpeed = -1;
int lastInclineSetpoint = -1;


int speed = 0;  // Speed in mm/min
int inclineSetpoint = 0;  // Incline angle in degrees
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


//Limit Switch Variables
bool limit_45 = false;
bool limit_15 = false;
bool limit_0 = false;

// Variables to store limit switch positions
long pos_45 = 0;   // Position at -45 degrees
long pos_15 = 0;   // Position at +15 degrees
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
  lcd.print("TreadWall SA");
  lcd.setCursor(0, 1);
  lcd.print("Let's Climb!");

  delay(2000);

  // Clear the screen after the welcome message
  lcd.clear();

    // Set the sensor pins as inputs
  pinMode(LIMIT_SENSOR_0, INPUT);
  pinMode(LIMIT_SENSOR_MINUS_45, INPUT);
  pinMode(LIMIT_SENSOR_PLUS_15, INPUT);

  // Attach interrupts for -45 and +15 degree sensors
  attachInterrupt(digitalPinToInterrupt(LIMIT_SENSOR_MINUS_45), sensorMinus45Triggered, FALLING);   // Triggered when voltage drops (metal detected)
  attachInterrupt(digitalPinToInterrupt(LIMIT_SENSOR_PLUS_15), sensorPlus15Triggered, FALLING);

    // Read steps per degree from EEPROM
  steps_per_incline_degree = readFloatFromEEPROM(eepromAddress);
  Serial.print("Loaded Steps per Degree from EEPROM: ");
  Serial.println(steps_per_incline_degree);

  if (steps_per_incline_degree <= 0.0) {
    Serial.println("Running auto-calibration because stored value is invalid...");
    run_auto_calibration();
  }


}


void loop() {
  static int currentButtonState = -1;

  // Read the analog value from A0 for button presses
  int analogValue = analogRead(A0);

  // Convert the analog value to voltage (assuming a 5V reference)
  float voltage = analogValue * (5.0 / 1023.0);

  // Determine which button is pressed based on voltage range
  int buttonState = -1;  // Default to -1 for no button pressed
  if (voltage > 4.8 && voltage <= 5.0) {
    buttonState = 0;  // No button pressed
  } 
  else if (voltage >= 0.0 && voltage <= 0.1) {
    buttonState = 1;  // Button 1 pressed
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
        case 1:  // Button 1 pressed - increase speed
          run_auto_calibration();
          break;
        case 2:  // Button 2 pressed - increase incline angle
          if (!movingToSetpoint && !paused) {
            inclineSetpoint += 5;
            if (inclineSetpoint > 15) inclineSetpoint = 15;  // Limit to +15 degrees
            Serial.print("Setpoint increased to: ");
            Serial.println(inclineSetpoint);
          }
          break;
        case 3:  // Button 3 pressed - decrease incline angle
          if (!movingToSetpoint && !paused) {
            inclineSetpoint -= 5;
            if (inclineSetpoint < -45) inclineSetpoint = -45;  // Limit to -45 degrees
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
  }

  // Run the stepper to the new position if needed
  if (!paused) {
    myStepper.run();
  }

  // Call the new updateLCD() function to refresh the screen only if needed
  updateLCD();
}


void run_auto_calibration() {
  // Set calibration flag
  calibrating = true;

  Serial.println("Starting auto calibration...");

  // Move the motor towards the -45 degrees limit switch
  myStepper.setSpeed(-500);  // Move in negative direction to find -45 degrees
  while (!limit_45) {
    myStepper.runSpeed();  // Continuously move until limit is hit
  }

  // Position is stored in the ISR for -45 degrees

  delay(500);  // Short pause before moving to +15 degrees

  // Now move the motor in the positive direction to find +15 degrees limit switch
  myStepper.setSpeed(500);  // Move in positive direction
  while (!limit_15) {
    myStepper.runSpeed();  // Continuously move until limit is hit
  }

  // Position is stored in the ISR for +15 degrees

  // Calculate the steps between -45 and +15 degrees
  long steps_between = pos_15 - pos_45;

  Serial.print("Steps between -45 and +15 degrees: ");
  Serial.println(steps_between);

  // Calculate steps per degree
  steps_per_incline_degree = (float)steps_between / 60.0;  // 60 degrees total

  // Save the calculated value to EEPROM
  saveFloatToEEPROM(eepromAddress, steps_per_incline_degree);
  Serial.println("Calibration complete, value saved to EEPROM.");

  // End calibration mode
  calibrating = false;

  // Calibration completed
  Serial.print("Steps per degree: ");
  Serial.println(steps_per_incline_degree);
  Serial.println("Calibration complete.");
}



// Interrupt service routine (ISR) for -45-degree sensor
void sensorMinus45Triggered() {
  Serial.println("Limit switch -45° triggered");
  limit_45 = true;
  inclineSetpoint = -45;
  myStepper.stop();  // Stop at the -45-degree limit

  if (calibrating) {
    pos_45 = myStepper.currentPosition();  // Store position for calibration
    Serial.print("Stored position at -45 degrees: ");
    Serial.println(pos_45);
  }
}

// Interrupt service routine (ISR) for +15-degree sensor
void sensorPlus15Triggered() {
  Serial.println("Limit switch +15° triggered");
  limit_15 = true;
  inclineSetpoint = 15;
  myStepper.stop();  // Stop at the +15-degree limit

  if (calibrating) {
    pos_15 = myStepper.currentPosition();  // Store position for calibration
    Serial.print("Stored position at +15 degrees: ");
    Serial.println(pos_15);
  }
}

// Save float to EEPROM
void saveFloatToEEPROM(int address, float value) {
  byte* bytePointer = (byte*)(void*)&value;  // Cast float to byte pointer
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address + i, bytePointer[i]);
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
  // Ensure the setpoint is within bounds (-45° to +15°)
  if (setpoint < -45) setpoint = -45;
  if (setpoint > 15) setpoint = 15;

  // Calculate the target position
  long targetPosition = (setpoint + 45) * steps_per_incline_degree + pos_45;

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


