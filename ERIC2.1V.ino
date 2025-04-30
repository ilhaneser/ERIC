
/*
 * Stepper Motor Control with Position Tracking in Millimeters
 * Position calculation: 1200 pulses = 10mm (1cm)
 *
 * Available Commands:
 * ------------------
 * ON - Turn motor on
 * OFF - Turn motor off
 * FWD - Set direction forward
 * REV - Set direction backward
 * FREQ:1000 - Set pulse frequency to 1000Hz
 * MOVE:1000 - Generate 1000 pulses
 * MOVE_MM:10 - Move 10 millimeters
 * HOME - Home the motor (move back until kill switch activated, then forward until deactivated)
 * LOAD - Move to load position (240mm forward from home) - Only works at HOME position (0.0mm)
 * POS - Show current position
 * STOP - Stop pulse generation
 * STATUS - Get system status including kill switch
 * TEST_KILL - Test the kill switch state
 * ZERO - Set current position to zero
 *
 * Note: The LOAD command can only be executed when position is exactly at 0.0mm.
 * If position is greater than 0, attempting to use LOAD will display an error message.
 */

// Pin definitions
const int MOTOR_ON_PIN = 10;     // Motor enable pin
const int DIRECTION_PIN = 5;     // Direction control pin
const int JOG_PIN = 2;           // Jog pulse pin
const int KILL_SWITCH_PIN = 13;  // Kill switch input pin (on pin 13)

// Movement parameters
unsigned long pulseFrequency = 1000;   // Default frequency (Hz)
unsigned long numPulses = 1000;        // Default number of pulses
bool isRunning = false;                // Whether the motor is running
bool motorDirection = true;            // true = forward (LOW), false = backward (HIGH)
long pulsePosition = 0;                // Current position in pulses
float mmPosition = 0.0;                // Current position in millimeters
bool load_allowed = false;             // Flag to allow LOAD command only at HOME position
const float PULSES_PER_MM = 120.0;     // Calibration: 1200 pulses = 10mm, so 120 pulses = 1mm
const float LOAD_POSITION_MM = 240.0;  // Load position in millimeters

// Kill switch state
bool killSwitchActivated = false;    // Whether the kill switch is currently activated
unsigned long debounceDelay = 50;    // Debounce time in milliseconds
unsigned long lastDebounceTime = 0;  // Last time the kill switch changed state
int lastSwitchReading = HIGH;        // Last raw reading from kill switch

// Homing parameters
bool isHoming = false;  // Whether we're in homing mode
int homingStage = 0;    // 0=moving back, 1=moving forward

// Timing variables
unsigned long lastPulseTime = 0;
unsigned long pulsesGenerated = 0;
unsigned long pulseInterval;  // Microseconds between pulses

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Stepper Motor Control with Position in Millimeters");

  // Set up pins
  pinMode(MOTOR_ON_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(JOG_PIN, OUTPUT);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);  // Use internal pull-up resistor

  // Initial states
  digitalWrite(MOTOR_ON_PIN, LOW);   // Motor initially off
  digitalWrite(DIRECTION_PIN, LOW);  // Forward direction
  digitalWrite(JOG_PIN, LOW);        // Jog pin initially low

  // Calculate initial pulse interval (in microseconds)
  updatePulseInterval();

  // Print instructions
  Serial.println("Available commands:");
  Serial.println("ON - Turn motor on");
  Serial.println("OFF - Turn motor off");
  Serial.println("FWD - Set direction forward");
  Serial.println("REV - Set direction backward");
  Serial.println("FREQ:1000 - Set pulse frequency to 1000Hz");
  Serial.println("MOVE:1000 - Generate 1000 pulses");
  Serial.println("MOVE_MM:10 - Move 10 millimeters");
  Serial.println("HOME - Home the motor (move back until kill switch activated, then forward until deactivated)");
  Serial.println("LOAD - Move to load position (240mm forward from home)");
  Serial.println("POS - Show current position");
  Serial.println("STOP - Stop pulse generation");
  Serial.println("STATUS - Get system status including kill switch");
  Serial.println("TEST_KILL - Test the kill switch state");
  Serial.println("ZERO - Set current position to zero");

  Serial.println("\nKILL SWITCH WIRING (with REVERSED logic):");
  Serial.println("- Connect one wire to GND (ground)");
  Serial.println("- Connect other wire to pin 13");
  Serial.println("- HIGH = ACTIVATED (switch not pressed/open circuit)");
  Serial.println("- LOW = DEACTIVATED (switch pressed/closed circuit)");

  // Initial reading of kill switch
  int initialReading = digitalRead(KILL_SWITCH_PIN);
  Serial.print("Initial kill switch reading: ");
  Serial.println(initialReading == HIGH ? "HIGH (ACTIVATED)" : "LOW (DEACTIVATED)");
}

void loop() {
  // Check kill switch state with debouncing
  checkKillSwitch();

  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }

  // Generate pulses if running
  if (isRunning) {
    generatePulses();
  }

  // Handle homing process if active
  if (isHoming) {
    processHoming();
  }
}

void processHoming() {
  // Stage 0: Moving backward until kill switch is activated
  if (homingStage == 0) {
    if (killSwitchActivated) {
      // Switch to stage 1 - moving forward
      stopPulses();
      homingStage = 1;
      Serial.println("Homing: Kill switch activated - Now moving forward until switch is deactivated");

      // Start moving forward
      motorDirection = true;
      digitalWrite(DIRECTION_PIN, LOW);  // Forward
      startPulses();
    }
  }
  // Stage 1: Moving forward until kill switch is deactivated
  else if (homingStage == 1) {
    if (!killSwitchActivated) {
      // Homing complete
      stopPulses();
      isHoming = false;
      homingStage = 0;

      // Set position to 0 and enable load_allowed
      pulsePosition = 0;
      mmPosition = 0.0;
      load_allowed = true;  // Enable LOAD command
      Serial.println("Homing complete - Position set to 0.0mm");
      Serial.println("LOAD command is now allowed");
    }
  }
}

void checkKillSwitch() {
  // Read the current state of the kill switch
  // With REVERSED logic:
  // - HIGH = ACTIVATED (when switch is not pressed/open circuit)
  // - LOW = DEACTIVATED (when switch is pressed/closed circuit)
  int reading = digitalRead(KILL_SWITCH_PIN);

  // REVERSED logic: HIGH means it's activated (not pressed)
  bool currentlyActivated = (reading == HIGH);

  // If the switch state changed, reset the debounce timer
  if (reading != lastSwitchReading) {
    lastDebounceTime = millis();
  }

  // Only consider switch state change after debounce time
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the switch reading has stabilized and is different from current state
    if (currentlyActivated != killSwitchActivated) {
      killSwitchActivated = currentlyActivated;

      // Only print message after debounce to avoid multiple messages
      if (killSwitchActivated) {
        Serial.println("KILL SWITCH ACTIVATED (NOT PRESSED - HIGH)");
      } else {
        Serial.println("KILL SWITCH DEACTIVATED (PRESSED - LOW)");
      }
    }
  }

  // Save the reading for next comparison
  lastSwitchReading = reading;
}

void processCommand(String command) {
  // Process ON/OFF commands
  if (command == "ON") {
    digitalWrite(MOTOR_ON_PIN, HIGH);
    Serial.println("Motor ON");
  } else if (command == "OFF") {
    digitalWrite(MOTOR_ON_PIN, LOW);
    Serial.println("Motor OFF");
  }
  // Process direction commands
  else if (command == "FWD") {  // Forward is LOW to Direction Pin
    motorDirection = true;
    digitalWrite(DIRECTION_PIN, LOW);  // Forward
    Serial.println("Direction: FORWARD");
  } else if (command == "REV") {  // Reverse is HIGH to Direction Pin
    motorDirection = false;
    digitalWrite(DIRECTION_PIN, HIGH);  // Reverse
    Serial.println("Direction: REVERSE");
  }
  // Process frequency command
  else if (command.startsWith("FREQ:")) {
    pulseFrequency = command.substring(5).toInt();
    updatePulseInterval();
    Serial.print("Frequency set to: ");
    Serial.print(pulseFrequency);
    Serial.println("Hz");
  }
  // Process move command (pulses)
  else if (command.startsWith("MOVE:")) {
    numPulses = command.substring(5).toInt();
    startPulses();
  }
  // Process move command (millimeters)
  else if (command.startsWith("MOVE_MM:")) {
    float mm = command.substring(8).toFloat();
    numPulses = (unsigned long)(mm * PULSES_PER_MM);
    startPulses();

    Serial.print("Moving ");
    Serial.print(mm);
    Serial.print("mm (");
    Serial.print(numPulses);
    Serial.println(" pulses)");

    // Disable load_allowed if moving away from home position
    if (mmPosition < 0.1 && motorDirection) {
      load_allowed = false;
      Serial.println("Moving away from HOME position - LOAD command disabled");
    }
  }
  // Process load command
  else if (command == "LOAD") {
    // Check if load command is allowed (must be at home position)
    if (load_allowed) {
      // Calculate pulses needed to move to load position
      float distanceToMove = LOAD_POSITION_MM;
      numPulses = (unsigned long)(distanceToMove * PULSES_PER_MM);

      // Ensure we're moving forward
      motorDirection = true;
      digitalWrite(DIRECTION_PIN, LOW);  // Forward

      // Ensure motor is on
      digitalWrite(MOTOR_ON_PIN, HIGH);

      // Start moving
      startPulses();

      // Disable load_allowed when moving away from home
      load_allowed = false;

      Serial.print("Moving to LOAD position (");
      Serial.print(LOAD_POSITION_MM);
      Serial.println("mm)");
    } else {
      Serial.println("ERROR: LOAD command only allowed at HOME position (0.0mm)");
      Serial.print("Current position: ");
      Serial.print(mmPosition);
      Serial.println("mm");
      Serial.println("Please run HOME command first");
    }
  }
  // Process home command
  else if (command == "HOME") {
    startHoming();
  }
  // Process position command
  else if (command == "POS") {
    reportPosition();
  }
  // Process stop command
  else if (command == "STOP") {
    stopPulses();
    if (isHoming) {
      isHoming = false;
      homingStage = 0;
      Serial.println("Homing canceled");
    }
  }
  // Process status command
  else if (command == "STATUS") {
    reportStatus();
  }
  // Process zero position command
  else if (command == "ZERO") {
    pulsePosition = 0;
    mmPosition = 0.0;
    load_allowed = true;  // Enable LOAD command when manually zeroing
    Serial.println("Position set to 0.0mm");
    Serial.println("LOAD command is now allowed");
  }
  // Process kill switch test command
  else if (command == "TEST_KILL") {
    int reading = digitalRead(KILL_SWITCH_PIN);
    Serial.print("Kill switch raw reading: ");
    Serial.print(reading);
    Serial.println(reading == HIGH ? " (HIGH/PRESSED/ACTIVATED)" : " (LOW/NOT PRESSED/DEACTIVATED)");

    Serial.print("Debounced state: ");
    Serial.println(killSwitchActivated ? "ACTIVATED" : "DEACTIVATED");

    Serial.println("\nWith REVERSED logic:");
    Serial.println("- HIGH reading = switch is PRESSED (open circuit) = ACTIVATED");
    Serial.println("- LOW reading = switch is NOT PRESSED (closed circuit) = DEACTIVATED");
  } else {
    Serial.println("Unknown command");
  }
}

void reportPosition() {
  Serial.print("Current position: ");
  Serial.print(mmPosition);
  Serial.println("mm");

  // Detect if at special positions
  if (abs(mmPosition) < 0.1) {
    Serial.println("(HOME position)");
    if (load_allowed) {
      Serial.println("LOAD command is allowed");
    }
  } else if (abs(mmPosition - LOAD_POSITION_MM) < 0.1) {
    Serial.println("(LOAD position)");
  }
}

void reportStatus() {
  Serial.println("STATUS:");
  Serial.print("Motor: ");
  Serial.println(digitalRead(MOTOR_ON_PIN) == HIGH ? "ON" : "OFF");

  Serial.print("Direction: ");
  Serial.println(motorDirection ? "FORWARD" : "REVERSE");

  Serial.print("Frequency: ");
  Serial.print(pulseFrequency);
  Serial.println("Hz");

  Serial.print("Running: ");
  Serial.println(isRunning ? "YES" : "NO");

  Serial.print("Homing: ");
  if (isHoming) {
    Serial.print("IN PROGRESS (Stage ");
    Serial.print(homingStage);
    Serial.println(")");
  } else {
    Serial.println("NO");
  }

  // Get current kill switch state
  int reading = digitalRead(KILL_SWITCH_PIN);
  Serial.print("Kill Switch Raw Reading: ");
  Serial.println(reading == HIGH ? "HIGH (NOT PRESSED/ACTIVATED)" : "LOW (PRESSED/DEACTIVATED)");

  Serial.print("Kill Switch State: ");
  Serial.println(killSwitchActivated ? "ACTIVATED" : "DEACTIVATED");

  // Report position with special position detection
  Serial.print("Position: ");
  Serial.print(mmPosition);
  Serial.println("mm");

  if (abs(mmPosition) < 0.1) {
    Serial.println("(HOME position)");
    Serial.print("LOAD command: ");
    Serial.println(load_allowed ? "ALLOWED" : "NOT ALLOWED");
  } else if (abs(mmPosition - LOAD_POSITION_MM) < 0.1) {
    Serial.println("(LOAD position)");
  }

  Serial.print("Position in pulses: ");
  Serial.println(pulsePosition);

  Serial.print("Calibration: ");
  Serial.print(PULSES_PER_MM);
  Serial.println(" pulses per mm");

  Serial.print("Pulses generated: ");
  Serial.print(pulsesGenerated);
  Serial.print("/");
  Serial.println(numPulses);
}

void startHoming() {
  // Make sure motor is on
  digitalWrite(MOTOR_ON_PIN, HIGH);

  // Initialize homing
  isHoming = true;
  homingStage = 0;

  // Set direction to backward
  motorDirection = false;
  digitalWrite(DIRECTION_PIN, HIGH);  // Backward

  // Set large number of pulses for continuous movement until limit
  numPulses = 1000000;  // Very large number
  pulsesGenerated = 0;

  // Start moving
  isRunning = true;

  Serial.println("Homing started - Moving backward until kill switch is activated");
}

void updatePulseInterval() {
  // Calculate microseconds between pulses based on frequency
  if (pulseFrequency > 0) {
    pulseInterval = 1000000 / pulseFrequency;
  } else {
    pulseInterval = 1000;  // Default 1ms
  }
}

void startPulses() {
  if (isRunning) {
    stopPulses();
  }

  pulsesGenerated = 0;
  isRunning = true;
  Serial.print("Generating ");
  Serial.print(numPulses);
  Serial.print(" pulses at ");
  Serial.print(pulseFrequency);
  Serial.println("Hz");
}

void stopPulses() {
  isRunning = false;
  digitalWrite(JOG_PIN, LOW);
  Serial.println("Pulse generation stopped");

  // Check if we've reached the load position
  if (abs(mmPosition - LOAD_POSITION_MM) < 0.1) {
    Serial.println("LOAD POSITION REACHED");
  }

  // Check if we've returned to home position
  if (abs(mmPosition) < 0.1 && !load_allowed) {
    load_allowed = true;
    Serial.println("HOME position reached - LOAD command is now allowed");
  }
}

void generatePulses() {
  // Check if we've generated all requested pulses
  if (pulsesGenerated >= numPulses) {
    stopPulses();
    return;
  }

  // Generate pulse at the specified frequency
  unsigned long currentMicros = micros();

  if (currentMicros - lastPulseTime >= pulseInterval) {
    lastPulseTime = currentMicros;

    // Toggle JOG pin (pulse)
    digitalWrite(JOG_PIN, HIGH);
    delayMicroseconds(5);  // Short pulse (5µs)
    digitalWrite(JOG_PIN, LOW);

    // Increment pulse counter
    pulsesGenerated++;

    // Update position based on direction
    if (motorDirection) {
      pulsePosition++;  // Forward movement increases position
    } else {
      pulsePosition--;  // Backward movement decreases position
    }

    // Update millimeter position
    mmPosition = pulsePosition / PULSES_PER_MM;

    // Disable load_allowed if moving away from home position
    if (abs(mmPosition) < 0.1 && motorDirection && load_allowed) {
      load_allowed = false;
      Serial.println("Moving away from HOME position - LOAD command disabled");
    }

    // Re-enable load_allowed if returning to home position
    if (abs(mmPosition) < 0.1 && !motorDirection && !load_allowed) {
      load_allowed = true;
      Serial.println("HOME position reached - LOAD command is now allowed");
    }

    // Print status every 100 pulses
    if (pulsesGenerated % 100 == 0) {
      Serial.print("Pulses: ");
      Serial.print(pulsesGenerated);
      Serial.print("/");
      Serial.println(numPulses);
    }
  }
}