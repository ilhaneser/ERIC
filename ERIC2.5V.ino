/*
 * Stepper Motor Control with Position Tracking and eGun Control
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
 * EGUN_ON - Turn eGun ON (5V on pin 11)
 * EGUN_OFF - Turn eGun OFF (0V on pin 11)
 * SEQ_STEPS:n - Set number of steps in sequence
 * SEQ_POS:i:p - Set position for step i to p mm
 * SEQ_PRE:i:t - Set pre-delay for step i to t seconds
 * SEQ_EGUN:i:t - Set eGun time for step i to t seconds
 * SEQ_POST:i:t - Set post-delay for step i to t seconds
 * RUN_SEQUENCE - Start automated sampling sequence
 * CANCEL_SEQUENCE - Cancel the running sequence
 *
 * Note: The LOAD command can only be executed when position is exactly at 0.0mm.
 * If position is greater than 0, attempting to use LOAD will display an error message.
 */

// Pin definitions
const int MOTOR_ON_PIN = 10;     // Motor enable pin
const int DIRECTION_PIN = 5;     // Direction control pin
const int JOG_PIN = 2;           // Jog pulse pin
const int KILL_SWITCH_PIN = 13;  // Kill switch input pin (on pin 13)
const int EGUN_PIN = 11;         // eGun control pin (5V signal) - Pin 11

// Movement parameters
unsigned long pulseFrequency = 1000;  // Default frequency (Hz)
unsigned long numPulses = 1000;       // Default number of pulses
bool isRunning = false;               // Whether the motor is running
bool motorDirection = true;           // true = forward (LOW), false = backward (HIGH)
long pulsePosition = 0;               // Current position in pulses
float mmPosition = 0.0;               // Current position in millimeters
bool load_allowed = false;            // Flag to allow LOAD command only at HOME position
bool at_load_position = false;        // Flag to indicate if at LOAD position
const float PULSES_PER_MM = 120.0;    // Calibration: 1200 pulses = 10mm, so 120 pulses = 1mm
const float LOAD_POSITION_MM = 240.0; // Load position in millimeters

// Kill switch state
bool killSwitchActivated = false;     // Whether the kill switch is currently activated
unsigned long debounceDelay = 50;     // Debounce time in milliseconds
unsigned long lastDebounceTime = 0;   // Last time the kill switch changed state
int lastSwitchReading = HIGH;         // Last raw reading from kill switch

// Homing parameters
bool isHoming = false;                // Whether we're in homing mode
int homingStage = 0;                  // 0=moving back, 1=moving forward

// Timing variables
unsigned long lastPulseTime = 0;
unsigned long pulsesGenerated = 0;
unsigned long pulseInterval;          // Microseconds between pulses

// eGun state
bool egunActive = false;              // Whether eGun is currently active

// Sequence variables
bool sequenceRunning = false;         // Whether a sequence is currently running
int currentSequenceStep = 0;          // Current step in the sequence
int totalSequenceSteps = 0;           // Total number of steps in the sequence
float sequencePositions[10];          // Array to store positions (max 10)
unsigned long preDelays[10];          // Pre-eGun delays in milliseconds
unsigned long egunTimes[10];          // eGun activation times in milliseconds
unsigned long postDelays[10];         // Post-eGun delays in milliseconds
unsigned long sequenceStartTime = 0;  // When the current sequence step started
int sequenceStage = 0;                // 0=moving, 1=pre-delay, 2=eGun-on, 3=post-delay

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Stepper Motor Control with Position in Millimeters and eGun Control");
 
  // Set up pins
  pinMode(MOTOR_ON_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(JOG_PIN, OUTPUT);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  pinMode(EGUN_PIN, OUTPUT);
 
  // Initial states
  digitalWrite(MOTOR_ON_PIN, LOW);   // Motor initially off
  digitalWrite(DIRECTION_PIN, LOW);  // Forward direction
  digitalWrite(JOG_PIN, LOW);        // Jog pin initially low
  digitalWrite(EGUN_PIN, LOW);       // eGun initially off
 
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
  Serial.println("EGUN_ON - Turn eGun ON (5V on pin 11)");
  Serial.println("EGUN_OFF - Turn eGun OFF (0V on pin 11)");
  Serial.println("SEQ_STEPS:n - Set number of steps in sequence");
  Serial.println("SEQ_POS:i:p - Set position for step i to p mm");
  Serial.println("RUN_SEQUENCE - Start automated sampling sequence");
  
  Serial.println("\nKILL SWITCH WIRING (with REVERSED logic):");
  Serial.println("- Connect one wire to GND (ground)");
  Serial.println("- Connect other wire to pin 13");
  Serial.println("- HIGH = ACTIVATED (switch not pressed/open circuit)");
  Serial.println("- LOW = DEACTIVATED (switch pressed/closed circuit)");
  
  Serial.println("\neGUN CONTROL:");
  Serial.println("- eGun control signal on pin 11");
  Serial.println("- HIGH (5V) = eGun ON");
  Serial.println("- LOW (0V) = eGun OFF");
  
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
  
  // Handle sequence process if active
  if (sequenceRunning) {
    processSequence();
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
      at_load_position = false; // Not at LOAD position
      Serial.println("Homing complete - Position set to 0.0mm");
      Serial.println("LOAD command is now allowed");
    }
  }
}

void processSequence() {
  // Only process if we have steps defined
  if (totalSequenceSteps <= 0) {
    sequenceRunning = false;
    return;
  }
  
  // If the motor is moving, wait for it to stop
  if (isRunning) {
    return;
  }
  
  // Process current step based on stage
  switch (sequenceStage) {
    case 0:  // Moving to position
      // Move to the next position in the sequence
      if (currentSequenceStep < totalSequenceSteps) {
        // Get the target position - this is absolute distance from LOAD position
        float targetPosition = sequencePositions[currentSequenceStep];
        
        // IMPORTANT: In sequence mode, we ALWAYS move BACKWARD from LOAD position
        // First, force the direction to BACKWARD
        motorDirection = false;
        digitalWrite(DIRECTION_PIN, HIGH);  // Backward (HIGH)
        
        // Convert the absolute target position to the amount we need to move from current position
        // The sign doesn't matter since we're always moving backward
        float distanceToMove = abs(targetPosition - abs(mmPosition));
        
        // Calculate pulses
        numPulses = (unsigned long)(distanceToMove * PULSES_PER_MM);
        
        // Start moving
        if (numPulses > 0) {
          Serial.print("Sequence step ");
          Serial.print(currentSequenceStep + 1);
          Serial.print("/");
          Serial.print(totalSequenceSteps);
          Serial.print(": Moving to position ");
          Serial.print(targetPosition);
          Serial.print("mm BACKWARD from LOAD (");
          Serial.print(distanceToMove);
          Serial.println("mm from current position)");
          
          startPulses();
        } else {
          // Already at position, proceed to next stage
          sequenceStage = 1;
          sequenceStartTime = millis();
          Serial.print("Already at position ");
          Serial.print(targetPosition);
          Serial.println("mm from LOAD, starting pre-delay");
        }
        
        // Move to pre-delay stage
        sequenceStage = 1;
      } else {
        // Sequence completed
        sequenceRunning = false;
        Serial.println("Sequence completed");
      }
      break;
      
    case 1:  // Pre-delay (waiting before turning on eGun)
      {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - sequenceStartTime;
        
        if (elapsedTime >= preDelays[currentSequenceStep]) {
          // Pre-delay completed, turn on eGun
          egunOn();
          
          // Start eGun timing
          sequenceStartTime = currentTime;
          sequenceStage = 2;
          
          Serial.print("Pre-delay completed. eGun ON for ");
          Serial.print(egunTimes[currentSequenceStep] / 1000.0);
          Serial.println(" seconds");
        }
      }
      break;
      
    case 2:  // eGun ON period
      {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - sequenceStartTime;
        
        if (elapsedTime >= egunTimes[currentSequenceStep]) {
          // eGun time completed, turn off eGun
          egunOff();
          
          // Start post-delay timing
          sequenceStartTime = currentTime;
          sequenceStage = 3;
          
          Serial.print("eGun time completed. eGun OFF. Post-delay for ");
          Serial.print(postDelays[currentSequenceStep] / 1000.0);
          Serial.println(" seconds");
        }
      }
      break;
      
    case 3:  // Post-delay (waiting after turning off eGun)
      {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - sequenceStartTime;
        
        if (elapsedTime >= postDelays[currentSequenceStep]) {
          // Post-delay completed, move to next step
          currentSequenceStep++;
          sequenceStage = 0;  // Back to movement stage
          
          if (currentSequenceStep < totalSequenceSteps) {
            Serial.print("Post-delay completed. Moving to next position (step ");
            Serial.print(currentSequenceStep + 1);
            Serial.print("/");
            Serial.print(totalSequenceSteps);
            Serial.println(")");
          } else {
            Serial.println("Sequence completed");
            sequenceRunning = false;
          }
        }
      }
      break;
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
  }
  else if (command == "OFF") {
    digitalWrite(MOTOR_ON_PIN, LOW);
    Serial.println("Motor OFF");
  }
  // Process direction commands
  else if (command == "FWD") {  // Forward is LOW to Direction Pin
    motorDirection = true;
    digitalWrite(DIRECTION_PIN, LOW); // Forward
    Serial.println("Direction: FORWARD");
  }
  else if (command == "REV") {   // Reverse is HIGH to Direction Pin
    motorDirection = false;
    digitalWrite(DIRECTION_PIN, HIGH); // Reverse
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
    
    // Reset LOAD position flag when moving
    at_load_position = false;
    
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
      
      // Set LOAD target flag
      at_load_position = true;
      
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
    if (sequenceRunning) {
      sequenceRunning = false;
      egunOff();
      Serial.println("Sequence canceled");
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
    Serial.println(reading == HIGH ? " (HIGH/NOT PRESSED/ACTIVATED)" : " (LOW/PRESSED/DEACTIVATED)");
    
    Serial.print("Debounced state: ");
    Serial.println(killSwitchActivated ? "ACTIVATED" : "DEACTIVATED");
    
    Serial.println("\nWith REVERSED logic:");
    Serial.println("- HIGH reading = switch is NOT PRESSED (open circuit) = ACTIVATED");
    Serial.println("- LOW reading = switch is PRESSED (closed circuit) = DEACTIVATED");
  }
  // Process eGun commands
  else if (command == "EGUN_ON") {
    egunOn();
  }
  else if (command == "EGUN_OFF") {
    egunOff();
  }
  // Process sequence commands
  else if (command.startsWith("SEQ_STEPS:")) {
    // Set number of steps in sequence
    totalSequenceSteps = command.substring(10).toInt();
    
    if (totalSequenceSteps > 10) {
      totalSequenceSteps = 10;  // Limit to max 10 steps
      Serial.println("WARNING: Limited to maximum 10 sequence steps");
    }
    
    if (totalSequenceSteps > 0) {
      currentSequenceStep = 0;
      Serial.print("Sequence set to ");
      Serial.print(totalSequenceSteps);
      Serial.println(" steps");
    } else {
      Serial.println("Invalid number of sequence steps");
    }
  }
  else if (command.startsWith("SEQ_POS:")) {
    // Parse position index:value
    int colonPos = command.indexOf(':', 8);
    if (colonPos > 0) {
      int idx = command.substring(8, colonPos).toInt();
      float pos = command.substring(colonPos + 1).toFloat();
      
      if (idx >= 0 && idx < 10) {
        sequencePositions[idx] = pos;
        Serial.print("Sequence position ");
        Serial.print(idx);
        Serial.print(" set to ");
        Serial.print(pos);
        Serial.println("mm");
      } else {
        Serial.println("Invalid position index (must be 0-9)");
      }
    }
  }
  else if (command.startsWith("SEQ_PRE:")) {
    // Pre-delay times in seconds for sequence steps
    int colonPos = command.indexOf(':', 8);
    if (colonPos > 0) {
      int idx = command.substring(8, colonPos).toInt();
      float seconds = command.substring(colonPos + 1).toFloat();
      
      if (idx >= 0 && idx < 10) {
        preDelays[idx] = (unsigned long)(seconds * 1000.0);  // Convert to milliseconds
        Serial.print("Sequence pre-delay ");
        Serial.print(idx);
        Serial.print(" set to ");
        Serial.print(seconds);
        Serial.println(" seconds");
      } else {
        Serial.println("Invalid index (must be 0-9)");
      }
    }
  }
  else if (command.startsWith("SEQ_EGUN:")) {
    // eGun on times in seconds for sequence steps
    int colonPos = command.indexOf(':', 9);
    if (colonPos > 0) {
      int idx = command.substring(9, colonPos).toInt();
      float seconds = command.substring(colonPos + 1).toFloat();
      
      if (idx >= 0 && idx < 10) {
        egunTimes[idx] = (unsigned long)(seconds * 1000.0);  // Convert to milliseconds
        Serial.print("Sequence eGun time ");
        Serial.print(idx);
        Serial.print(" set to ");
        Serial.print(seconds);
        Serial.println(" seconds");
      } else {
        Serial.println("Invalid index (must be 0-9)");
      }
    }
  }
  else if (command.startsWith("SEQ_POST:")) {
    // Post-delay times in seconds for sequence steps
    int colonPos = command.indexOf(':', 9);
    if (colonPos > 0) {
      int idx = command.substring(9, colonPos).toInt();
      float seconds = command.substring(colonPos + 1).toFloat();
      
      if (idx >= 0 && idx < 10) {
        postDelays[idx] = (unsigned long)(seconds * 1000.0);  // Convert to milliseconds
        Serial.print("Sequence post-delay ");
        Serial.print(idx);
        Serial.print(" set to ");
        Serial.print(seconds);
        Serial.println(" seconds");
      } else {
        Serial.println("Invalid index (must be 0-9)");
      }
    }
  }
  else if (command == "RUN_SEQUENCE") {
    // Start the sequence execution
    if (totalSequenceSteps > 0) {
      currentSequenceStep = 0;
      sequenceStage = 0;
      sequenceRunning = true;
      Serial.println("Starting sequence execution");
    } else {
      Serial.println("No sequence defined. Use SEQ_STEPS:n to set number of steps");
    }
  }
  else if (command == "CANCEL_SEQUENCE") {
    // Cancel the current sequence
    if (sequenceRunning) {
      sequenceRunning = false;
      egunOff();
      Serial.println("Sequence canceled");
    } else {
      Serial.println("No sequence running");
    }
  }
  else {
    Serial.println("Unknown command: " + command);
  }
}

void egunOn() {
  digitalWrite(EGUN_PIN, HIGH);
  egunActive = true;
  Serial.println("eGun ON (5V signal on pin 11)");
}

void egunOff() {
  digitalWrite(EGUN_PIN, LOW);
  egunActive = false;
  Serial.println("eGun OFF (0V signal on pin 11)");
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
  }
  else if (abs(mmPosition - LOAD_POSITION_MM) < 0.1 || at_load_position) {
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
  }
  else if (abs(mmPosition - LOAD_POSITION_MM) < 0.1 || at_load_position) {
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
  
  // eGun status
  Serial.print("eGun: ");
  Serial.println(egunActive ? "ON (pin 11 HIGH)" : "OFF (pin 11 LOW)");
  
  // Sequence status
  Serial.print("Sequence: ");
  if (sequenceRunning) {
    Serial.print("RUNNING (Step ");
    Serial.print(currentSequenceStep + 1);
    Serial.print("/");
    Serial.print(totalSequenceSteps);
    Serial.print(", Stage ");
    Serial.print(sequenceStage);
    Serial.println(")");
  } else {
    Serial.println("NOT RUNNING");
    if (totalSequenceSteps > 0) {
      Serial.print("Sequence defined with ");
      Serial.print(totalSequenceSteps);
      Serial.println(" steps");
    }
  }
}

void startHoming() {
  // Make sure motor is on
  digitalWrite(MOTOR_ON_PIN, HIGH);
  
  // Initialize homing
  isHoming = true;
  homingStage = 0;
  
  // Reset load position flag
  at_load_position = false;
  
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
    pulseInterval = 1000; // Default 1ms
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
  if (at_load_position && abs(mmPosition - LOAD_POSITION_MM) < 5.0) {
    Serial.println("LOAD POSITION REACHED");
    
    // Auto-zero at load position for sample positioning
    pulsePosition = 0;
    mmPosition = 0.0;
    Serial.println("Position automatically zeroed at LOAD position");
    Serial.println("Ready for sample positioning");
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
      pulsePosition++; // Forward movement increases position
    } else {
      pulsePosition--; // Backward movement decreases position
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