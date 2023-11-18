#include <AccelStepper.h>
#include <MultiStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (STEP PIN=2, DIR PIN=5)
long currentTargetPosition = 0;
bool hasTarget = false;

void setup() {
  // Set maximum speed value for the stepper
  // 200 Steps/revolution by default
  hasTarget = false;
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  Serial.begin(9600);
}

void loop() {
  // read in a new position if one is sent and we aren't currently moving to a target
  if (Serial.available() > 0 and !hasTarget) { 
    long target = Serial.readString().toInt();
    currentTargetPosition += target;
    hasTarget = true;
  }
  else if (hasTarget) {
    stepper1.moveTo(currentTargetPosition);
    stepper1.setSpeed(1000);
    stepper1.runSpeedToPosition();

    // once we are at the target position, stop moving
    if (stepper1.currentPosition() == currentTargetPosition) {
      hasTarget = false;
      Serial.println("DONE");
    }
  }
}