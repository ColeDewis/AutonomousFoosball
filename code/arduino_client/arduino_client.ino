#include <AccelStepper.h>
#include <MultiStepper.h>

#define LEFT_STEPPER_ID 1
#define RIGHT_STEPPER_ID 2
#define RESET_ID 0
#define MIN_ALLOWED_STEPS 0
#define MAX_ALLOWED_STEPS 2100

// Define the stepper motor and the pins that is connected to
AccelStepper leftStepper(1, 3, 4);   // (STEP PIN=3, DIR PIN=4)
AccelStepper rightStepper(1, 2, 5);  // (STEP PIN=2, DIR PIN=5)
int leftCurrentTargetPosition = 0;
int rightCurrentTargetPosition = 0;
bool leftHasTarget = false;
bool rightHasTarget = false;
bool reset = false;

void setup() {
  // Set maximum speed value for the stepper
  // 200 Steps/revolution by default
  leftHasTarget = false;
  rightHasTarget = false;
  leftStepper.setCurrentPosition(0);
  leftStepper.setMaxSpeed(1000);
  rightStepper.setCurrentPosition(0);
  rightStepper.setMaxSpeed(1000);
  Serial.begin(115200);
}

void loop() {
  if (reset) 
  {
    // reset all steppers to 0. we will NEVER exit this once we get here, so this serves as termination
    leftStepper.moveTo(0);
    leftStepper.setSpeed(1000);
    leftStepper.runSpeedToPosition();
    rightStepper.moveTo(0);
    rightStepper.setSpeed(1000);
    rightStepper.runSpeedToPosition();
  } 
  else if (Serial.available() > 0) 
  {
    // read in a new position if one is sent and we aren't currently moving to a target
    // read one byte
    int motorId = Serial.read(); 
    if (motorId == RESET_ID) 
    {
      reset = true;
    } 
    else 
    {
      char buf[3];
      Serial.readBytes(buf, 3);
      int sign = buf[0];

      // reconstruct the step target
      uint8_t a = buf[1];
      uint8_t b = buf[2];
      uint16_t u_target = (a << 8) | b;

      // get proper sign and validate target
      int target = sign > 0 ? u_target * -1 : u_target;
      if (target < MIN_ALLOWED_STEPS) target = MIN_ALLOWED_STEPS;
      if (target > MAX_ALLOWED_STEPS) target = MAX_ALLOWED_STEPS;

      // send target to appropriate motor
      if (motorId == LEFT_STEPPER_ID) 
      {
        leftCurrentTargetPosition = target;
        leftHasTarget = true;
      } 
      else if (motorId == RIGHT_STEPPER_ID) 
      {
        rightCurrentTargetPosition = target;
        rightHasTarget = true;
      }
    }
  } 
  else if (leftHasTarget) 
  {
    leftStepper.moveTo(leftCurrentTargetPosition);
    leftStepper.setSpeed(1000);
    leftStepper.runSpeedToPosition();

    // once we are at the target position, stop moving
    if (leftStepper.currentPosition() == leftCurrentTargetPosition) {
      leftHasTarget = false;
      Serial.write(LEFT_STEPPER_ID);
    }
  } 
  else if (rightHasTarget) 
  {
    rightStepper.moveTo(rightCurrentTargetPosition);
    rightStepper.setSpeed(1000);
    rightStepper.runSpeedToPosition();

    // once we are at the target position, stop moving
    if (rightStepper.currentPosition() == rightCurrentTargetPosition) {
      rightHasTarget = false;
      Serial.write(RIGHT_STEPPER_ID);
    }
  }
}