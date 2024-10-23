#include <Romi32U4.h>
#include <servo32u4.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include <BlueMotor.h>

Servo32U4 servo;
Romi32U4ButtonB buttonB;
Rangefinder rangefinder(17, 12);
BlueMotor motor;
Chassis chassis;
IRDecoder decoder(14);

int targetCount2 = 0;
int distance = 100;

/*
if ((leftRead > 700) && (rightRead > 700)) {
    if(turn){
      chassis.setMotorEfforts(200, 30);
      delay(780);
      turn = false;   
    }
  } 
*/
bool turning = false;
int16_t currentLeftRotation = 0;
int16_t currentRightRotation = 0;

int16_t initialLeftRotation = 0;
int16_t initialRightRotation = 0;
int16_t eStopLeftState = 0;
int16_t eStopRightState = 0;

float degreesSpun = 0;

int linearPotPin = 18;
int servoStop = 1470;  
int servoJawDown = 1350;  
int servoJawUp = 1700;  
int printDelay = 500;
int linearPotVoltageADC = 510;
int jawOpenPotVoltageADC = 430;
int jawClosedPotVoltageADC = 500;
int prevLinearPotVoltageADC = 0;
long startingTheta = 265;

int speed = -100;
int LineTrackerPin1 = 22;
int LineTrackerPin3 = 20;

bool driveToHousePickupPlate = false;
bool pickupPlate45 = false;
bool pickupPlate25 = false;
bool driveToBox = false;
bool changePlate = false;
bool driveToHousePlacePlate = false;
bool placePlate = false;
bool driveAcross = false;
bool nudgeUp = false;
bool nudgeDown = false;

bool eStopHappened = false;
bool emergencyStop = false;
bool turn = true;

bool start = false;
int sequence = 0;

void lineFollower(bool direction) {
  int leftRead = analogRead(LineTrackerPin3);
  int rightRead = analogRead(LineTrackerPin1);

  int leftMotorSpeed = -100;
  int rightMotorSpeed = -110;
  if ((leftRead > 500) && (rightRead < 400)) {
    rightMotorSpeed += 20;
  } else if ((leftRead < 400) && (rightRead > 500)) {
    leftMotorSpeed += 20;
  }
  if (!(direction)) {
    leftMotorSpeed = leftMotorSpeed * -1;
    rightMotorSpeed = rightMotorSpeed * -1;
  }
  chassis.setMotorEfforts(leftMotorSpeed, rightMotorSpeed);
}

bool hitIntersection() {
  int leftRead2 = analogRead(LineTrackerPin1);
  int rightRead2 = analogRead(LineTrackerPin3);
  if ((leftRead2 > 600) && (rightRead2 > 600)) {
    Serial.println("HIT");
    return true;
  } else {
    Serial.println("NOT HIT");
    return false;
  }
}

bool ultrasonicSensor(float distanceToGo) {
  float distanceReading = rangefinder.getDistance();
  Serial.println(distanceReading);
  if (distanceReading < distanceToGo) {
    return false;
  }
  return true;
}

// bottom out
void gripperRelease() {
  servo.writeMicroseconds(950);
}

// bottom out
void gripperGrip() {
  long read = 0;
  long previousRead = 0;
  int count = 0;
  // 950: openState, 1700: closedState
  for (unsigned t = 950; t < 1700; t += 25)
  {
    read = analogRead(A0);
    servo.writeMicroseconds(t);
    if (previousRead == read) {
      count++;
    }
    // stops gripper closing
    // makes sure gripper got stuck
    if (count > 4) {
      gripperRelease();
      break;
    }
    previousRead = read;
    delay(20);
  } 
}

// linear
void moveGripperUp()
{
  // makes servo start spinning counter-clockwise
  servo.writeMicroseconds(servoJawUp);
  // while loop prevents stopping servo until 
  // linear potentiometer reaches the correct height
  while (linearPotVoltageADC < jawClosedPotVoltageADC)
  {
    linearPotVoltageADC = analogRead(linearPotPin);
  }
  // stops servo
  servo.writeMicroseconds(servoStop);
}

// linear
void moveGripperDown() 
{
  servo.writeMicroseconds(servoJawDown);
  int counter = 0;
  while (linearPotVoltageADC > jawOpenPotVoltageADC)
  {
    linearPotVoltageADC = analogRead(linearPotPin);
    // check if linear potentiometer doesnt move enough
    if ((prevLinearPotVoltageADC - linearPotVoltageADC) < 1) {
      counter++;
      // make sure linear potentiometer did get stuck on something
      if (counter == 20) { 
        // stops GripperDown and moves gripper up
        moveGripperUp();
        break;
      }
    }
    prevLinearPotVoltageADC = linearPotVoltageADC;
    delay(500);
  }
  servo.writeMicroseconds(servoStop);
}

// Question 5 - Fourbar Code
bool moveFourbarUp() {
  if(abs(distance) > 1) {
    targetCount2 = ((275 - 265)/360.0*540) * 25;
    distance = targetCount2 + motor.getPosition();
    motor.setEffortWithoutDB(distance);
    return true;
  } else {
    motor.setEffort(0); 
    startingTheta = startingTheta + 10; 
    return false;
  }
}

// Question 5 - Fourbar Code
bool moveFourbarDown() {
  if(abs(distance) > 1) {
    targetCount2 = ((255 - 265)/360.0*540) * 25;
    distance = targetCount2 + motor.getPosition();
    motor.setEffortWithoutDB(distance);
    return true;
  }
  else {
    motor.setEffort(0); 
    startingTheta = startingTheta - 10; 
    return false;
  }
}

// Question 5 - Fourbar Code
bool moveFourbarTo(int gripperAngle) {
  long targetTheta = 0;
  long targetCount = 0;

  // move to gripper down
  if (gripperAngle == 0) {
    targetTheta = 265;
    targetCount = ((targetTheta - startingTheta)/360.0*540) * 25;
    if (abs(distance) > 1) {
      distance = targetCount + motor.getPosition();
      motor.setEffortWithoutDB(distance);
      return true;
    } else {
      motor.setEffort(0);
      startingTheta = targetTheta;
      return false;
    }
  }

  // move to gripper 25 degree plate
  if (gripperAngle == 25) {
    targetTheta = 115;
    targetCount = ((targetTheta - startingTheta)/360.0*540) * 25;
    while (abs(distance) > 1) {
      distance = targetCount + motor.getPosition();
      motor.setEffortWithoutDB(distance);
    }
    motor.setEffort(0);
    startingTheta = targetTheta;
  }

  // move to gripper 45 degree plate
  if (gripperAngle == 45) {
    targetTheta = 375;
    targetCount = ((targetTheta - startingTheta)/360.0*540) * 25;
    if (abs(distance) > 1) {
      distance = targetCount + motor.getPosition();
      motor.setEffortWithoutDB(distance);
      return true;
    } else {
    motor.setEffort(0);
    startingTheta = targetTheta;
    return false;
    }
  }
  return false;
}

//remote
void checkRemote(){
  //noInterrupts();
  int16_t code = decoder.getKeyCode();
  switch(code) {
    case remote1:
      if (!emergencyStop) {
        driveToHousePickupPlate = true;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = false;
        driveToHousePlacePlate = false;
        placePlate = false;
        driveAcross = false;
      }
      break;
    case remote2:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = true;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = false;
        driveToHousePlacePlate = false;
        placePlate = false;
        driveAcross = false;
      }
      break;
    case remote3:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = true;
        changePlate = false;
        driveToHousePlacePlate = false;
        placePlate = false;
        driveAcross = false;
      }
      break;
    case remote4:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = true;
        driveToHousePlacePlate = false;
        placePlate = false;
        driveAcross = false;
      }
      break;
    case remote5:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = false;
        driveToHousePlacePlate = true;
        placePlate = false;
        driveAcross = false;
      }
      break;
    case remote6:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = false;
        driveToHousePlacePlate = false;
        placePlate = true;
        driveAcross = false;
      }
      break;
    case remote7:
      if (!emergencyStop) {
        driveToHousePickupPlate = false;
        pickupPlate45 = false;
        pickupPlate25 = false;
        driveToBox = false;
        changePlate = false;
        driveToHousePlacePlate = false;
        placePlate = false;
        driveAcross = true;
      }
      break;
    case remoteEnterSave:
      emergencyStop = !(emergencyStop);
      break;
    case remoteUp:
      nudgeUp = true;
      break;
    case remoteDown:
      nudgeDown = true;
  }
  //delay(2000);
  //interrupts();
}

void setup()
{
  Serial.begin(9600);
  servo.setMinMaxMicroseconds(500, 2500);
  chassis.init();
  rangefinder.init();
  decoder.init();
  motor.setup();
  motor.reset();
  delay(1000);
  Serial.println("Started");
  pinMode(LineTrackerPin1, INPUT);
  pinMode(LineTrackerPin3, INPUT);
}

void loop()
{
  checkRemote();
  motor.setEffort(0);
  servo.writeMicroseconds(servoStop);
  if (driveToHousePickupPlate && !emergencyStop) {
    Serial.println("drive to house pickup plate");
    if (sequence == 0) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 1) {
      if (ultrasonicSensor(13.05)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
        Serial.println("sequence done");
      }
    } 
    if (sequence == 2) {
      if (!(moveFourbarTo(45))) {
        motor.reset();
        distance = 100;
        sequence++;
        Serial.println("sequence 2 done");
      } 
    }
    if (sequence == 3) {
      moveGripperDown();
      sequence++;
    }
    if (sequence == 4) {
      sequence = 0;
      driveToHousePickupPlate = false;
      Serial.println("all done");
    }
  }
  if (pickupPlate45 && !emergencyStop) {
    Serial.println("pickup plate 45");
    if (sequence == 0) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 1) {
      chassis.setMotorEfforts(100, 110);
      delay(400);
      chassis.idle();
      sequence++;
    }
    if (sequence == 2) {
      if (!(turning)) {
        chassis.turnFor(170 - degreesSpun, 60, false);
        Serial.println(170 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 3) {
      if (!(hitIntersection())) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 4) {
      chassis.setMotorEfforts(-110, -100);
      delay(500);
      chassis.idle();
      sequence++;
    }
    if (sequence == 5) {
      if (!(turning)) {
        chassis.turnFor(-80 + degreesSpun, 60, false);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentLeftRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 6) {
      if (ultrasonicSensor(5)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 7) {
      if (!(moveFourbarTo(0))) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 8) {
      if (!(moveFourbarDown())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 9) {
      moveGripperUp();
      sequence = 0;
      pickupPlate45 = false;
    }
  }
  if (changePlate && !emergencyStop) {
    Serial.println("change plate");
    if (sequence == 0) {
      moveGripperDown();
      sequence++;
    }
    if (sequence == 1) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 2) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 3) {
      if (!(moveFourbarTo(45))) {
        motor.reset();
        distance = 100;
        sequence++;
      } 
    }
    if (sequence == 4) {
      if (!(turning)) {
        chassis.turnFor(170 - degreesSpun, 60, false);
        Serial.println(170 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 5) {
      if (!(hitIntersection())) {
        lineFollower(true);
      } else {
        chassis.idle();
        chassis.setMotorEfforts(-110, -100);
        delay(550);
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 6) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 7) {
      if (!(moveFourbarUp())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 8) {
      if (!(turning)) {
        chassis.turnFor(80 - degreesSpun, 60, false);
        Serial.println(80 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 9) {
      chassis.setMotorEfforts(-100, -110);
      delay(100);
      if (ultrasonicSensor(12.35)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 10) {
      chassis.setMotorEfforts(-100, -110);
      delay(100);
      if (ultrasonicSensor(12.35)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 11) {
      if (ultrasonicSensor(12.35)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 12) {
      if (ultrasonicSensor(12.35)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 13) {
      moveGripperUp();
      changePlate = false;
      sequence = 0;
    }
  }
  if (driveToHousePlacePlate && !emergencyStop) {
    Serial.println("drive to house place plate");
    if (sequence == 0) {
      if (!(moveFourbarDown())) {
        motor.reset();
        distance = 100;
        chassis.setMotorEfforts(110, 100);
        delay(400);
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 1) {
      if (!(turning)) {
        chassis.turnFor(170 - degreesSpun, 60, false);
        Serial.println(170 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 2) {
      if (!(hitIntersection())) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 3) {
      chassis.setMotorEfforts(-110, -100);
      delay(500);
      chassis.idle();
      sequence++;
    }
    if (sequence == 4) {
      if (!(turning)) {
        chassis.turnFor(-80 + degreesSpun, 60, false);
        Serial.println(-80 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 5) {
      if (ultrasonicSensor(4.25)) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 6) {
      if (!(moveFourbarTo(0))) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 7) {
      if (!(moveFourbarDown())) {
        motor.reset();
        distance = 100;
        sequence++;
      }
    }
    if (sequence == 8) {
      if (!(turning)) {
        chassis.turnFor(-80 + degreesSpun, 60, false);
        Serial.println(-80 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 9) {
      if (!(hitIntersection())) {
        chassis.setMotorEfforts(-100, -105);
      } else {
        chassis.idle();
        sequence++;
      }
    }
    if (sequence == 10) {
      chassis.setMotorEfforts(-100, -110);
      delay(350);
      chassis.idle();
      sequence++;
    }
    if (sequence == 11) {
      if (!(turning)) {
        chassis.turnFor(-80 + degreesSpun, 60, false);
        Serial.println(-80 - degreesSpun);
        initialLeftRotation = chassis.getLeftEncoder();
        initialRightRotation = chassis.getRightEncoder();
        if (eStopHappened) {
          initialLeftRotation = eStopLeftState;
          initialRightRotation = eStopRightState;
        }
        delay(1);
        turning = true;
      } else {
        if (chassis.checkMotionComplete()) {
          sequence++;
          degreesSpun = 0;
          turning = false;
          eStopHappened = false;
        } else {
          currentLeftRotation = chassis.getLeftEncoder() - initialLeftRotation;
          currentRightRotation = chassis.getRightEncoder() - initialRightRotation;
          degreesSpun = currentRightRotation / 1440.0 * 7.35 / 3.5 * 90;
        }
      }
    }
    if (sequence == 12) {
      if (!(hitIntersection())) {
        lineFollower(true);
      } else {
        chassis.idle();
        sequence = 0;
        driveToHousePlacePlate = false;
      }
    }
  }
  if (emergencyStop) {
    eStopHappened = true;
    chassis.idle();
    servo.writeMicroseconds(servoStop);
    motor.setEffort(0);
    eStopLeftState = initialLeftRotation;
    eStopRightState = initialRightRotation;
    turning = false;
  }

  if (nudgeUp) {
    if (!(moveFourbarUp())) {
      motor.reset();
      distance = 100;
      nudgeUp = false;
    }
  }

  if (nudgeDown) {
    if (!(moveFourbarDown())) {
      motor.reset();
      distance = 100;
      nudgeDown = false;
    }
  }
  delay(10);
}
