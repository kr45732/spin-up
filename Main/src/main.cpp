/*---------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                 */
/*    Author:       Krish Ranjan                                             */
/*    Created:      Thu Oct 26 2020                                          */
/*    Description:  Code for VEX Spin Up 2022 - 2023                         */
/*---------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Global variables
const float flywheelUpperPercent = 0.97;
const float flywheelLowerPercent = 0.78;
float flywheelPercent = flywheelLowerPercent;

bool indexerCycling = false;
int shootCount = 0;
bool shouldAsyncShoot = false;
bool isTripleShot = false;

bool pneumaticsYPressed = false;
bool pneumaticsRightPressed = false;

// Function prototypes
void chassis(double forwardScale = 1.0, double turnScale = 1.0,
             double strafeScale = 1.0, int deadZone = 18);

void oneIndexerCycle();
void tripleShotCycle();
void toggleFlywheelSpeed();

void togglePneumatics();
void togglePneumaticsY();
void togglePneumaticsRight();

void resetDriveEncoders();
double avgDriveEncoderValue();

void move(int degrees, int degreesPerSecond, bool hold = true);
void strafe(int degrees, int degreesPerSecond);
void rotateTo(double degrees, double speed, int differenceThreshold = 20);
void moveForward(int left, int right);
void moveStrafe(int left, int right);
void shoot(int count = 1, bool skipWait = false, double waitSec = 0.75);
int asyncShoot();

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function. You must return from this function    */
/*  or the autonomous and usercontrol tasks will not be started. This        */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration
  vexcodeInit();

  Pneumatics.close();

  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }

  resetDriveEncoders();
  InertialSensor.resetHeading();
  InertialSensor.resetRotation();
  wait(20, msec);

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 0);
  Controller.Screen.print("Initialized");
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  Pneumatics.close();

  Flywheel.spin(fwd, 11.3, volt);

  strafe(-680, 600);

  // Roller
  Intakes.spin(fwd, -360, rpm);
  move(170, 300);
  move(-100, 600);
  Intakes.stop();

  strafe(250, 600);

  rotateTo(18, 60, 30);

  // Preload disc
  shoot(1, true);

  rotateTo(135, 60);

  Intakes.spin(fwd, 390, rpm);
  Flywheel.spin(fwd, 10.1, volt);

  // Intake line of three
  move(1400, 600); //, false

  rotateTo(45.5, 60, 30);

  // Shoot 3 from center
  shoot(3);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Pneumatics.close();
  if (shootCount == 0) {
    Indexer.resetPosition();
  }

  Controller.Screen.clearScreen();

  Controller.ButtonUp.pressed(oneIndexerCycle);
  Controller.ButtonR2.pressed(tripleShotCycle);

  Controller.ButtonY.pressed(togglePneumaticsY);
  Controller.ButtonRight.pressed(togglePneumaticsRight);
  Controller.ButtonY.released(togglePneumaticsY);
  Controller.ButtonRight.released(togglePneumaticsRight);

  Controller.ButtonLeft.pressed(toggleFlywheelSpeed);

  while (true) {
    chassis(1.7, 1.1, 1.7);

    if (Controller.ButtonL1.pressing()) {
      Intakes.spin(fwd, 600, rpm);
    } else if (Controller.ButtonL2.pressing()) {
      Intakes.spin(fwd, -600, rpm);
    } else {
      Intakes.stop();
    }

    if (!isTripleShot) {
      if (Controller.ButtonR1.pressing()) {
        Flywheel.spin(fwd, 12 * flywheelPercent, voltageUnits::volt);
      } else if (Controller.ButtonDown.pressing()) {
        Flywheel.spin(fwd, -12 * flywheelPercent, voltageUnits::volt);
      } else {
        Flywheel.stop(coast);
      }
    }

    // Sleep for a short amount of time to prevent wasted resources
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

/* --- Autonomous period functions --- */
// -- Main functions -- //
void move(int degrees, int degreesPerSecond, bool hold) {
  // positive degrees = move forward
  // negative degrees = move backward
  int direction = abs(degrees) / degrees; // variable //

  resetDriveEncoders();
  InertialSensor.resetRotation();

  if (hold) {
    moveForward(
        degreesPerSecond * direction * 0.5 - InertialSensor.rotation() * 10,
        degreesPerSecond * direction * 0.5 + InertialSensor.rotation() * 10);

    wait(0.3, sec);
  }

  while (avgDriveEncoderValue() < abs(degrees)) {
    moveForward(degreesPerSecond * direction - InertialSensor.rotation() * 10,
                degreesPerSecond * direction + InertialSensor.rotation() * 10);
    wait(20, msec);
  } // Conditional statment //

  if (hold) {
    moveForward(-600 * direction, -600 * direction);
    wait(125, msec);

    moveForward(0, 0);
    wait(20, msec);
  }
} // Move forward or backward using internal motor encoders

void strafe(int degrees, int degreesPerSecond) {
  // positive degrees = strafe right
  // negative degrees = strafe left
  int direction = abs(degrees) / degrees;

  resetDriveEncoders();
  InertialSensor.resetRotation();

  moveStrafe(degreesPerSecond * direction * 0.5 - InertialSensor.rotation(),
             degreesPerSecond * direction * 0.5 + InertialSensor.rotation());
  wait(0.3, sec);

  while (avgDriveEncoderValue() < abs(degrees)) {
    moveStrafe(degreesPerSecond * direction - 10 * InertialSensor.rotation(),
               degreesPerSecond * direction + 10 * InertialSensor.rotation());
    wait(20, msec);
  }

  moveStrafe(-600 * direction, -600 * direction);
  wait(125, msec);

  moveStrafe(0, 0);
  wait(20, msec);
} // Strafe using internal motor encoders

void rotateTo(double degrees, double speed, int differenceThreshold) {
  double rightDeg = 0;
  double leftDeg = 0;
  double difference = 0;
  bool turnLeft = false;

  if (InertialSensor.heading() < degrees) {
    rightDeg = degrees - InertialSensor.heading();
    leftDeg = 360 + InertialSensor.heading() - degrees;
  } else {
    rightDeg = 360 - (InertialSensor.heading() - degrees);
    leftDeg = InertialSensor.heading() - degrees;
  }

  if (rightDeg > leftDeg) { // Turning left is shorter
    speed *= -1;
    difference = leftDeg;
    turnLeft = true;
  } else { // Turning right is shorter
    difference = rightDeg;
  }

  if (difference > differenceThreshold) {
    FrontLeft.spin(forward, speed, pct);
    FrontRight.spin(reverse, speed, pct);
    BackLeft.spin(forward, speed, pct);
    BackRight.spin(reverse, speed, pct);

    while (difference > differenceThreshold) {
      if (InertialSensor.heading() < degrees) {
        if (turnLeft) {
          difference = 360 + InertialSensor.heading() - degrees;
        } else {
          difference = degrees - InertialSensor.heading();
        }
      } else {
        if (turnLeft) {
          difference = InertialSensor.heading() - degrees;
        } else {
          difference = 360 - (InertialSensor.heading() - degrees);
        }
      }
      task::sleep(20);
    }
  }

  FrontLeft.spin(forward, speed * 0.1, pct);
  FrontRight.spin(reverse, speed * 0.1, pct);
  BackLeft.spin(forward, speed * 0.1, pct);
  BackRight.spin(reverse, speed * 0.1, pct);

  int overshootError = 3;
  if (turnLeft) {
    waitUntil(degrees + overshootError >= InertialSensor.heading() &&
              InertialSensor.heading() >= degrees);
  } else {
    waitUntil(degrees + overshootError >= InertialSensor.heading() &&
              InertialSensor.heading() >= (degrees));
  }

  FrontLeft.stop();
  FrontRight.stop();
  BackLeft.stop();
  BackRight.stop();
  wait(20, msec);
}

// -- Helper functions -- //
void resetDriveEncoders() {
  FrontLeft.resetPosition();
  FrontRight.resetPosition();
  BackLeft.resetPosition();
  BackRight.resetPosition();
} // Reset chassis motor encoders

double avgDriveEncoderValue() {
  return (fabs(FrontLeft.position(deg)) + fabs(FrontRight.position(deg)) +
          fabs(BackLeft.position(deg)) + fabs(BackRight.position(deg))) /
         4;
} // Get average value of all the chassis motor encoders

void moveForward(int left, int right) {
  FrontLeft.spin(fwd, left, dps);
  FrontRight.spin(fwd, right, dps);
  BackLeft.spin(fwd, left, dps);
  BackRight.spin(fwd, right, dps);
} // Spin chassis forward or backward

void moveStrafe(int left, int right) {
  FrontLeft.spin(fwd, left, dps);
  FrontRight.spin(fwd, -left, dps);
  BackLeft.spin(fwd, -right, dps);
  BackRight.spin(fwd, right, dps);
} // Strafe chassis right or left

void shoot(int count, bool skipWait, double waitSec) {
  if (shootCount == 0) {
    Indexer.resetPosition();
  }

  for (int i = 0; i < count; i++) {
    shootCount++;
    Indexer.rotateTo(360 * shootCount, deg, 200, rpm);
    Indexer.stop(hold);
    if (!skipWait) {
      wait(waitSec, sec);
    }
  }
}

int asyncShoot() {
  while (shouldAsyncShoot) {
    shoot(1);
    wait(20, msec);
  }
  return 0;
}

/* --- Driver period functions --- */
void chassis(double forwardScale, double turnScale, double strafeScale,
             int deadZone) {
  double forward = Controller.Axis3.position() * forwardScale;
  double turnVal = Controller.Axis4.position() * turnScale;
  double strafe = Controller.Axis1.position() * strafeScale;

  int frontLeftVoltage = forward + strafe + turnVal;
  int frontRightVoltage = forward - strafe - turnVal;
  int backLeftVoltage = forward + strafe - turnVal;
  int backRightVoltage = forward - strafe + turnVal;

  bool frontLeftMoving =
      !(-deadZone < frontLeftVoltage && frontLeftVoltage < deadZone);
  bool frontRightMoving =
      !(-deadZone < frontRightVoltage && frontRightVoltage < deadZone);
  bool backLeftMoving =
      !(-deadZone < backLeftVoltage && backLeftVoltage < deadZone);
  bool backRightMoving =
      !(-deadZone < backRightVoltage && backRightVoltage < deadZone);

  if (frontLeftMoving || frontRightMoving || backLeftMoving ||
      backRightMoving) {
    FrontLeft.spin(fwd, frontLeftVoltage, rpm);
    FrontRight.spin(fwd, frontRightVoltage, rpm);
    BackLeft.spin(fwd, backLeftVoltage, rpm);
    BackRight.spin(fwd, backRightVoltage, rpm);

  } else {
    FrontLeft.stop(brakeType::brake);
    FrontRight.stop(brakeType::brake);
    BackLeft.stop(brakeType::brake);
    BackRight.stop(brakeType::brake);
  }
} // Moving, rotating, and strafing for chassis

void oneIndexerCycle() {
  if (!indexerCycling) {
    indexerCycling = true;
    shootCount++;
    Indexer.rotateTo(360 * shootCount, deg, 200, rpm);
    Indexer.stop(hold);
    indexerCycling = false;
  }
}

void tripleShotCycle() {
  if (!indexerCycling) {
    indexerCycling = isTripleShot = true;

    shootCount++;
    Indexer.rotateTo(360 * shootCount, deg, 200, rpm);

    Flywheel.spin(fwd, 12, volt);
    Indexer.stop(hold);
    wait(0.1, sec);
    Flywheel.spin(fwd, 9.5, volt);
    shootCount++;
    Indexer.rotateTo(360 * shootCount, deg, 200, rpm);

    Flywheel.spin(fwd, 12, volt);
    Indexer.stop(hold);
    wait(0.1, sec);
    Flywheel.spin(fwd, 9.5, volt);
    shootCount++;
    Indexer.rotateTo(360 * shootCount, deg, 200, rpm);

    Indexer.stop(hold);

    indexerCycling = isTripleShot = false;
  }
}

void toggleFlywheelSpeed() {
  Controller.Screen.clearLine(1);
  Controller.Screen.setCursor(1, 0);
  wait(50, msec);

  if (flywheelPercent == flywheelUpperPercent) {
    Controller.Screen.print("Slow Mode");
    flywheelPercent = flywheelLowerPercent;
  } else {
    Controller.Screen.print("Fast Mode");
    flywheelPercent = flywheelUpperPercent;
  }

  wait(50, msec);
}

void togglePneumatics() {
  if (pneumaticsYPressed && pneumaticsRightPressed) {
    Controller.Screen.clearLine(2);
    Controller.Screen.setCursor(2, 0);
    wait(50, msec);

    if (Pneumatics.value() == 0) {
      FrontRight.stop(hold);
      FrontLeft.stop(hold);
      BackLeft.stop(hold);
      BackRight.stop(hold);
      Pneumatics.open();
      Controller.Screen.print("Pneumatics Open");
    } else {
      Pneumatics.close();
      Controller.Screen.print("Pneumatics Closed");
    }

    wait(50, msec);
  }
}

void togglePneumaticsY() {
  pneumaticsYPressed = !pneumaticsYPressed;
  togglePneumatics();
}

void togglePneumaticsRight() {
  pneumaticsRightPressed = !pneumaticsRightPressed;
  togglePneumatics();
}