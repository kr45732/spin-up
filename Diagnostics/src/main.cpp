/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Krish Ranjan                                              */
/*    Created:      Tue Sep 01 2020                                           */
/*    Description:  Diagnostics program for Spin Up                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

/* --- Motor constructors --- */
motor FrontLeft = motor(PORT15, ratio18_1, false);
motor FrontRight = motor(PORT3, ratio18_1, true);
motor BackLeft = motor(PORT20, ratio18_1, false);
motor BackRight = motor(PORT8, ratio18_1, true);

motor Intakes = motor(PORT6, ratio6_1, false);
motor Indexer = motor(PORT18, ratio18_1, true);

motor FlywheelOne = motor(PORT10, ratio6_1, true);
motor FlywheelTwo = motor(PORT5, ratio6_1, false);
motor_group Flywheel = motor_group(FlywheelOne, FlywheelTwo);

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Brain.Screen.clearScreen();

  Brain.Screen.setPenColor(vex::color::red);
  Brain.Screen.drawLine(180, 70, 300, 70);

  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFont(fontType::mono60);
  Brain.Screen.printAt(180, 50, "421H");

  // Temperature levels guide
  Brain.Screen.setFont(fontType::mono40);
  Brain.Screen.printAt(170, 140, "T1 - 55");
  Brain.Screen.printAt(170, 180, "T2 - 60");
  Brain.Screen.printAt(170, 220, "T3 - 65");

  Brain.Screen.setFillColor(vex::color::red);

  // Chassis
  Brain.Screen.drawRectangle(0, 90, 150, 150);

  // Flywheel
  Brain.Screen.drawRectangle(0, 0, 45, 90);
  Brain.Screen.drawRectangle(105, 0, 45, 90);

  // Misc
  Brain.Screen.drawRectangle(330, 0, 150, 150);

  // Battery
  Brain.Screen.drawRectangle(330, 165, 150, 70);

  while (true) {
    Brain.Screen.setFont(fontType::mono40);

    // Chassis
    Brain.Screen.printAt(2, 125, "%0.0f",
                         FrontLeft.temperature(celsius)); // Front Left
    Brain.Screen.printAt(107, 125, "%0.0f",
                         FrontRight.temperature(celsius)); // Front Right
    Brain.Screen.printAt(2, 225, "%0.0f",
                         BackLeft.temperature(celsius)); // Back Left
    Brain.Screen.printAt(107, 225, "%0.0f",
                         BackRight.temperature(celsius)); // Back Right

    // Flywheel
    Brain.Screen.printAt(2, 45, "2:%0.0f",
                         FlywheelOne.temperature(celsius));
    Brain.Screen.printAt(107, 45, "%0.0f",
                         FlywheelTwo.temperature(celsius));

    // Misc
    Brain.Screen.printAt(340, 38, "Int: %0.0f",
                         Intakes.temperature(celsius));
    Brain.Screen.printAt(340, 138, "Idx: %0.0f",
                         Indexer.temperature(celsius));

    // Battery
    Brain.Screen.setFont(fontType::mono60);
    Brain.Screen.printAt(355, 218, "%d%%%", Brain.Battery.capacity());

    wait(20, msec);
  }
}
