#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}

/* --- Controller constructor --- */
controller Controller = controller(primary);

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

inertial InertialSensor = inertial(PORT1);

pneumatics Pneumatics = pneumatics(Brain.ThreeWirePort.G);