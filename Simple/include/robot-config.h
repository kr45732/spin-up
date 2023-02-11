using namespace vex;

extern brain Brain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

/* --- Controller --- */
extern controller Controller;

/* --- Motors --- */
extern motor FrontLeft;
extern motor FrontRight;
extern motor BackLeft;
extern motor BackRight;

extern motor Intakes;
extern motor Indexer;
extern motor_group Flywheel;

extern inertial InertialSensor;

extern pneumatics Pneumatics;