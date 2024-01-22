#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftSideMotorA = motor(PORT2, ratio18_1, true);
motor LeftSideMotorB = motor(PORT3, ratio18_1, true);
motor_group LeftSide = motor_group(LeftSideMotorA, LeftSideMotorB);
motor RightSideMotorA = motor(PORT4, ratio18_1, true);
motor RightSideMotorB = motor(PORT5, ratio18_1, true);
motor_group RightSide = motor_group(RightSideMotorA, RightSideMotorB);
motor MotorGroup19MotorA = motor(PORT19, ratio18_1, false);
motor MotorGroup19MotorB = motor(PORT20, ratio18_1, true);
motor_group MotorGroup19 = motor_group(MotorGroup19MotorA, MotorGroup19MotorB);
motor Hang = motor(PORT6, ratio36_1, false);
digital_out wing = digital_out(Brain.ThreeWirePort.A);
gps gpss = gps(PORT18, 114.30, 25.40, mm, 180);
motor wingthing = motor(PORT17, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}