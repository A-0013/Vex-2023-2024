#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
digital_out Wings1 = digital_out(Brain.ThreeWirePort.E);
digital_out Wings2 = digital_out(Brain.ThreeWirePort.D);
motor IntakeMotorA = motor(PORT1, ratio6_1, true);
motor IntakeMotorB = motor(PORT10, ratio6_1, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);
motor shooter = motor(PORT18, ratio6_1, false);
motor Lift = motor(PORT15, ratio18_1, false);
digital_out Hanger = digital_out(Brain.ThreeWirePort.F);
motor LDrive_GroupMotorA = motor(PORT11, ratio18_1, false);
motor LDrive_GroupMotorB = motor(PORT13, ratio18_1, false);
motor_group LDrive_Group = motor_group(LDrive_GroupMotorA, LDrive_GroupMotorB);
motor RDrive_GroupMotorA = motor(PORT12, ratio18_1, false);
motor RDrive_GroupMotorB = motor(PORT14, ratio18_1, false);
motor_group RDrive_Group = motor_group(RDrive_GroupMotorA, RDrive_GroupMotorB);
gps GPS8 = gps(PORT8, 0.00, 0.00, mm, 180);

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