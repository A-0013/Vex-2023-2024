using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern digital_out Wings1;
extern digital_out Wings2;
extern motor_group Intake;
extern motor shooter;
extern motor Lift;
extern digital_out Hanger;
extern motor_group LDrive_Group;
extern motor_group RDrive_Group;
extern gps GPS8;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );