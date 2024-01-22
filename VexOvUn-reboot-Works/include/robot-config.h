using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group LeftSide;
extern motor_group RightSide;
extern motor_group MotorGroup19;
extern motor Hang;
extern digital_out wing;
extern gps gpss;
extern motor Motor17;
extern optical eye;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );