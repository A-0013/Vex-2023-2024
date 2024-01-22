/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftSide             motor_group   2, 3            
// RightSide            motor_group   4, 5            
// MotorGroup19         motor_group   19, 20          
// Hang                 motor         6               
// wing                 digital_out   A               
// gpss                 gps           18              
// Motor17              motor         17              
// eye                  optical       7               
// ---- END VEXCODE CONFIGURED DEVICES ----
#define backThatAssUp Drivetrain.drive(forward)
#define aanshgotgetsumpussy
#include "vex.h"
#include "robot-config.h"
using namespace vex;
bool tRed = true; //team red
int count = 0;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void LDF (vex::directionType type, int percentage)
{
  if (abs(percentage) <= 30)
  {
    percentage = 0;
  }
  if (percentage >= 0)
  {
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
  }
  else
  {
    percentage = -1 * percentage;
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
    percentage = -1 * percentage;
  }
  LeftSide.spin(type, percentage, vex::velocityUnits::pct);
}

void RDF (vex::directionType type, int percentage)
{
  if (abs(percentage) <= 30)
  {
    percentage = 0;
  }
  if (percentage >= 0)
  {
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
  }
  else
  {
    percentage = -1 * percentage;
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
    percentage = -1 * percentage;
  }
  RightSide.spin(type, percentage, vex::velocityUnits::pct);
}

void Drive (double x) // takes in inches
{
  double D_degrees = (x / 12.95) * 360; //find how many degrees to drive
  RightSide.spinFor(reverse, D_degrees, degrees, false); //drive that much
  LeftSide.spinFor(forward, D_degrees, degrees);
}


void Turn (int x, bool Direct) //takes in degree and dirrection
{
  int T_degrees = x * 3; //multiply by 3 for the correct degree
  aanshgotgetsumpussy;

  if (Direct == true) //if true turn right
  {
    RightSide.spinFor(forward, T_degrees, degrees, false); //go
    LeftSide.spinFor(forward, T_degrees, degrees);
  }
  if (Direct == false) //if false turn left
  {
    RightSide.spinFor(reverse, T_degrees, degrees, false); //go
    LeftSide.spinFor(reverse, T_degrees, degrees);
  }
}


void buttY()
{
  Motor17.spinFor(forward,180,degrees);
  Motor17.setStopping(hold);
}

void buttA()
{
  Motor17.spinFor(reverse,190,degrees);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


bool skills = false;

void autonomous(void) {

  //Must be 9.5 inches to 11.5 inches tall
  if (skills == false)
  {

    LeftSide.setVelocity(90,percent);
    RightSide.setVelocity(90,percent);
  
    Drive(19.5);
    Drive(-19.5);
    Turn(62,false);
    Drive(45);
    Turn(62,true);
    Drive(8);
    wait(30, sec);
    /*
    if (gpss.xPosition() > 0 && gpss.yPosition() > 0) //Q1 red side/blue corner
    {
      Drive(22);
      Drive(-22);
      Turn(100,false);
      Drive(50);
    }

    else if(gpss.xPosition() < 0 && gpss.yPosition() > 0) //Q2 red side red corner
    {
      //no net
      Drive(19.5);
      Drive(-19.5);
      Turn(55,false);
      Drive(44);
      Turn(55,true);
      Drive(8);
      wait(30, sec);
    }
    else if(gpss.xPosition() < 0 && gpss.yPosition() < 0) //Q3 blue side red corner
    {
      Drive(22);
      Drive(-22);
      Turn(100,false);
      Drive(50);
     }

    else if(gpss.xPosition() > 0 && gpss.yPosition() < 0) //Q4 blue side blue corner
    {
      //no net
      Drive(19.5);
      Drive(-19.5);
      Turn(55,false);
      Drive(44);
      Turn(55,true);
      Drive(8);
      wait(30, sec);
    }
    */
  }
  
  if (skills == true)
  {
    MotorGroup19.setVelocity(95,percent);
    LeftSide.setVelocity(80,percent);
    RightSide.setVelocity(80,percent);
    MotorGroup19.spinFor(forward,16200,degrees); //true = R false = L
    MotorGroup19.spinFor(forward,16200,degrees);
    Drive(24);
    Turn(45, false);
    Drive(91);
    Turn(90, true);
    Drive(34);

    /*Turn(45,true);
    Drive(102);
    Turn(45,false);
    Drive(17); //i think it here
    Turn(45, false);
    Drive(24);
    Drive(-24);
    Turn(45, false);
    Drive(4.2);
    Turn(45, true);
    Drive(6);
    Drive(-18);
    Turn(90, false);
    Drive(42);
    Turn(90, true);
    Drive(20);
    Turn(90, true);
    Drive(42);
    Drive(-42);
    Turn(90, false);
    Drive(18);
    Turn(90, true);
    Drive(42);
    Drive(-42);
    Turn(90, false);
    Drive(18);
    Turn(90, true);
    Drive(42);
    Drive(-42);
    Turn(45, false); 
    Drive(25.5); 
    Turn(45, true);
    Drive(18);
    Turn(90, true);
    Drive(18);
    Drive(-18); 
    Turn(45, false);
    Drive(3); 
    Turn(45, true);
    Drive(2);
    Drive(-2);
    Turn(135, true); 
    Drive(12.7);
    Turn(45, false);
    Hang.spinFor(forward, 180, degrees); 
    Drive(60);*/


  }
}


  


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    LDF (vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis4.value()));
    RDF (vex::directionType::rev, (Controller1.Axis3.value() - Controller1.Axis4.value()));

    Brain.Screen.print("This drive code is proved by Aansh and Logan");
    MotorGroup19.setMaxTorque(95,percent);
    MotorGroup19.setVelocity(95,percent);
    
    int temp = MotorGroup19.temperature(percent);
    Controller1.Screen.print(temp);
    if (temp >= 55)
    {
      Controller1.Screen.newLine();
      Controller1.Screen.print("Too hot, current cut by 50%");
    }


    if (Controller1.ButtonUp.pressing() == true)
    {
      MotorGroup19.spin(forward);
    }
    if (Controller1.ButtonDown.pressing() == true)
    {
      MotorGroup19.stop();
    }

    if (Controller1.ButtonX.pressing() == true)
    {
      Hang.spinFor(forward, 85, degrees);
    }
    if (Controller1.ButtonB.pressing() == true)
    {
      Hang.spinFor(reverse, 85, degrees);
    }

    if (Controller1.ButtonRight.pressing() == true)
    {
      wing.set(true);
    }
    if (Controller1.ButtonRight.pressing() == false)
    {
      wing.set(false);
    }

    Controller1.ButtonY.pressed(buttY);
    Controller1.ButtonA.pressed(buttA);



    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
    // prevent wasted resources.
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
