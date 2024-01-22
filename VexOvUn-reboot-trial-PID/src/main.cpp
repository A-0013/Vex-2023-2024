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
// ---- END VEXCODE CONFIGURED DEVICES ----
#define backThatAssUp FPID(12);
#include "vex.h"
#include "robot-config.h"
using namespace vex;
bool tRed = true; //team red

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

// Placeholder variables for PID control
int target_position = 0;
int L_motor_output = 0;
int R_motor_output = 0;
int Lefterror = 0;
int Righterror = 0;


// PID constants
double kp = 0.1;
double ki = 0.01;
double kd = 0.05;

double integralBound = 0.8;

// Integral and derivative terms for PID
double L_integral = 0;
double L_derivative = 0;
double L_last_error = 0;

double R_integral = 0;
double R_derivative = 0;
double R_last_error = 0;

// Function to set drivetrain speed
void setDrivetrainSpeed(int left_speed, int right_speed) {
  LeftSide.spin(forward, left_speed, pct);
  RightSide.spin(forward, right_speed, pct);
}

void setTurntrainSpeed(int left_speed, int right_speed) {
  LeftSide.spin(forward, left_speed, pct);
  RightSide.spin(forward, right_speed, pct);
}

int LeftgeEncoderValue() {
  return LeftSide.position(degrees);
  //print(Leftside.position(degrees));
    
}

int RightgeEncoderValue() {
  return RightSide.position(degrees);
   
}


// Function for autonomous PID movement
void FPID(int target) {
    // Main control loop

    target = (target / 4.25 * M_PI) * 360;
    target_position = target;
    L_motor_output = 0;
    R_motor_output = 0;
    Lefterror = 0;
    Righterror = 0;
    L_integral = 0;
    L_derivative = 0;
    L_last_error = 0;

    R_integral = 0;
    R_derivative = 0;
    R_last_error = 0;

    LeftSide.setPosition(0,degrees);
    RightSide.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - LeftgeEncoderValue();  // Adjust based on your encoder reading function
        Righterror = target_position - RightgeEncoderValue();

        // Proportional, integral, and derivative terms
        double Leftproportional = kp * Lefterror;
        L_integral += ki * Lefterror;
        L_derivative = kd * (Lefterror - L_last_error);

        // Calculate motor output
        L_motor_output = Leftproportional + L_integral + L_derivative;

        // Proportional, integral, and derivative terms
        double Rightproportional = kp * Righterror;
        R_integral += ki * Righterror;
        R_derivative = kd * (Righterror - R_last_error);

        // Calculate motor output
        R_motor_output = Rightproportional + R_integral + R_derivative;

        // Calculate motor speeds based on PID output
        int left_motor_speed = L_motor_output;
        int right_motor_speed = R_motor_output;

        // Apply motor speeds to the drivetrain
        setDrivetrainSpeed(left_motor_speed, right_motor_speed);

        // Update last error for the next iteration
        L_last_error = Lefterror;
        R_last_error = Righterror;


        // Check if the target position is reached within a tolerance

        if(abs(Lefterror) < integralBound || abs(Righterror) < integralBound){
          L_integral+=Lefterror*L_derivative;
          R_integral+=Righterror*R_derivative;
        } 
        else
        {
          L_integral = 0; 
          R_integral = 0; 
        }

        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
          // Stop the drivetrain when the target position is reached
          setDrivetrainSpeed(0, 0);
          break;
        }

        Brain.Screen.print("Left Encoder: %f", LeftSide.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RightSide.position(degrees));
    }
}

// Function for autonomous PID movement
void TPID(int target) {
    // Main control loop

    target = target * 3;
    target_position = target;

    L_motor_output = 0;
    R_motor_output = 0;
    Lefterror = 0;
    Righterror = 0;
    L_integral = 0;
    L_derivative = 0;
    L_last_error = 0;

    R_integral = 0;
    R_derivative = 0;
    R_last_error = 0;

    LeftSide.setPosition(0,degrees);
    RightSide.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - LeftgeEncoderValue();  // Adjust based on your encoder reading function
        Righterror = target_position - RightgeEncoderValue();

        // Proportional, integral, and derivative terms
        double Leftproportional = kp * Lefterror;
        L_integral += ki * Lefterror;
        L_derivative = kd * (Lefterror - L_last_error);

        // Calculate motor output
        L_motor_output = Leftproportional + L_integral + L_derivative;

        // Proportional, integral, and derivative terms
        double Rightproportional = kp * Righterror;
        R_integral += ki * Righterror;
        R_derivative = kd * (Righterror - R_last_error);

        // Calculate motor output
        R_motor_output = Rightproportional + R_integral + R_derivative;

        // Calculate motor speeds based on PID output
        int left_motor_speed = L_motor_output;
        int right_motor_speed = R_motor_output;

        // Apply motor speeds to the drivetrain
        setTurntrainSpeed(left_motor_speed, right_motor_speed);

        // Update last error for the next iteration
        L_last_error = Lefterror;
        R_last_error = Righterror;


        // Check if the target position is reached within a tolerance

        if(abs(Lefterror) < integralBound || abs(Righterror) < integralBound){
          L_integral+=Lefterror*L_derivative;
          R_integral+=Righterror*R_derivative;
        }  else {
            L_integral = 0; 
            R_integral = 0; 
        }

        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
          // Stop the drivetrain when the target position is reached
          setDrivetrainSpeed(0, 0);
          break;
        }

        Brain.Screen.print("Left Encoder: %f", LeftSide.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RightSide.position(degrees));
    }
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

void autonomous(void) {
  //auton game
  FPID(12);
  //wait(30, seconds);
  //TPID(90);
  //auton 
  
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
    
    MotorGroup19.setMaxTorque(80,percent);
    //MotorGroup19.setVelocity(80,percent);
    if (Controller1.ButtonUp.pressing() == true)
    {
      MotorGroup19.spin(forward);
    }
    if (Controller1.ButtonDown.pressing() == true)
    {
      MotorGroup19.stop();
    }


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
