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
// Wings1               digital_out   E               
// Wings2               digital_out   D               
// Lift                 motor_group   15, 10          
// Intake               motor         1               
// Hanger               digital_out   F               
// LDrive_Group         motor_group   11, 13          
// RDrive_Group         motor_group   12, 14          
// GPS8                 gps           8               
// shooter              motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#define Abdul_Shoot_Your_Shot_With_Isabelle L_motor_output = 0;
using namespace vex;

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

// Placeholder variables for PID control
int target_position = 0;
int L_motor_output = 0;
int R_motor_output = 0;
int Lefterror = 0;
int Righterror = 0;


// PID constants
double kp = 0.183;
double ki = 0.015;
double kd = 0.05;
//Turn
double Tkp = 0.25;
double Tki = 0.019;
double Tkd = 0.078;

double integralBound = 0.8;

// Integral and derivative terms for PID
double L_integral = 0;
double L_derivative = 0;
double L_last_error = 0;

double R_integral = 0;
double R_derivative = 0;
double R_last_error = 0;

// Function to set drivetrain speed
void setFWDDrivetrainSpeed(int left_speed, int right_speed) {
    LDrive_Group.spin(forward, left_speed, pct);
    RDrive_Group.spin(reverse, right_speed, pct);
}

void setREVDrivetrainSpeed(int left_speed, int right_speed) {
    LDrive_Group.spin(reverse, left_speed, pct);
    RDrive_Group.spin(forward, right_speed, pct);
}

void setRightTurntrainSpeed(int left_speed, int right_speed) {
    LDrive_Group.spin(forward, left_speed, pct);
    RDrive_Group.spin(forward, right_speed, pct);
}

void setLeftTurntrainSpeed(int left_speed, int right_speed) {
    LDrive_Group.spin(reverse, left_speed, pct);
    RDrive_Group.spin(reverse, right_speed, pct);
}

int LeftgeEncoderValue() {
    return LDrive_Group.position(degrees);
    //print(LDrive_Group.position(degrees));
    
}

int RightgeEncoderValue() {
    return LDrive_Group.position(degrees);
   
}


// Function for autonomous PID movement
void FWDPID(int target) {
    // Main control loop

    target_position = (target/(4*M_PI)) * 240;
    //target_position = target;
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

    LDrive_Group.setPosition(0,degrees);
    RDrive_Group.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - abs(LeftgeEncoderValue());  // Adjust based on your encoder reading function
        Righterror = target_position - abs(RightgeEncoderValue());

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

          setFWDDrivetrainSpeed(left_motor_speed, right_motor_speed);
   

        // Update last error for the next iteration
        L_last_error = Lefterror;
        R_last_error = Righterror;


        if(abs(Lefterror) < integralBound || abs(Righterror) < integralBound){
          L_integral+=Lefterror*L_derivative;
          R_integral+=Righterror*R_derivative;
        }  else {
            L_integral = 0; 
            R_integral = 0; 
        }

        // Check if the target position is reached within a tolerance
        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
            // Stop the drivetrain when the target position is reached
            setFWDDrivetrainSpeed(0, 0);
            setREVDrivetrainSpeed(0, 0);
            break;
        }

        Brain.Screen.print("Left Encoder: %f", LDrive_Group.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RDrive_Group.position(degrees));
    }
}

void REVPID(int target) {
    // Main control loop
    target_position = (target/(4*M_PI)) * 240;
    //target_position = target;
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

    LDrive_Group.setPosition(0,degrees);
    RDrive_Group.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - abs(LeftgeEncoderValue());  // Adjust based on your encoder reading function
        Righterror = target_position - abs(RightgeEncoderValue());

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
  
          setREVDrivetrainSpeed(left_motor_speed, right_motor_speed);
      

        // Update last error for the next iteration
        L_last_error = Lefterror;
        R_last_error = Righterror;


        if(abs(Lefterror) < integralBound || abs(Righterror) < integralBound){
          L_integral+=Lefterror*L_derivative;
          R_integral+=Righterror*R_derivative;
        }  else {
            L_integral = 0; 
            R_integral = 0; 
        }

        // Check if the target position is reached within a tolerance
        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
            // Stop the drivetrain when the target position is reached
            setFWDDrivetrainSpeed(0, 0);
            setREVDrivetrainSpeed(0, 0);
            break;
        }

        Brain.Screen.print("Left Encoder: %f", LDrive_Group.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RDrive_Group.position(degrees));
    }
}

// Function for autonomous PID movement
void RightPID(int target) {
    // Main control loop
    target_position = target;


    
    Abdul_Shoot_Your_Shot_With_Isabelle
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

    LDrive_Group.setPosition(0,degrees);
    RDrive_Group.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - abs(LeftgeEncoderValue());  // Adjust based on your encoder reading function
        Righterror = target_position - abs(RightgeEncoderValue());

        // Proportional, integral, and derivative terms
        double Leftproportional = Tkp * Lefterror;
        L_integral += Tki * Lefterror;
        L_derivative = Tkd * (Lefterror - L_last_error);

        // Calculate motor output
        L_motor_output = Leftproportional + L_integral + L_derivative;

        // Proportional, integral, and derivative terms
        double Rightproportional = Tkp * Righterror;
        R_integral += Tki * Righterror;
        R_derivative = Tkd * (Righterror - R_last_error);

        // Calculate motor output
        R_motor_output = Rightproportional + R_integral + R_derivative;

        // Calculate motor speeds based on PID output
        int left_motor_speed = L_motor_output;
        int right_motor_speed = R_motor_output;

        // Apply motor speeds to the drivetrain

        setRightTurntrainSpeed(left_motor_speed, right_motor_speed);


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

        // Check if the target position is reached within a tolerance
        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
            // Stop the drivetrain when the target position is reached
            setRightTurntrainSpeed(0, 0);
            //setLeftTurntrainSpeed(0,0);
            break;
        }

        Brain.Screen.print("Left Encoder: %f", LDrive_Group.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RDrive_Group.position(degrees));
    }
}

void LeftPID(int target) {
    // Main control loop

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

    LDrive_Group.setPosition(0,degrees);
    RDrive_Group.setPosition(0,degrees);

    while (true) {
        // Placeholder PID calculations
        Lefterror = target_position - abs(LeftgeEncoderValue());  // Adjust based on your encoder reading function
        Righterror = target_position - abs(RightgeEncoderValue());

        // Proportional, integral, and derivative terms
        double Leftproportional = Tkp * Lefterror;
        L_integral += Tki * Lefterror;
        L_derivative = Tkd * (Lefterror - L_last_error);

        // Calculate motor output
        L_motor_output = Leftproportional + L_integral + L_derivative;

        // Proportional, integral, and derivative terms
        double Rightproportional = Tkp * Righterror;
        R_integral += Tki * Righterror;
        R_derivative = Tkd * (Righterror - R_last_error);

        // Calculate motor output
        R_motor_output = Rightproportional + R_integral + R_derivative;

        // Calculate motor speeds based on PID output
        int left_motor_speed = L_motor_output;
        int right_motor_speed = R_motor_output;

        // Apply motor speeds to the drivetrain

        setLeftTurntrainSpeed(left_motor_speed, right_motor_speed);


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

        // Check if the target position is reached within a tolerance
        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
            // Stop the drivetrain when the target position is reached
            //setRightTurntrainSpeed(0, 0);
            setLeftTurntrainSpeed(0,0);
            break;
        }

        Brain.Screen.print("Left Encoder: %f", LDrive_Group.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RDrive_Group.position(degrees));
    }
}

void CircleautonomousPIDMovement(double right_distance, double left_distance)
{

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

    LDrive_Group.setPosition(0,degrees);
    RDrive_Group.setPosition(0,degrees);
      // Main control loop
    while (true) {
        // Placeholder PID calculations
        Lefterror = left_distance - LeftgeEncoderValue();  // Adjust based on your encoder reading function
        Righterror = right_distance - RightgeEncoderValue();

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
        setFWDDrivetrainSpeed(left_motor_speed, right_motor_speed);

        // Update last error for the next iteration
        L_last_error = Lefterror;
        R_last_error = Righterror;


        if(abs(Lefterror) < integralBound || abs(Righterror) < integralBound){
          L_integral+=Lefterror*L_derivative;
          R_integral+=Righterror*R_derivative;
        }  else {
            L_integral = 0; 
            R_integral = 0; 
        }

        // Check if the target position is reached within a tolerance
        if (abs(Lefterror) < 10 || abs(Righterror) < 10) {
            // Stop the drivetrain when the target position is reached
            setFWDDrivetrainSpeed(0, 0);
            break;
        }

        Brain.Screen.print("Left Encoder: %f", LDrive_Group.position(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Right Encoder: %f", RDrive_Group.position(degrees));
    }
}

int referenceAngle;
int actualAngle;
double convturn;
double pidTurn;
double targetX;
double targetY;
double convDrive;
double pidDrive;

void driveToPositionX(double x) {

  // Reorient the robot before driving along the X-axis
  if (GPS8.xPosition(mm) < x) {
    referenceAngle = 90;
  }
  else {
    referenceAngle = 270;
  }
  // Using an absolute reference angle along with the GPS heading,
  // we can turn the robot to face the correct direction before driving along the X-axis
  actualAngle = referenceAngle - GPS8.heading();
  convturn = (actualAngle*12* M_PI)/(4*M_PI);
  pidTurn = convturn * 360 * 0.6;
  //TurnautonomousPIDMovement(pidTurn, true);

  while(fabs(x - GPS8.xPosition()) > 5)
  {
  
  if (GPS8.xPosition(mm) < x) {
    targetX = x - GPS8.xPosition(mm);
  }
  else {
    targetX = GPS8.xPosition(mm) - x;
  }

  convDrive = targetX*25.4;
  pidDrive = (convDrive/(4*M_PI)) * 360 * 0.6;

  //DriveautonomousPIDMovement(pidDrive, true);

  }

}

void driveToPositionY(double y) {

  // Reorient the robot before driving along the Y-axis
  if (GPS8.yPosition(mm) < y) {
    referenceAngle = 0;
  }  
  else {
    referenceAngle = 180;
  }

    // Using an absolute reference angle along with the GPS heading,
    // we can turn the robot to face the correct direction before driving along the Y-axis
    actualAngle = referenceAngle - GPS8.heading();
    convturn = (actualAngle*12*M_PI)/(4*M_PI);
    pidTurn = convturn * 360 * 0.6;
    //TurnautonomousPIDMovement(pidTurn, true);



  while(fabs(y - GPS8.yPosition()) > 5)
  {

    if (GPS8.yPosition(mm) < y) {
      targetY = y - GPS8.yPosition(mm);
    }
    else {
      targetY = GPS8.yPosition(mm) - y;
    }

    convDrive = targetY*25.4;
    pidDrive = (convDrive/(4*M_PI)) * 360 * 0.6;

    //DriveautonomousPIDMovement(pidDrive, true);

  }
}

void GoTo(double x, double y)
{

    driveToPositionX(x);
    driveToPositionY(y);
}

double right_Speed;
double left_Speed;
// Distance between left wheels and right wheels
double wheelbase = 12.0;

void circle(double radius, double degree, bool dir)//using time
{
    
    if(dir)//if true the robot will go in a counter clockwise movement
    {
      right_Speed = 50.0; // Adjust this speed as needed
      left_Speed = (radius - wheelbase / 2.0) / (radius + wheelbase / 2.0) * right_Speed;
    }
    else //if false the robot will in a clockwise movement
    {
      left_Speed = 50.0; // Adjust this speed as needed
      right_Speed = (radius - wheelbase / 2.0) / (radius + wheelbase / 2.0) * left_Speed;
    }



    // Calculate the time needed to cover the specified number of degrees
    double duration = degree / (360.0 / (2.0 * M_PI * radius));

    // Drive the robot in a circle
    setFWDDrivetrainSpeed(left_Speed, right_Speed); 


    // Allow the robot to drive in a circle for the specified time
    wait(duration, seconds);

    // Stop the motors
    LDrive_Group.stop();
    RDrive_Group.stop();

    
}

double right_distance;
double left_distance;

void PIDcircle (double radius, double degree, bool dir)
{
    if(dir)//if true the robot will go in a counter clockwise movement
    {
      right_distance = (2*radius*M_PI*degree)/360; 
      right_distance = right_distance * 360 * 0.6;
      left_distance = (2*(radius - wheelbase)) / 360;
      left_distance = left_distance * 360 * 0.6;
    }
    else //if false the robot will in a clockwise movement
    {
      left_distance = (2*radius*M_PI*degree)/360; 
      left_distance = right_distance * 360 * 0.6;
      right_distance = (2*(radius - wheelbase)) / 360;
      right_distance = left_distance * 360 * 0.6;
    }

    CircleautonomousPIDMovement(right_distance, left_distance);

}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
Hanger.set(true);

//Close Side
/*
FWDPID(46);
LeftPID(184);
Intake.spin(reverse, 95, pct);
wait(1,sec);
Intake.stop();
RightPID(184);
RightPID(184);
REVPID(8);
//LeftPID(162);
Wings1.set(true);
Wings2.set(true);
FWDPID(18);


//RightPID(324);

//Far Side 
FWDPID(50);
RightPID(184);
FWDPID(12);
Intake.spin(reverse, 95, pct);
wait(1,sec);
Intake.stop();
REVPID(12);
RightPID(368);
FWDPID(30);
Intake.spin(forward, 95, pct);
wait(1,sec);
Intake.stop();
LeftPID(368);
FWDPID(42);
Intake.spin(reverse, 95, pct);
wait(1,sec);
Intake.stop();
FWDPID(3);



FWDPID(24);
REVPID(24);
RightPID(184);
LeftPID(184);
*/


FWDPID(24);
Intake.spin(reverse, 95, pct);
wait(1,sec);
Intake.stop();
REVPID(24);



//Programming Skills
/*
Lift.spin(forward, 95, pct);
wait(1160,msec);
Lift.stop();
shooter.spin(reverse, 65, pct);
wait(20,sec);
shooter.stop();
shooter.spin(reverse, 55, pct);
wait(20,sec);
shooter.stop();
RightPID(184);
LDrive_Group.spin(forward, 95, pct);
RDrive_Group.spin(reverse, 95, pct);
wait(5,sec);
LDrive_Group.stop();
RDrive_Group.stop();
Lift.spin(reverse, 95, pct);
wait(1360,msec);
Lift.stop();
Hanger.set(false);



Intake.spin(forward, 80, pct);
wait(500,msec);
Intake.stop();
RightPID(552);
LDrive_Group.spin(reverse, 95, pct);
RDrive_Group.spin(forward, 95, pct);
wait(5,sec);
LDrive_Group.stop();
RDrive_Group.stop();


*/

  

  



  

  


  
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

bool spinner = false;
bool revspinner = false;
bool clickState = true;

bool shoot = false;
bool Hang = false;
bool wings = false;

double Rvalue;
double Lvalue;

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Set the default velocity of the motors
    //LDrive_Group.setVelocity(50, velocityUnits::pct);
    //RDrive_Group.setVelocity(50, velocityUnits::pct);

    Rvalue = Controller1.Axis3.value() - Controller1.Axis1.value();
    Lvalue = Controller1.Axis3.value() + Controller1.Axis1.value();

     // Making the robot move
    LDrive_Group.spin(forward, Lvalue, pct);
    RDrive_Group.spin(reverse, Rvalue, pct);

    if(Controller1.Axis3.value() < 5 && Controller1.Axis1.value() < 5 && Controller1.Axis3.value() > -5 && Controller1.Axis1.value() > -5)
    {
          LDrive_Group.stop(brakeType::hold);
          RDrive_Group.stop(brakeType::hold);
    }

    //Pneumatics
        if (Controller1.ButtonX.pressing()) {
          if (!(wings)) {
            wings = true;
          }
          else if (wings) {
            wings = false;
          }

          while (Controller1.ButtonX.pressing()) {
            wait(1, msec);
          }
        }

        if(wings)
        {
            Wings1.set(true);
            Wings2.set(true);
        }
        else if(!wings)
        {
            Wings1.set(false);
            Wings2.set(false);
        }
;
    //shooter
      if (Controller1.ButtonA.pressing()) {

        if (!(shoot)) {
          shoot = true;
        }
        else if (shoot) {
          shoot = false;
        }

        while (Controller1.ButtonA.pressing()) {
          wait(1, msec);
        }
      }

      if (shoot){
        shooter.spin(reverse, 50, pct);
      } 
      else if (!(shoot)) {
        shooter.stop();
      }


      //Intake
      /*
        if (Controller1.ButtonR2.pressing())
        {
          spinner = !spinner;
          revspinner = false;
        }
        else if(Controller1.ButtonL2.pressing())
        {
          spinner = false;
          revspinner = !revspinner;
        }

        if(spinner && !revspinner)
        {
          Intake.spin(reverse, 90, pct);
        }
        else if(revspinner && !spinner)
        {
          Intake.spin(forward, 95, pct);
        }
        else
        {
          Intake.spin(reverse, 0, pct);
        }
        */
        if(Controller1.ButtonR2.pressing())
        {
          Intake.spin(reverse, 95, pct);
        }
        else if (Controller1.ButtonL2.pressing())
        {
          Intake.spin(forward, 95, pct);
        }
        else
        {
          Intake.stop(brakeType::hold);
        }


        //Lift

          if(Controller1.ButtonR1.pressing())
          {
            Lift.spin(forward, 95, pct);
          }
          else if (Controller1.ButtonL1.pressing())
          {
            Lift.spin(reverse, 95, pct);
          }
          else
          {
            Lift.stop(brakeType::hold);
          }

      //Hanger

        if (Controller1.ButtonUp.pressing()) {
          if (!(Hang)) {
            Hang = true;
          }
          else if (Hang) {
            Hang = false;
          }
          while (Controller1.ButtonUp.pressing()) {
            wait(1, msec);
          }
        }

        if (Hang){
          Hanger.set(true);
        } 
        else if (!(Hang)) {
          Hanger.set(false);
        }


}
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
Competition.autonomous(autonomous);
Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);
Lift.stop(brakeType::hold);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
