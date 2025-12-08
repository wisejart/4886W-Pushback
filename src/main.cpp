
#include "vex.h"
#include "Robot_Config.h"

using namespace vex;
timer perrywinkle = timer();
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
#define circ ((float)(3.14) * 1.75) // circumference of wheel in inches
#define gear ((float)(36) / 48)
#define turncirc 17.25 // circumference of turning circle in inches

int TongueStatus = 1;
#define Tongueup                                                                \
  Tongue.set(false);                                                           \
  TongueStatus = 1;
#define Tonguedown                                                              \
  Tongue.set(true);                                                            \
  TongueStatus = 0;

int ScoreStatus = 1;
#define ScoreUp                                                                \
  Gate.set(true);                                                           \
  ScoreStatus = 1;
#define ScoreDown                                                              \
  Gate.set(false);                                                            \
  ScoreStatus = 0;

int FingerStatus = 1;
#define FingerUp                                                                \
  Finger.set(true);                                                           \
  FingerStatus = 1;
#define FingerDown                                                              \
  Finger.set(false);                                                            \
  FingerStatus = 0;



#define Cubed Stick15.spinToPosition(362,degrees,false);

#define Reload Stick15.spinToPosition(-1,degrees,false);      





void SetLeftVelocities(int speed) {
  Left11.setVelocity(speed, percent);
  Left12.setVelocity(speed, percent);
  Left13.setVelocity(speed, percent);
}
void SetRightVelocities(int speed) {
  Right18.setVelocity(speed, percent);
  Right19.setVelocity(speed, percent);
  Right20.setVelocity(speed, percent);
}
void SetVelocities(int speed) {
  SetLeftVelocities(speed);
  SetRightVelocities(speed);
}

void SpinForing(float dist) {
  Left11.spinFor(forward, ((float)dist * gear * 360 / circ), degrees, false);
  Left12.spinFor(forward, ((float)dist * gear * 360 / circ), degrees, false);
  Left13.spinFor(forward, ((float)dist * gear * 360 / circ), degrees, false);
  Right18.spinFor(reverse, ((float)dist * gear * 360 / circ), degrees, false);
  Right19.spinFor(reverse, ((float)dist * gear * 360 / circ), degrees, false);
  Right20.spinFor(reverse, ((float)dist * gear * 360 / circ), degrees, true);
}

void SpinAllMotorsFwd(void)
{
  Left11.spin(forward);   Left12.spin(forward);   Left13.spin(forward);
  Right18.spin(forward);   Right19.spin(forward);  Right20.spin(forward);
}

void SpinAllMotorsRev(void)
{
  Left11.spin(reverse);   Left12.spin(reverse);   Left13.spin(reverse);
  Right18.spin(reverse);   Right19.spin(reverse);  Right20.spin(reverse);
}

void StopAllMotors(void)
{
  Left11.setStopping(brake);   Left12.setStopping(brake);   Left13.setStopping(brake);
  Right18.setStopping(brake);   Right19.setStopping(brake);  Right20.setStopping(brake);
  Left11.stop();   Left12.stop();   Left13.stop();
  Right18.stop();   Right19.stop();  Right20.stop();
}



/*
void DriveDist(float dist, int speed) // dist is in inches, speed is in percent
{
  SetVelocities(speed);
  SpinForing(dist);
}
*/

#define Kp ((float)(0.3))
float Kd = 0.1;          // Derivative Gain
float prevDirnError = 0; // Last error

void DriveDist(float dist, int speed, float dirn) // dist is in inches, speed is in percent, dirn is inertial rotation in degrees
{
  SetVelocities(speed);
  float dirnError;
  float derivative;
  Left11.setPosition(0, degrees);
  Right18.setPosition(0, degrees);   // zero the motor position counters
  if (dist >0)
  {  

    while ( (Left11.position(degrees) + Right18.position(degrees))/2 < ((float)dist * gear * 360 / circ))
    {  dirnError = dirn - Inertial4.rotation(degrees);

      derivative = dirnError - prevDirnError;
      prevDirnError = dirnError;

      float correction = Kp*dirnError + Kd*derivative;

       SetLeftVelocities(speed + correction);
       SetRightVelocities(speed - correction);
       SpinAllMotorsFwd();

    }
  }
  else // going backwards
  { 
    
    while ( (Left11.position(degrees) + Right18.position(degrees))/2 > ((float)dist * gear * 360 / circ))
    {  dirnError = Inertial4.rotation(degrees) - dirn;

      derivative = dirnError - prevDirnError;
      prevDirnError = dirnError;

      float correction = Kp*dirnError + Kd*derivative;

       SetLeftVelocities(speed + correction);
       SetRightVelocities(speed - correction);
       SpinAllMotorsRev();
       wait(50,msec);
    }
  }
  StopAllMotors();

}


void TurnRobot(int degs, int speed) {
 // int InitialHeading = Inertial4.rotation(degrees);
 // int DesiredHeading = InitialHeading + degs;


  Left11.setVelocity(speed, percent);
  Left13.setVelocity(speed, percent);
  Left12.setVelocity(speed, percent);
  Right18.setVelocity(speed, percent);
  Right19.setVelocity(speed, percent);
  Right20.setVelocity(speed, percent);

  if (degs > 0) {
    Left11.spin(forward);
    Left13.spin(forward);
    Left12.spin(forward);
    Right18.spin(reverse);
    Right19.spin(reverse);
    Right20.spin(reverse);
  } else if (degs < 0) {
    Left11.spin(reverse);
    Left13.spin(reverse);
    Left12.spin(reverse);
    Right18.spin(forward);
    Right19.spin(forward);
    Right20.spin(forward);
  }

//  while ((DesiredHeading > InitialHeading &&
//          Inertial4.rotation(degrees) < DesiredHeading) ||
//         (DesiredHeading < InitialHeading &&
//          Inertial4.rotation(degrees) > DesiredHeading)) {}  

/*  Left11.stop();
  Left13.stop();
  Left12.stop();
  Right18.stop();
  Right19.stop();
  Right20.stop();  */

}

int autonSelection = 1;
int autonSet = 0;
void SelectAuton(void) {
  int press = 0;

  while (!Controller1.ButtonRight.pressing()) {
    if (Controller1.ButtonUp.pressing()) {
      if (press == 0) {
        press = 1;
        autonSelection = autonSelection + 1;
      }
    } else if (Controller1.ButtonDown.pressing()) {
      if (press == 0) {
        press = 1;
        autonSelection = autonSelection - 1;
      }

    } else // another button pressed
    {
      press = 0;
    }
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Auton =");
    Controller1.Screen.print(autonSelection);
    Controller1.Screen.setCursor(2, 1);
    switch (autonSelection) {

    case 1:
      Controller1.Screen.print("Solo AWP");
      break;
    case 2:
      Controller1.Screen.print("Elims Left");
      break;
    case 3:
      Controller1.Screen.print("Elims Right");
      break;
    case 4:
      Controller1.Screen.print("Skills");
      break;
    default:
      Controller1.Screen.print("No Auton");
      break;
    }
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("> to lock");
    wait(100, msec);

  } // while > not pressing
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("AUTON LOCKED");
  while (Controller1.ButtonRight.pressing()) {
  } // wait for button to be released
  autonSet = 1;
}






void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!

  Stick15.setVelocity(100,percent);



  SelectAuton();

  Inertial4.calibrate();
}

// All activities that occur before the competition starts
// Example: clearing encoders, setting servo positions, ...

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

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  switch (autonSelection) {

  case 1: // Solo AWP


  

    break;

  case 2: // Elims Left




    break;

  case 3: // Elims Right

    while (Inertial4.isCalibrating()) {
      wait(20, vex::msec);
    }

    break;

  case 4: // Skills

    while (Inertial4.isCalibrating()) {
      wait(20, vex::msec);
    }

    break;
   

  case 5:

  default:
    break;
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

void SetLeftMotors(int speed) {
  Left11.setVelocity(speed, percent);
  Left11.spin(forward);
  Left13.setVelocity(speed, percent);
  Left13.spin(forward);
  Left12.setVelocity(speed, percent);
  Left12.spin(forward);
}

void SetRightMotors(int speed) {
  Right18.setVelocity(speed, percent);
  Right18.spin(forward);
  Right19.setVelocity(speed, percent);
  Right19.spin(forward);
  Right20.setVelocity(speed, percent);
  Right20.spin(forward);
}

int direction = 1; // 1 = fwd, -1 = reverse


#define YawScale (float(0.8))
#define absval(arg) (arg < 0 ? -arg : arg)



int TongueButton = 0;

int ScoreButton = 0; // 0 when button isnt pressed, 1 when button is pressed

int FingerButton = 0;

void usercontrol(void) {
  // enableDrivePID = true;

  int yaw = 0;
  int Velocity = 0;



  // User control code here, inside the loop

  while (1) {

  Left11.setStopping(coast);   Left12.setStopping(coast);   Left13.setStopping(coast);
  Right18.setStopping(coast);   Right19.setStopping(coast);  Right20.setStopping(coast);


  

    Velocity = direction * Controller1.Axis1.position(percent) * YawScale;
    yaw = Controller1.Axis3.position(percent);
    SetLeftMotors(Velocity + yaw);
    SetRightMotors(Velocity - yaw);




if (Controller1.ButtonR1.pressing()) {     // INTAKE
      FIP16.spin(reverse,100,pct);
      }
    else if (Controller1.ButtonR2.pressing()) {        // OUTTAKE
      FIP16.spin(forward,100,pct);
    }    
if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing() && !Controller1.ButtonL1.pressing()) {   // INTAKE DOESN'T SPIN
        FIP16.stop();
}



    if (Controller1.ButtonA.pressing()) {     // SCORE HEIGHT
      if (ScoreButton == 0) {
        ScoreButton = 1;
        if (ScoreStatus == 0) // It's Down now
        {
          ScoreUp; // it goes up
        } else     // It's already up
        {
          ScoreDown; // makes it go down
        }
      }
    } else // L1 bit pressed let up
    {
      ScoreButton = 0; // set 0 so loop recognized when pressed
    }



if (Controller1.ButtonL1.pressing()) { // Stick Goes Up to Push in Cubes
  FIP16.spin(reverse,100,pct);
  Cubed; 
}
    if (!Controller1.ButtonL1.pressing()) { // Stick Returns to Starting Position
  Reload;
}



    if (Controller1.ButtonY.pressing()) {     // TONGUE
      if (TongueButton == 0) {
        TongueButton = 1;
        if (TongueStatus == 0) // It's Down now
        {
          Tongueup; // it goes up
        } else     // It's already up
        {
          Tonguedown; // makes it go down
        }
      }
    } else // L1 bit pressed let up
    {
      TongueButton = 0; // set 0 so loop recognized when pressed
    }



    if (Controller1.ButtonX.pressing()) {     // FINGER
      if (FingerButton == 0) {
        FingerButton = 1;
        if (FingerStatus == 0) // It's Down now
        {
          FingerUp; // it goes up
        } else     // It's already up
        {
          FingerDown; // makes it go down
        }
      }
    } else // L1 bit pressed let up
    {
      FingerButton = 0; // set 0 so loop recognized when pressed
    }


  



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
