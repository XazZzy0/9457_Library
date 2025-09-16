#include "9457Lib.h"

using namespace vex;

// === Object specification ===
competition Competition; 
controller Controller(primary);
brain Brain;

motor testmotor = motor(PORT18, ratio18_1, false);
motor LF = motor(PORT16, ratio6_1, true);
motor LM = motor(PORT6, ratio6_1, true);
motor LR = motor(PORT5, ratio6_1, false);
motor RF = motor(PORT14, ratio6_1, false);
motor RM = motor(PORT4, ratio6_1, false);
motor RR = motor(PORT3, ratio6_1, true);
motor_group leftMotors = motor_group( LF, LM, LR );
motor_group rightMotors = motor_group( RF, RM, RR );

rotation testRot = rotation(PORT19, true);
rotation vDead = rotation(PORT20, false);
rotation hDead = rotation(PORT21, false);
inertial IMU = inertial(PORT2);

// === Global Library specification ===
// This is how you declare these library classes, the "&" keys are references to your objects.
chassis DB(&leftMotors, &rightMotors, &IMU, systemHz);    // Creating a chassis class
botOdom Robot(&vDead, 0, &hDead, 0, &IMU, systemHz);      // Creating a odom class with 2 Offset Deadwheels and an IMU
controlMotor PIDMotor(&testmotor, systemHz);              // Creating a controlMotor class

// === Global storage variables ===
TEAMCOLOR setColor = emptyColor;                    // Set your Color (RED, BLUE)
AUTONSET setAuton = emptyAuton;                     // Set your Auton (LEFT, RIGHT, SKILLS)

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

// This is a smaller custom function for an update callback

void odomUpdate ( void ){
  odomTrackCall(&Robot, &vDead, &hDead, &IMU, false);
}

/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗     ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║
██████╔╝██║   ██║██████╔╝██║   ██║   ██║       ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║       ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║       ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝        ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝

*/

// Pre-autonomous intial setups
void pre_auton ( void ){
  Robot.setVerticalDiameter(3.25);    // Set your vertical Diameter of the deadwheel
  Robot.setHorizontalDiameter(3.25);  // Set your vertical Diameter of the deadwheel
  Robot.setBotSize(16, 18);           // Set your robot size
  
  Robot.initializeSystem();           // initalize your odometry system -> FEEDBACK
  DB.initialize();                    // initalize your chassis system -> PLANT
}

void userControl( void ) {
  while( true ) {
    // do nothing
  }
}

void autoControl( void ) {
  switch ( setAuton )
  {
  case RIGHT:
    if( setColor == RED ){
      /* RED LEFT code */
    }
    else if ( setColor == BLUE ) {
      /* BLUE LEFT code */
    }  
    else { return; }
    break;
  
  case LEFT:
    if( setColor == RED ){
      /* RED LEFT code */
    }
    else if ( setColor == BLUE ) {
      /* BLUE LEFT code */
    }  
    else { return; }
    break;

  case SKILLS:
    /* SKILLS code */
    break;

  default:
    /* Do nothing */
    break;
  }
}

int main() {
  pre_auton();

  PIDMotor.setBrake();
  PIDMotor.setPID(0.55, 0.0, 0.0);
  PIDMotor.pidRotate(180, 30, 0);

  Competition.drivercontrol( userControl );
  Competition.autonomous( autoControl );
  
  //thread Odometry = thread( odomUpdate );     // creating a thread for multi-threading.

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}
