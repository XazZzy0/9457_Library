#include "vex.h"
#include "9457Lib.h"
using namespace vex;

// === Object specification ===
competition Competition; 
controller Controller(primary);
brain Brain;

motor testmotor = motor(PORT5, ratio18_1, false);
motor LF = motor(PORT19, ratio6_1, true);
motor LM = motor(PORT8, ratio6_1, true);
motor LR = motor(PORT4, ratio6_1, false);
motor RF = motor(PORT18, ratio6_1, false);
motor RM = motor(PORT1, ratio6_1, true);
motor RR = motor(PORT2, ratio6_1, false);
motor_group leftMotors = motor_group( LF, LM, LR );
motor_group rightMotors = motor_group( RF, RM, RR );
motor_group motors = motor_group( LF, LM, LR, RF, RM, RR );

rotation lbRot = rotation(PORT10, true);
rotation vDead = rotation(PORT9, false);
rotation hDead = rotation(PORT5, false);
inertial IMU = inertial(PORT3);

// === Global Library specification ===
// This is how you declare these library classes, the "&" keys are references to your objects.
botOdom yourRobot(&vDead, 0, &hDead, 0);            // Creating a odom class
controlMotor yourMotor(&testmotor);                 // Creating a controlMotor class
controlMotor yourMotorGroup(&motors);               // Creating a controlMotor group class
chassis yourDB(&leftMotors, &rightMotors, &IMU);    // Creating a chassis class

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
  odomTrackCall(&yourRobot, &vDead, &hDead, &IMU);
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
  yourRobot.setVerticalDiameter(3.25);    // Set your vertical Diameter of the deadwheel
  yourRobot.setHorizontalDiameter(3.25);  // Set your vertical Diameter of the deadwheel
  yourRobot.setBotSize(16, 18);           // Set your robot size
  
  yourRobot.initializeSystem();           // initalize your system
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
  }
}

int main() {
  pre_auton();

  Competition.drivercontrol( userControl );
  Competition.autonomous( autoControl );
  
  thread Odometry = thread( odomUpdate );     // creating a thread for multi-threading.

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}
