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
botOdom yourRobot(&vDead, 0, &hDead, 0);
controlMotor yourMotor(&testmotor);
controlMotor yourMotorGroup(&motors);
chassis yourDB(&leftMotors, &rightMotors, &IMU);

// === Global storage variables ===
double tempVar = 0;

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

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
  yourRobot.setVerticalDiameter(3.25);
  yourRobot.setHorizontalDiameter(3.25);
  yourRobot.setBotSize(16, 18);
  
  yourRobot.initializeSystem();
  lbRot.resetPosition();
}

void userControl( void ) {
  while( true ) {
    // do nothing
  }
}

void autoControl( void ) {
  //initalize
  yourRobot.setPose(0, 0, 0);
  yourDB.initialize();

  //move
  yourDB.setDrivePID(2.55, 0, .225);
  yourDB.driveAccel(9500, 100, 15.0, 3.0);
  yourDB.pointTurn(175, 50);
  yourDB.driveAccel(9500, 100, 15.0, 3.0);  
}

int main() {
  pre_auton();

  Competition.drivercontrol( userControl );
  Competition.autonomous( autoControl );
  
  thread Odometry = thread( odomUpdate ); 

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}
