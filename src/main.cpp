#include "vex.h"
#include "9457Lib.h"
using namespace vex;

//USE THIS AS A GUIDE MOVING FORWARD, CHANGE MOTORS AND NAMES APPROPRIATELY FOR YOUR TEAM

// === Object specification ===
competition Competition;
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

rotation vDead = rotation(PORT9, false);
rotation hDead = rotation(PORT5, false);
inertial IMU = inertial(PORT3);

// === Global Library specification ===
botOdom yourRobot(&vDead, 0, &hDead, 0);
controlMotor yourMotor(&testmotor);
controlMotor yourMotorGroup(&motors);
chassis yourDB(&leftMotors, &rightMotors, &IMU);

void odomUpdate ( void ){
  odomThreadSetup(&yourRobot, &vDead, &hDead, &IMU);
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
}

void userControl( void ) {
  while( true ) {
    task::sleep(100);
  }
}

void autoControl( void ) {
  yourRobot.setPose(0, 0, 0);
}

int main() {
  pre_auton();

  Competition.drivercontrol( autoControl );
  Competition.autonomous( autoControl );
  
  //thread Odometry = thread( odomUpdate ); 

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}