#include "vex.h"
#include "9457Lib.h"
using namespace vex;

//USE THIS AS A GUIDE MOVING FORWARD, CHANGE MOTORS AND NAMES APPROPRIATELY FOR YOUR TEAM

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
motor tempMotor = motor(PORT7, ratio18_1, false);
motor lbMotor = motor(PORT6, false);              //half-motor
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

// Global storage variables
int lbState = 0;
int lbAngle[4] = { 0, 35, 160, 70 }; // (idle, holding, scoring, idle carry)
bool idleCarry = false;

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

void lbUpdate ( void ){
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(1.0, 0, .225); // create PID instance (needs to be adjusted base on manuever)
  int update_hz = 50;
  int prevState = lbState;
  double totalError = fabs(360.0);
  double currPos = 0;

  while(true) {
    if (lbState != prevState){
      totalError = fabs(lbAngle[lbState] - currPos);
      prevState = lbState;
    }

    switch(lbState){
      case (0): anglePID.adjPID(1.5, 0, .225);
      break;

      case (1): anglePID.adjPID(0.5, 0, .225);
      break;

      case (2): anglePID.adjPID(1.5, 0, .225);
      break;

      case (3): anglePID.adjPID(.4, 0, .225);
      break;
    }

    currPos = lbRot.position( deg );
    
    double error = lbAngle[lbState] - currPos;
    double pctError = error / totalError * 100;

    double toPower = anglePID.calculate( pctError );
    
    // debugging purposes - uncomment below if needed
    //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f, --%i \n" , (left->position(deg) + right->position(deg))/2, startDist+dist-tolBound, startDist+dist+tolBound, breakout);
    printf("State: %i --- currPos: %f \t Error: %f \t pctError: %f \t toPower: %f \n", lbState, currPos, error, pctError, toPower);

    lbMotor.spin( reverse, toPower, velocityUnits::pct );

    task::sleep( 1000 / update_hz );
  }
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
    
  if (Controller.ButtonL2.PRESSED){
    if (lbState == 3 && idleCarry) { // idle carry stage
      lbState = 2; 
      idleCarry = false;
      continue;
    } 

    lbState++;
    
    if (lbState > 2 ) { 
      lbState = 0;
      idleCarry = false; 
    }
  } 
  else if (Controller.ButtonLeft.PRESSED) {
    lbState = 0;
    idleCarry = false;
  }
  else if (Controller.ButtonDown.PRESSED){
    lbState = 3;
    idleCarry = true;
  }

  task::sleep(20);
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
  
  thread lbThink = thread( lbUpdate );
  //thread Odometry = thread( odomUpdate ); 

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}