#include "9457Lib/chassis.h"

using namespace vex; // using namespace vex helps to type less

/*
███████╗███╗   ███╗ █████╗ ██████╗ ████████╗    ██████╗ ██████╗ ██╗██╗   ██╗███████╗
██╔════╝████╗ ████║██╔══██╗██╔══██╗╚══██╔══╝    ██╔══██╗██╔══██╗██║██║   ██║██╔════╝
███████╗██╔████╔██║███████║██████╔╝   ██║       ██║  ██║██████╔╝██║██║   ██║█████╗
╚════██║██║╚██╔╝██║██╔══██║██╔══██╗   ██║       ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝
███████║██║ ╚═╝ ██║██║  ██║██║  ██║   ██║       ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗
╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝       ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝

Currently in work. Relative coordinate system.
working to implement motor plug-ins and more advanced manuevers.
Needs Odom to work as of the moment
*/

chassis::chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, const double sysRate ):
    leftDB(leftGroup), rightDB(rightGroup), IMU(nullptr), updateRate(sysRate)
    {}

chassis::chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, vex::inertial *botIMU, const double sysRate ) : 
    leftDB(leftGroup), rightDB(rightGroup), IMU(botIMU), updateRate(sysRate)
    {}

// Setting the brake type of the chassis
void chassis::setBrake( brakeType type ) {
  switch (type) {
    case brakeType::coast:
      leftDB->setStopping(coast);
      rightDB->setStopping(coast);
      break;
    case brakeType::brake: 
      leftDB->setStopping(brake);
      rightDB->setStopping(brake);
      break;
   case brakeType::hold:
      leftDB->setStopping(hold);
      rightDB->setStopping(hold);
      break;
  } 
}

void chassis::setODOM(botOdom *botODOM) { ODOM = botODOM; }

void chassis::initialize( void ){
  leftDB->resetPosition();
  rightDB->resetPosition();
  this->setBrake(brakeType::brake);
}

void chassis::setDrivePID( double pTerm, double iTerm, double dTerm ){
  drivePID[0] = pTerm;
  drivePID[1] = iTerm;
  drivePID[2] = dTerm;
}

void chassis::setTurnPID( double pTerm, double iTerm, double dTerm ){
  turnPID[0] = pTerm;
  turnPID[1] = iTerm;
  turnPID[2] = dTerm;
}

void chassis::testSpin(){
  leftDB->spin(fwd, 25, pct);
  rightDB->spin(fwd, 25, pct);
}

void chassis::driveFwd( double dist, double maxVel, double minVel, int breakoutCount, bool waitCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(drivePID[0], drivePID[1], drivePID[2]); // create PID instance
  int update_hz = updateRate;
  double tolBound = 10, tolBound_in = .25; 
  int breakout = 0;
  
  double startDist = (leftDB->position(deg) + rightDB->position(deg))/2;     // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running 
  double currDist = 0;                                                   // initialize Encoder holder variable                            
  double totalError = fabs( dist );                                      // initialize absolute total error of manuever
  double error = dist;                                                   // initialize error
  double toPower;                                                        // initialize the speed variable
  double pctError = error / totalError * 100;                            // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response

 if (ODOM) { startDist = ODOM->vWheel->position(deg);}

  while(breakout < breakoutCount) {
    if (ODOM){  // dist will use inches
      currDist = (ODOM->vWheel->position(deg) - startDist)/360 * (M_PI*ODOM->vWheel_Diameter) ;
      
      error = dist - currDist;
      pctError = error / totalError * 100;

      toPower = anglePID.calculate( pctError );
      
      // debugging purposes - uncomment below if needed
      //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f, --%i \n" , (leftDB->position(deg) + rightDB->position(deg))/2, startDist+dist-tolBound, startDist+dist+tolBound, breakout);
      //printf("Point turn --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);

      leftDB->spin( fwd, toPower, velocityUnits::pct );
      rightDB->spin( fwd, toPower, velocityUnits::pct );

      task::sleep( 1000 / update_hz );

      if (currDist >=   dist - tolBound_in && currDist <=  dist + tolBound_in) { ++breakout; }
      else { breakout = 0; }
    }
    else{       // dist will use degrees
      currDist = (leftDB->position(deg) + rightDB->position(deg))/2;
      
      error = (startDist + dist) - currDist;
      pctError = error / totalError * 100;

      toPower = anglePID.calculate( pctError );
      
      // debugging purposes - uncomment below if needed
      //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f, --%i \n" , (leftDB->position(deg) + rightDB->position(deg))/2, startDist+dist-tolBound, startDist+dist+tolBound, breakout);
      //printf("driveFwd --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currDist, error, pctError, toPower);

      leftDB->spin( fwd, toPower, velocityUnits::pct );
      rightDB->spin( fwd, toPower, velocityUnits::pct );

      task::sleep( 1000 / update_hz );

      if (currDist >=  (startDist + dist) - tolBound && currDist <= (startDist + dist) + tolBound) { ++breakout; }
      else { breakout = 0; }
    }
  }

  leftDB->stop();
  rightDB->stop();
  return;
}

void chassis::driveAccel( double dist, double vel, double accelPeriod, double minVel, int breakoutCount, bool waitCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(drivePID[0], drivePID[1], drivePID[2]); // create PID instance
  int update_hz = updateRate;
  double tolBound = 10; 
  int breakout = 0;
  
  double startDist = (leftDB->position(deg) + rightDB->position(deg))/2;     // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running 
  double currDist = 0;                                                   // initialize Encoder holder variable                            
  double totalError = fabs( dist );                                      // initialize absolute total error of manuever
  double error = dist;                                                   // initialize error
  double toPower;                                                        // initialize the speed variable
  double pctError = error / totalError * 100;                            // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  anglePID.setVel(minVel, vel);         // Set the Minimum and Maximum Velocity of the response

  while(breakout < breakoutCount) {
    if (ODOM){  // dist will use inches
    
    }
    else{       // dist will use degrees
      currDist = (leftDB->position(degrees) + rightDB->position(degrees))/2;    // grabs current position 
      error =  (startDist + dist) - currDist;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (!signbit(pctError)){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (vel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (vel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
      //printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      leftDB->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)
      rightDB->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currDist >= (startDist + dist) - tolBound) && (currDist <= (startDist + dist) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
  }

  leftDB->stop();
  rightDB->stop();
  return;
}

void chassis::pointTurn( double angle, double vel, double minVel, int breakoutCount, bool waitForCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(turnPID[0], turnPID[1], turnPID[2]); // create PID instancey
  int update_hz = updateRate;
  double tolBound = 3; 
  int breakout = 0;
  
  double startAngle = IMU->rotation( degrees );     // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running 
  double currAngle = 0;                             // initialize IMU holder variable                            
  double totalError = fabs( angle );                // initialize absolute total error of manuever
  double error = angle;                             // initialize error
  double toPower;                                   // initialize the speed variable
  double pctError = error / totalError * 100;       // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  anglePID.setVel(minVel, vel);         // Set the Minimum and Maximum Velocity of the response                    

  while( breakout < breakoutCount ) {
    currAngle = IMU->rotation( degrees );
    error = (startAngle + angle) - currAngle;
    pctError = error / totalError * 100;

    toPower = anglePID.calculate( pctError );
    
    // debugging purposes - uncomment below if needed
    //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f \n" , IMU->rotation( degrees ), startAngle+angle-tolBound, startAngle+angle+tolBound);
    //printf("Point turn --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);

    leftDB->spin( fwd, toPower, velocityUnits::pct );
    rightDB->spin( reverse, toPower, velocityUnits::pct );

    task::sleep( 1000 / update_hz );

    if (currAngle >=  (startAngle + angle) - tolBound && currAngle <= (startAngle + angle) + tolBound) { ++breakout; }
    else { breakout = 0; }
  }

  leftDB->stop();
  rightDB->stop();
  return;
}

void chassis::swingTurn( turnType dir, double toTic, double vel, bool waitForCompletion ){ // Example of a differential swing turn, default units for is motor 
  double ticSum;   // initialize counter
  vel = fabs(vel); // makes positive velocity standard

    if (dir == turnType::left) {
      double startTic = rightDB->position(deg);
      do {
        ticSum = rightDB->position(deg);
        if (!signbit(toTic)){ // positive
          rightDB->spin( fwd, vel, velocityUnits::pct );
          leftDB->stop();
        }
        else { //negative
          rightDB->spin( reverse, vel, velocityUnits::pct );
          leftDB->stop();
          }

        //printf("ticSum: %f, startTic: %f, toTic: %f \n", ticSum, startTic, toTic);
      }  
      while(ticSum <= startTic + toTic);
    }  
    else if (dir == turnType::right) {
      double startTic = leftDB->position(deg);
      do{
        ticSum = leftDB->position(deg);
        if (!signbit(toTic)){ // positive
          leftDB->spin( fwd, vel, velocityUnits::pct );
          rightDB->stop();
        }
        else { //negative
          leftDB->spin( reverse, vel, velocityUnits::pct );
          rightDB->stop();
          }

        //printf("ticSum: %f, startTic: %f, toTic: %f \n", ticSum, startTic, toTic);
      }
      while(ticSum <= startTic + toTic);
    }
    else { return; }
  

 leftDB->stop();
 rightDB->stop();
 return;
}

void chassis::diffDrive( double leftPower, double rightPower, double tics){
  double ticSum;   // initialize counter

  leftDB->stop();
  rightDB->stop();
  return;
}


void chassis::arcadeDrive( controller *Controller, float deadband ) { // Standard Arcade Drive
  float throttle, turn, outputL, outputR;
  if ( fabs(Controller->Axis3.position()) >= deadband ) { throttle = Controller->Axis3.position()/100.0; }
  else { throttle = 0; }
  if ( fabs(Controller->Axis1.position()) >= deadband ) { turn = Controller->Axis1.position()/100.0; }
  else { turn = 0; }

  outputL = throttle + turn;
  outputR = throttle - turn;

  if (outputL > 1)       { outputL = 1.0; }
  else if (outputL < -1) { outputL = -1.0; }
  if (outputR > 1)       { outputR = 1.0; }
  else if (outputR < -1) { outputR = -1.0; }
 
  if( fabs(Controller->Axis3.position()) >= deadband || fabs(Controller->Axis1.position()) >= deadband ){
     leftDB->spin(fwd, 12*outputL, volt);
     rightDB->spin(fwd, 12*outputR, volt); 
  } else {
    leftDB->stop();
    rightDB->stop(); 
  }
}

void chassis::arcadeDrive( controller *Controller, float spline, float deadband ){ // accel curve code (spline < .50 = decel response, .50 = linear response, > .50 = accel response)
  float startY = 0, endY = 1; 
  float throttle, turn, outputL, outputR;
  if ( fabs(Controller->Axis3.position()) >= deadband ) { throttle = Controller->Axis3.position()/100.0; }
  else { throttle = 0; }
  if ( fabs(Controller->Axis1.position()) >= deadband ) { turn = Controller->Axis1.position()/100.0; }
  else { turn = 0; }
   
  float newTurn = pow(1-turn,2)*startY + (1-turn)*turn*spline + pow(turn, 2)*endY;

  if (!signbit(Controller->Axis1.position())){
    outputL = throttle + newTurn;
    outputR = throttle - newTurn;
  } else {
    outputL = throttle - newTurn;
    outputR = throttle + newTurn;
  }
  

  if (outputL > 1.0)       { outputL = 1.0; }
  else if (outputL < -1.0) { outputL = -1.0; }
  if (outputR > 1.0)       { outputR = 1.0; }
  else if (outputR < -1.0) { outputR = -1.0; }

  if( fabs(Controller->Axis3.position()) >= deadband || fabs(Controller->Axis1.position()) >= deadband ){
     leftDB->spin(fwd, 12*outputL, volt);
     rightDB->spin(fwd, 12*outputR, volt); 
  } else {
    leftDB->stop();
    rightDB->stop(); 
  }
}

void chassis::tankDrive( controller *Controller, float deadband) {
  float throttleL, throttleR;
  if ( fabs(Controller->Axis3.position()) >= deadband ) { throttleL = Controller->Axis3.position()/100; }
  else { throttleL = 0.0; }
  if ( fabs(Controller->Axis2.position()) >= deadband ) { throttleR = Controller->Axis2.position()/100; }
  else { throttleR = 0.0; }
  leftDB->spin(fwd, 12*throttleL, volt);
  rightDB->spin(fwd, 12*throttleR, volt);
}