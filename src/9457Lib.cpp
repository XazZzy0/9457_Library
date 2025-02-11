#include "9457Lib.h"

/*
██████╗ ██╗██████╗      ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗     ███████╗██████╗
██╔══██╗██║██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║     ██╔════╝██╔══██╗
██████╔╝██║██║  ██║    ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║     █████╗  ██████╔╝
██╔═══╝ ██║██║  ██║    ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ██║     ██╔══╝  ██╔══██╗
██║     ██║██████╔╝    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗███████╗███████╗██║  ██║
╚═╝     ╚═╝╚═════╝      ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝

Self Explanatory
*/
// Initialize outside of class definition
PID::PID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}

// Calculate the PID response (expects an error input, in degrees or pct manuever)
double PID::calculate( double error ) {
  Pterm = Kp * error;               // This is the P response - acts like a spring
  Iterm = Ki * integral;            // This is the I response - pushes error to 0 over time.
  Dterm = Kd * (error - prevError); // This is the D response - acts like a Dampener

 // The I variable sometimes has issues if it runs a command for a long time, here are some fixes which mitigate the I term from impacting too much:
 // Integral Fix 1: only count up when the error is less than the start I term;
 if (fabs(error) < initI)  { integral += error; }           // this is the I summation - totals the error over time
 // Integral Fix 2: Preventing integral windup - when the integral term gets too high and the system cannot respond to the next manuever because of the Iterm.
 if (integral > windup)      { integral = windup; }
 else if (integral < windup) { integral = -windup; }
 // Integral Fix 3: Setting integral to 0 when error crosses zero - reduce the impact of the integral term.
 if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0))  { integral = 0; }
 
  double output = Pterm + Iterm + Dterm; // Sum all of the values - the PID response

  // clamps the output
  if (output > maxPower)          { output = maxPower; }
  else if (output < -maxPower)    { output = -maxPower; }

  // clamps the min speed if there is one
  if (minPower != 0){
    if (fabs(output) < minPower){
      if (signbit(output)) {output = -minPower;}
      else                 {output = minPower;}
    }
  }

  // sets the previous error to the new one
  prevError = error;

  // return the PID output 
  return output;
}

// Reset the PID internal variables
void PID::reset( void ){
  integral = 0;
  prevError = 0;
  return;
}

// Adjust the PID coefficients
void PID::adjPID(double kP, double kI, double kD){
  Kp = kP;
  Ki = kI;
  Kd = kD;
  return;
}

//adjust the windup behavior of the PID
void PID::adjWindup( double wTerm ) { windup = wTerm; }

//set the maximum and the minimum velocity output of the pid
void PID::setVel(double toMinPower, double toMaxPower){
  if (fabs(toMaxPower) > 100)  { toMaxPower = 100; } // Max speed is 100 pct
  if (fabs(toMinPower) > 100)  { toMinPower = 100; } // Min speed is 100 pct - don't do this please
  if (toMinPower > toMaxPower) { return; } 

  minPower = fabs(toMinPower); // No negative signs for power, standard is positive
  maxPower = fabs(toMaxPower); // No negative signs for power, standard is positive
  return;
}

/*
 ██████╗ ██████╗  ██████╗ ███╗   ███╗███████╗████████╗██████╗ ██╗   ██╗     ██████╗██╗      █████╗ ███████╗███████╗
██╔═══██╗██╔══██╗██╔═══██╗████╗ ████║██╔════╝╚══██╔══╝██╔══██╗╚██╗ ██╔╝    ██╔════╝██║     ██╔══██╗██╔════╝██╔════╝
██║   ██║██║  ██║██║   ██║██╔████╔██║█████╗     ██║   ██████╔╝ ╚████╔╝     ██║     ██║     ███████║███████╗███████╗
██║   ██║██║  ██║██║   ██║██║╚██╔╝██║██╔══╝     ██║   ██╔══██╗  ╚██╔╝      ██║     ██║     ██╔══██║╚════██║╚════██║
╚██████╔╝██████╔╝╚██████╔╝██║ ╚═╝ ██║███████╗   ██║   ██║  ██║   ██║       ╚██████╗███████╗██║  ██║███████║███████║
 ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝     ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝        ╚═════╝╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝

Self Explanatory
*/

// Empty Odom class - for testing
botOdom::botOdom( void ) :  
  xG(0), yG(0), tG(0)
  {} 

// CASE 2 - 2 DW (perpedicular to eachother), 1 IMU
botOdom::botOdom( rotation *vertWheel, double offset_V, rotation *horWheel, double offset_H ) :  
  vOffset(offset_V), hOffset(offset_H),  
  vWheel(vertWheel), hWheel(horWheel),
  xG(0), yG(0), tG(0)
  {} 

void botOdom::initializeSystem( void ){ // used to completely set the odom class for start
  if (IMU){
    IMU->calibrate( int32_t(1.5) );
    while(IMU->isCalibrating()) { task::sleep(100); }

    IMU->setRotation(0, degrees);
  
    if (vWheel) { vWheel->resetPosition(); }
    if (vLWheel) { vLWheel->resetPosition(); }
    if (vRWheel) { vRWheel->resetPosition(); }
    if (hWheel) { hWheel->resetPosition(); }

    if (vWheel_Diameter == 0) { printf("VERTICAL WHEEL DIAMETER NEEDS TO BE SET FOR ODOM TO CALIBRATE PROPERLY \n"); }
    if (hWheel_Diameter == 0) { printf("HORIZONTAL WHEEL DIAMETER NEEDS TO BE SET TO CALIBRATE PROPERLY \n"); }
    //if (baseWidth == 0) { printf("BASE WIDTH NEEDS TO BE SET TO CALIBRATE PROPERLY\n"); }
    //if (baseLength == 0) { printf("BASE LENGTH NEEDS TO BE SET TO CALIBRATE PROPERLY\n"); }

    xG = 0, yG = 0, tG = 0;

    isCalibrated = true; // init passed
  }
  else{ // initialization failed - NO IMU
    xG = 0, yG = 0, tG = 0;
    if (vWheel) { vWheel->resetPosition(); }
    if (hWheel) { hWheel->resetPosition(); }
      
    printf("THERE IS NO IMU PRESENT - ODOMETRY CLASS WILL NOT WORK \n");
  }
}

void botOdom::setVerticalDiameter( double Diameter )    { vWheel_Diameter = Diameter; }
void botOdom::setHorizontalDiameter( double Diameter)   { hWheel_Diameter = Diameter; }
void botOdom::setBotSize( double width, double length ) { baseWidth = width, baseLength = length; }

// Declare your global location 
void botOdom::setPose ( double xPose, double yPose, double tPose ) {
  xG = xPose;
  yG = yPose;
  tG = tPose;
  if (IMU) { IMU->setRotation(tPose, degrees); }
}

// Declare the update rate of the system. CANNOT GO ABOVE 100 HZ
void botOdom::setRate ( double rate_hz ) { 
  if (rate_hz > 100)     { rate_hz = 100; }
  else if (rate_hz < 20) { rate_hz = 20; }
  else                   { update_hz = rate_hz; }
}

/* --- SYSTEM UPDATE 1 FUNCTION ---
THIS IS HOW TO UPDATE THE ROBOT FRAME SO THAT THE X, Y, AND HEADING
OF THE ROBOT IS ACCURATELY REPRESENTED.

ONLY USE WHEN 2 DEADWHEELS AND AN IMU ARE PRESENT, A VERTICAL ONE AND A 
HORIZONTAL ONE,

INPUTS:
vWheel           -> Vertical Deadwheel position reading [degrees]
hWheel           -> Horizontal Deadwheel position reading [degrees]
angle            -> Totaled IMU Reading -Inf - Inf [degrees]
vWheel_Diameter  -> Vertical Deadwheel Diameter (optional input)
hWheel_Diameter  -> Horizontal Deadwheel Diameter (optional input) 
*/
void botOdom::trackLocation( double vWheel, double hWheel, double angle ) {
  tdot = (angle*DEG2RAD) - tPrev;                          // change in heading from the previous position.  [Radians/hz]
  vdot = (vWheel - vPrev)/360*PI*vWheel_Diameter;            // change in the x location from the previous position. [in/hz]
  hdot = (hWheel - hPrev)/360*PI*vWheel_Diameter;            // change in the y location from the previous position. [in/hz]
  if (tdot == 0) {tdot = 1E-8;};                             // prevent divide by zero errors 

  // All the following calculations are taken from the Pilons position tracking document
  double lx = 2*sin(tdot/2)*(hdot/tdot + hOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
  double ly = 2*sin(tdot/2)*(vdot/tdot + vOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
                                                                                
  xG += (ly*cos(tG + tdot/2)) + (lx*sin(tG + tdot/2));     // update Inertial Frame [Inches] - aka field frame
  yG += (ly*sin(tG + tdot/2)) - (lx*cos(tG + tdot/2));     // update Inertial Frame [Inches] - aka field frame
  tG += tdot*RAD2DEG; 

  if      (tG > 360) { tG = 0; }    // checks to see if the global is outside of normal bounds (over-travel)
  else if (tG < 0)   { tG = 360; }  // checks to see if the global is outside of normal bounds (under-travel)

  // update the previous positions to the current positions for the next iteration
  vPrev = vWheel; 
  hPrev = hWheel;
  tPrev = angle;
} 


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

chassis::chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup ):
    left(leftGroup), right(rightGroup), IMU(nullptr)
    {}

chassis::chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, vex::inertial *botIMU ) : 
    left(leftGroup), right(rightGroup), IMU(botIMU)
    {}

// Setting the brake type of the chassis
void chassis::setBrake( int type ) {
  switch (type) {
    case 0:
      left->setStopping(coast);
      right->setStopping(coast);
      break;
    case 1: 
      left->setStopping(brake);
      right->setStopping(brake);
      break;
   case 2: 
      left->setStopping(hold);
      right->setStopping(hold);
      break;
  } 
}

void chassis::setODOM(botOdom *botODOM) { ODOM = botODOM; }

void chassis::initialize( void ){
  left->resetPosition();
  right->resetPosition();
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

void chassis::driveFwd( double dist, double maxVel, double minVel, int breakoutCount, bool waitCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(drivePID[0], drivePID[1], drivePID[2]); // create PID instance
  int update_hz = updateRate;
  double tolBound = 10, tolBound_in = .25; 
  int breakout = 0;
  
  double startDist = (left->position(deg) + right->position(deg))/2;     // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running 
  double currDist = 0;                                                   // initialize Encoder holder variable                            
  double totalError = fabs( dist );                                      // initialize absolute total error of manuever
  double error = dist;                                                   // initialize error
  double toPower;                                                        // initialize the speed variable
  double pctError = error / totalError * 100;                            // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response

 if (ODOM) { startDist = ODOM->vWheel->position(deg);}

  while(breakout < breakoutCount) {
    if (ODOM){  // dist will use inches
      currDist = (ODOM->vWheel->position(deg) - startDist)/360 * (PI*ODOM->vWheel_Diameter) ;
      
      error = dist - currDist;
      pctError = error / totalError * 100;

      toPower = anglePID.calculate( pctError );
      
      // debugging purposes - uncomment below if needed
      //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f, --%i \n" , (left->position(deg) + right->position(deg))/2, startDist+dist-tolBound, startDist+dist+tolBound, breakout);
      //printf("Point turn --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);

      left->spin( fwd, toPower, velocityUnits::pct );
      right->spin( fwd, toPower, velocityUnits::pct );

      task::sleep( 1000 / update_hz );

      if (currDist >=   dist - tolBound_in && currDist <=  dist + tolBound_in) { ++breakout; }
      else { breakout = 0; }
    }
    else{       // dist will use degrees
      currDist = (left->position(deg) + right->position(deg))/2;
      
      error = (startDist + dist) - currDist;
      pctError = error / totalError * 100;

      toPower = anglePID.calculate( pctError );
      
      // debugging purposes - uncomment below if needed
      //printf("currROT: %.2f, targetROT(lower): %.2f, (upper): %.2f, --%i \n" , (left->position(deg) + right->position(deg))/2, startDist+dist-tolBound, startDist+dist+tolBound, breakout);
      //printf("Point turn --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);

      left->spin( fwd, toPower, velocityUnits::pct );
      right->spin( fwd, toPower, velocityUnits::pct );

      task::sleep( 1000 / update_hz );

      if (currDist >=  (startDist + dist) - tolBound && currDist <= (startDist + dist) + tolBound) { ++breakout; }
      else { breakout = 0; }
    }
  }

  left->stop();
  right->stop();
  return;
}

void chassis::driveAccel( double dist, double vel, double accelPeriod, double minVel, int breakoutCount, bool waitCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(drivePID[0], drivePID[1], drivePID[2]); // create PID instance
  int update_hz = updateRate;
  double tolBound = 10; 
  int breakout = 0;
  
  double startDist = (left->position(deg) + right->position(deg))/2;     // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running 
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
      currDist = (left->position(degrees) + right->position(degrees))/2;    // grabs current position 
      error =  (startDist + dist) - currDist;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (vel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (vel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
      //printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      left->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)
      right->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currDist >= (startDist + dist) - tolBound) && (currDist <= (startDist + dist) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
  }

  left->stop();
  right->stop();
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

    left->spin( fwd, toPower, velocityUnits::pct );
    right->spin( reverse, toPower, velocityUnits::pct );

    task::sleep( 1000 / update_hz );



    if (currAngle >=  (startAngle + angle) - tolBound && currAngle <= (startAngle + angle) + tolBound) { ++breakout; }
    else { breakout = 0; }
  }

  left->stop();
  right->stop();
  return;
}


void chassis::swingTurn( double rVel, double lVel, int runTime_ms){ // Example of a differential swing turn, default units for time is ms - change velocity for response.
  int elapsedTime = 0;
 
  do{
    right->spin( fwd, rVel, velocityUnits::pct );
    left->spin( fwd, lVel, velocityUnits::pct );
    task::sleep( 1000/ updateRate );
    elapsedTime += 1000/updateRate;
  }
  while (elapsedTime < runTime_ms);

 left->stop();
 right->stop();
 return;
}


void chassis::arcadeDrive( controller *Controller, float deadband ) {
  float throttle, turn, outputL, outputR;
  if ( fabs(Controller->Axis3.value()) >= deadband ) { throttle = Controller->Axis3.value()/100; }
  else { throttle = 0; }
  if ( fabs(Controller->Axis1.value()) >= deadband ) { turn = Controller->Axis1.value()/100; }
  else { turn = 0; }

  outputL = throttle + turn;
  outputR = throttle - turn;

  if (outputL > 1)       { outputL = 1.0; }
  else if (outputL < -1) { outputL = -1.0; }
  if (outputR > 1)       { outputR = 1.0; }
  else if (outputR < -1) { outputR = -1.0; }
 
  left->spin(fwd, 12*outputL, volt);
  right->spin(fwd, 12*outputR, volt);
}

void chassis::arcadeDrive( controller *Controller, float spline, float deadband ){ // accel curve code (spline < .50 = decel response, .50 = linear response, > .50 = accel response)
  float startY = 0, endY = 1; 
  float throttle, turn, outputL, outputR;
  if ( fabs(Controller->Axis3.value()) >= deadband ) { throttle = Controller->Axis3.value()/100; }
  else { throttle = 0; }
  if ( fabs(Controller->Axis1.value()) >= deadband ) { turn = Controller->Axis1.value()/100; }
  else { turn = 0; }
   
  float newTurn = pow(1-turn,2)*startY + (1-turn)*turn*spline + pow(turn, 2)*endY;
 
  outputL = throttle + newTurn;
  outputR = throttle - newTurn;

  if (outputL > 1)       { outputL = 1.0; }
  else if (outputL < -1) { outputL = -1.0; }
  if (outputR > 1)       { outputR = 1.0; }
  else if (outputR < -1) { outputR = -1.0; }
 
  left->spin(fwd, 12*outputL, volt);
  right->spin(fwd, 12*outputR, volt);
}

void chassis::tankDrive( controller *Controller, float deadband) {
  float throttleL, throttleR;
  if ( fabs(Controller->Axis3.value()) >= deadband ) { throttleL = Controller->Axis3.value()/100; }
  else { throttleL = 0; }
  if ( fabs(Controller->Axis2.value()) >= deadband ) { throttleR = Controller->Axis2.value()/100; }
  else { throttleR = 0; }
  left->spin(fwd, 12*throttleL, volt);
  right->spin(fwd, 12*throttleR, volt);
}

/*
███████╗███╗   ███╗ █████╗ ██████╗ ████████╗    ███╗   ███╗ ██████╗ ████████╗ ██████╗ ██████╗
██╔════╝████╗ ████║██╔══██╗██╔══██╗╚══██╔══╝    ████╗ ████║██╔═══██╗╚══██╔══╝██╔═══██╗██╔══██╗
███████╗██╔████╔██║███████║██████╔╝   ██║       ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝
╚════██║██║╚██╔╝██║██╔══██║██╔══██╗   ██║       ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔══██╗
███████║██║ ╚═╝ ██║██║  ██║██║  ██║   ██║       ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║  ██║
╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝       ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝  ╚═╝

*/


controlMotor::controlMotor( vex::motor *ptrMotor ) : 
  refMotor(ptrMotor), refGroup(nullptr), refEncoder(nullptr) 
  {}

controlMotor::controlMotor( vex::motor *ptrMotor, vex::rotation *ptrRot) :
  refMotor(ptrMotor), refGroup(nullptr), refEncoder(ptrRot)
  {}

controlMotor::controlMotor( vex::motor_group *ptrGroup, vex::rotation *ptrRot) :
  refMotor(nullptr), refGroup(ptrGroup), refEncoder(ptrRot)
  {}

controlMotor::controlMotor( vex::motor_group *ptrGroup ) :
  refMotor(nullptr), refGroup(ptrGroup), refEncoder(nullptr)
  {}

// Setting the brake type of the chassis
void controlMotor::setBrake( int type ) {
 if (refGroup){
    switch (type) {
      case 0:
        refGroup->setStopping(coast);
        break;
      case 1: 
        refGroup->setStopping(brake);
        break;
     case 2: 
        refGroup->setStopping(hold);
        break;
     } 
 }
 else if (refMotor){
    switch (type) {
      case 0:
        refMotor->setStopping(coast);
        break;
      case 1: 
        refMotor->setStopping(brake);
        break;
     case 2: 
        refMotor->setStopping(hold);
        break;
     } 
 }
}

void controlMotor::setPID( double pTerm, double iTerm, double dTerm ) {
  PID_Coef[0] = pTerm;
  PID_Coef[1] = iTerm;
  PID_Coef[2] = dTerm;
}

void controlMotor::setRate( double rate_hz ){
  if (rate_hz > 100)     { updateRate = 100; }
  else if (rate_hz < 20) { updateRate = 20; }
  else                   { updateRate = rate_hz; }
}

void controlMotor::testSpin( void ){
  if (!refGroup) {
    refMotor->spin(vex::forward, 100, vex::velocityUnits::pct);
  } 
  else{
    refGroup->spin(vex::forward, 100, vex::velocityUnits::pct);
  }
  
}

void controlMotor::pidRotate( double target, double maxVel, double minVel, int breakoutCount ){
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);    // set PID response
  int update_hz = updateRate;                             // set the update rate;
  double tolBound = 5;                                    // tic offset bound of the target -> larger = easier to breakout
  int breakout = 0;                                       // initialized the breakout variable
  
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target);                  // initialize absolute total error of manuever
  double error = target;                             // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)
  
  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response
  
  if ( !refGroup && !refEncoder ) { // Single motor case, no encoder 
     double startTic = refMotor->position( degrees ); // Establish a reference point for your encoders

     while(breakout < breakoutCount){               // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refMotor->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError);   // calculate PID response

      // uncomment statement below to debug
      //printf("M w/o E --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }

  else if ( !refMotor && !refEncoder ) { // Motor group case
    double startTic = refGroup->position( degrees );

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refGroup->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError); // calculate PID response

      // uncomment statement below to debug
      //printf("MG w/o E--- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refGroup->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refGroup->stop();
  }

  else if ( !refGroup ) { // Single Motor and Encoder
    double startTic = refEncoder->position( degrees );

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError); // calculate PID response

      // uncomment statement below to debug
      //printf("M w/ E --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
  else if ( !refMotor ) {  // Motor Group with Encoder 
    double startTic = refEncoder->position( degrees );

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError); // calculate PID response

      // uncomment statement below to debug
      //printf("MG w/ E --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop(); // ensures that the motor stops so it doesn't draw power
  }
}

void controlMotor::pidAccel( double target, double maxVel, double minVel, double accelPeriod, int breakoutCount ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);    // set PID response
  int update_hz = updateRate;                             // set the update rate;
  double tolBound = 5;                                    // tic offset bound of the target -> larger = easier to breakout
  int breakout = 0;                                       // initialized the breakout variable

  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target);                  // initialize absolute total error of manuever
  double error = target;                             // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response

  if (!refEncoder && !refGroup) {
     double startTic = refMotor->position( degrees ); // Establish a reference point for your encoders                 

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refMotor->position( degrees );  // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
     // printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }

      //printf("%i \t %f \t %f \n", breakout, fabs((startTic + target) - tolBound), fabs((startTic + target) + tolBound));
    }

    refMotor->stop();
  }
  else if (!refMotor && !refEncoder) {
    double startTic = refGroup->position( degrees ); // Establish a reference point for your encoders                    

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refGroup->position(degrees);    // grabs current position 
      error =  (startTic + target) - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
      //printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refGroup->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refGroup->stop();
  }
  else if ( !refGroup ) {
    double startTic = refEncoder->position( degrees ); // Establish a reference point for your encoders                   

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = (startTic + target) - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
      //printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
   else if ( !refMotor ) {
    double startTic = refEncoder->position( degrees ); // Establish a reference point for your encoders                   

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = (startTic + target) - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(pctError); } // Standard PID response

      // uncomment statement below to debug
      //printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refGroup->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound) && (currAngle <= (startTic + target) + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refGroup->stop();
  }
}

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝


--- ODOMETRY THREAD CALLBACK---
*/

void odomTrackCall(botOdom *botObject, rotation *verticalDW, rotation *horizontalDW, inertial *imuObject){
  while(true){
    // Insert for your robot accordingly for the thread
    // --- Updating the Position and heading of the robot (use custom "update" for your system)
    botObject->trackLocation(verticalDW->position( degrees ), horizontalDW->position( degrees ), imuObject->rotation( degrees ) );

    // --- Print the position and heading of the robot for debugging purposes
    printf("Global Coordinates: [%.2f, %.2f, %.2f] \n", botObject->xG, botObject->yG, (botObject->tG)*RAD2DEG);

    // --- Sleep the task for accurate update tracking.
    task::sleep(1000/botObject->update_hz);
  }
}

vector<vector<double>> linInject (vector<double> startCoords, vector<double> endCoords, double division ) {
  double m = (endCoords[1] - startCoords[1])/(endCoords[0] - startCoords[0]);
  double b = startCoords[1] - m*startCoords[0];
  double dx = (endCoords[0] - startCoords[0])/division;
  
  vector<vector<double>> pathCoords;
  double tempX = startCoords[0];
  double tempY = startCoords[1];
  vector<double> tempPosition = {tempX, tempY};
  
  for (int iX = 1; iX <= division; iX++) {
    tempX += dx;
    tempY = m*tempX - b;
    //printf("lin, Step %i --- X: %f, Y: %f \n", iX, tempX, tempY);
    tempPosition = {tempX, tempY};
    pathCoords.push_back(tempPosition);
  }

  return pathCoords;
}

vector<vector<double>> bezInject (vector<double> startCoords, vector<double> controlCoords1, vector<double> controlCoords2, vector<double> endCoords, double division) {
  vector<vector<double>> pathCoords;
  double tempX = startCoords[0];
  double tempY = startCoords[1];
  vector<double> tempPosition = {tempX, tempY};
  double t = 1/division;
  
  for(int iX = 1; iX <= division; iX++) {
    tempX = pow(1-t,3)*startCoords[0] + 3*pow(1-t, 2)*t*controlCoords1[0] + 3*(1-t)*pow(t,2)*controlCoords2[0] + pow(t, 3)*endCoords[0];
    tempY = pow(1-t,3)*startCoords[1] + 3*pow(1-t, 2)*t*controlCoords1[1] + 3*(1-t)*pow(t,2)*controlCoords2[1] + pow(t, 3)*endCoords[1];
    
    //printf("bez4, Step %i, t:%.2f  --- X: %f, Y: %f \n", iX, t, tempX, tempY);
    
    t += 1/division;
    tempPosition = {tempX, tempY};
    pathCoords.push_back(tempPosition);
  }
  
  return pathCoords;
}

double findVecDist(vector<double> p1, vector<double> p2) {
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
}

double findVecDet(vector<double> p1, vector<double> p2) {
  return sqrt(p1[0]*p2[1] - p2[0]*p1[1]);
}

int sgn( double val ) {
  if (val >= 0) { return  1; }
  else          { return -1; }
}
  
void pursuePath (vector<double> currPos, vector<vector<double>> path, double velocity, double lookAhead) {
  for (int iX = 0; iX < path.size(); iX++) {
    bool intersectFound = false;
    double x1_offset = path[iX][1] - currPos[1];
    double y1_offset = path[iX][2] - currPos[2];
    double x2_offset = path[iX+1][1] - currPos[1];
    double y2_offset = path[iX+1][2] - currPos[2];
    
    double dx = x2_offset - x1_offset;      
    double dy = y2_offset - y1_offset;
    double dr = sqrt(pow(dx, 2) + pow(dy, 2));
    double D = x1_offset*y2_offset - x2_offset*y1_offset;
    double discriminant = pow(lookAhead, 2) * pow(dr, 2) - pow(D, 2);
    
    if (discriminant >= 0){
      intersectFound = true;
        
      double x1Sol = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
      double x2Sol = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
      double y1Sol = (- D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
      double y2Sol = (- D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);  
        
      printf("Disc: %f \t pt1: [%f, %f] \t pt2: [%f, %f] \n", discriminant, x1Sol, y1Sol, x2Sol, y2Sol);
    }
  }
}
