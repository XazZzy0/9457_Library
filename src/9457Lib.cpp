/* 
--- EVERYONE: ---

This code is for reference only, since it has been written by ME an "Adult", 
direct use of this library during a competition is not allowed!!!

If used/distributed by the students (such as copy and pasting), the decision is not supported by the 
coaches or our organization. The student takes full responsibility for any punishment that may occur 
proceeding such instance.

This is not intended to be distributed as a general use-case library. Its purpose is to demonstrate 
high level engineering concepts during practice such as:
 - Object oriented programming
 - Custom library creation
 - Feedback control systems
 - Sensor filtering
 - Odometry
 - Pure pursuit 
and provide examples for how robotics teams can use such concepts to succeed and solve problems in and 
out of competition. All code must and will be written by the students. It is our full intention to abide
by the REC Foundation student-centered rules: 

 - https://kb.roboticseducation.org/hc/en-us/articles/5449868745367-Student-Centered-Policy 
 - https://www.vexforum.com/t/code-templates-and-student-centered-policy-discussion/115567/15
 
-----------------------------------------------------------------------------------------------------------------------------

I do not approve of the stance that competition robotics has taken with open-source libraries and I believe 
it goes against the student-centered policies (especially since many students cannot write these functions themselves, explain the concepts,
or even properly explain why they use it)
(See below for general REC ruling) 
~ "Students use a custom library developed by another team, but cannot create their own custom libraries." ~

In general, the rules are unenforced and therefore abused which devalues the time, effort, and experience that many other very talented programmers have put into 
their own robot and programming techniques.

For clarity, libraries are not bad, in fact quite the opposite. But in a learning environment - it's usage without citation and proper understanding is plagarism.
Therefore, this library was written to properly document, provide resources, explain the concepts, and verify it works so I may guide my students 
through something that is known to be difficult. Hence, this is given as a line by line general use-case and is fully 
commented so any student may reverse engineer and better understand the process workflow and math for these concepts. 

Nobody is an expert when they first start - so sometimes it is best to work from an example and deconstruct the solution.

~ Signed by: Jacob Wood [2025-02-07 : 15:27], Change Rev: more updates header/main documentation

*/

#include "vex.h"
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

// Calculate the PID response (expects a percentage input from 0-100)
double PID::calculate( double error ) {
  Pterm = Kp * error;               // This is the P response - acts like a spring
  integral += error;                // this is the I summation - totals the error over time
  Iterm = Ki * integral;            // This is the I response - pushes error to 0 over time.
  Dterm = Kd * (error - prevError); // This is the D response - acts like a Dampener

 // Preventing integral windup - when the integral term gets too high and the system cannot respond to the next manuever because of the Iterm.
 if (integral > windup)      { integral = windup; }
 else if (integral < windup) { integral = -windup; }
 
  double output = Pterm + Iterm + Dterm; // Sum all of the values - the PID response

  // clamps the output
  if (output > maxPower)          { output = maxPower; }
  else if (output < -maxPower)    { output = -maxPower; }

  // updates to min speed if there is one
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

void PID::reset( void ){
  integral = 0;
  prevError = 0;
  return;
}

void PID::adjPID(double kP, double kI, double kD){
  Kp = kP;
  Ki = kI;
  Kd = kD;
  return;
}

void PID::adjWindup( double wTerm ) { windup = wTerm; }

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
  vWheel(vertWheel), hWheel(horWheel),
  vOffset(offset_V), hOffset(offset_H),
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
chassis::chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, vex::inertial *botIMU ) : 
    left(leftGroup), right(rightGroup), IMU(botIMU)
    {}


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
