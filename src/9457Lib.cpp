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
  Pterm = Kp * error;
  integral += error;
  Iterm = Ki * integral; 
  Dterm = Kd * (error - prevError);

  double output = Pterm + Iterm + Dterm; // Sum all of the values

  // clamps the output
  if (output > maxPower)          { output = maxPower; }
  else if (output < -maxPower)    { output = -maxPower; }

  // sets the previous error to the new one
  prevError = error;

  // return the PID output 
  return output;
}

void PID::reset( void ){
  integral = 0;
  prevError = 0;
}

void PID::setVel(double toPower){
  if (fabs(toPower) > 100) { toPower = 100; } // Max speed is 100 pct

  maxPower = fabs(toPower); // No negative signs for power, std to positive
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


void botOdom::setVerticalDiameter( double Diameter )    { vWheel_Diameter = Diameter; }
void botOdom::setHorizontalDiameter( double Diameter)   { hWheel_Diameter = Diameter; }
void botOdom::setBotSize( double width, double length ) { baseWidth = width, baseLength = length; }

// Declare your global location 
void botOdom::setPose ( double xPose, double yPose, double tPose ) {
  xG = xPose;
  yG = yPose;
  tG = tPose;
  IMU->setRotation(tPose, degrees);
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
void botOdom::update( double vWheel, double hWheel, double angle ) {
  tdot = (angle - tPrev)*(DEG2RAD);                          // change in heading from the previous position.  [Radians/hz]
  vdot = (vWheel - vPrev)/360*PI*vWheel_Diameter;            // change in the x location from the previous position. [in/hz]
  hdot = (hWheel - hPrev)/360*PI*vWheel_Diameter;            // change in the y location from the previous position. [in/hz]
  if (tdot == 0) {tdot = 1E-8;};                             // prevent divide by zero errors 

  double lx = 2*sin(tdot/2)*(hdot/tdot + hOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
  double ly = 2*sin(tdot/2)*(vdot/tdot + vOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
                                                                                
  xG += (ly*cos(tG + tdot/2)) + (lx*sin(tG + tdot/2));     // update Inertial Frame [Inches]
  yG += (ly*sin(tG + tdot/2)) - (lx*cos(tG + tdot/2));     // update Inertial Frame [Inches]
  tG += tdot; 

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

void chassis::driveFwd( double dist, double vel, bool waitCompletion ) {
  //do nothing, for right now
}

void chassis::pointTurn( double angle, double vel, int breakoutCount, bool waitCompletion ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(turnPID[0], turnPID[1], turnPID[2]);
  int update_hz = updateRate;
  double tolBound = 3; 
  int breakout = 0;
  
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(angle - currAngle);      // initialize absolute total error of manuever
  double error = angle - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  // Set the motor power to the specified amount
  anglePID.setVel(vel);

  while( breakout < breakoutCount ) {
    currAngle = IMU->rotation( degrees );
    error = angle - currAngle;
    pctError = error / totalError * 100;

    toPower = anglePID.calculate( pctError ) / 100;

    printf("With Encoder (MG) --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);

    left->spin( fwd, toPower * 12, voltageUnits::volt );
    right->spin( reverse, toPower * 12, voltageUnits::volt );

    task::sleep( 1000 / update_hz );

    if (fabs( currAngle ) >= fabs( angle - tolBound ) && fabs( currAngle ) <= fabs( angle + tolBound ) ) { ++breakout; }
    else { breakout = 0; }
  }

  left->stop();
  right->stop();
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

void controlMotor::pidRotate( double target, double maxVel, int breakoutCount ){
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);
  int update_hz = updateRate;
  double tolBound = 5; 
  int breakout = 0;
  
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target - currAngle);      // initialize absolute total error of manuever
  double error = target - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)
  
  // Set the motor power to the specified amount
  anglePID.setVel(maxVel);
  
  if (!refEncoder && !refGroup) { // Single motor case, no encoder 
     refMotor->resetPosition(); // Reset the encoder to zero

     while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refMotor->position(degrees);       // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError)/100; // calculate PID response

      printf("Without Encoder --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
  else if (!refMotor && !refEncoder) { // Motor group case
    refGroup->resetPosition(); // Reset the encoder to zero

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refGroup->position(degrees);       // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError)/100; // calculate PID response

      printf("With Encoder (MG) --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refGroup->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refGroup->stop();
  }
  else if ( !refGroup ) { // Single Motor and Encoder
    refEncoder->resetPosition(); // Reset the encoder to zero

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);       // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError)/100; // calculate PID response

      printf("With Encoder (MG) --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
  /*
  else {  // Default case 
    refEncoder->resetPosition();  // Reset the encoder to zero

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);       // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError)/100; // calculate PID response

      printf("With Encoder (Motor) --- target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
  }
  */

  //refMotor->stop(); // ensures that the motor stops so it doesn't draw power
}

void controlMotor::pidAccel( double target, double maxVel, double accelPeriod, double minVel, int breakoutCount ) {
  // Initialize PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);
  int update_hz = updateRate; 
  double tolBound = 5; 
  int breakout = 0;

  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target - currAngle);      // initialize absolute total error of manuever
  double error = target - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  // Set the motor power to the specified amount
  anglePID.setVel(maxVel);

  if (!refEncoder && !refGroup) {
    // Reset the encoder to zero
    refMotor->resetPosition();                    

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
          toPower /= 100; // convert to a pct
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
          toPower /= 100; // convert to a pct
        }  
      }
      else { toPower = anglePID.calculate(pctError)/100; } // Standard PID response

      printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
  else if (!refMotor && !refEncoder) {
    // Reset the encoder to zero
    refGroup->resetPosition();                    

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refGroup->position(degrees);    // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
          toPower /= 100; // convert to a pct
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
          toPower /= 100; // convert to a pct
        }  
      }
      else { toPower = anglePID.calculate(pctError)/100; } // Standard PID response

      printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refGroup->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refGroup->stop();
  }
  else if ( !refGroup ) {
    // Reset the encoder to zero
    refEncoder->resetPosition();                    

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);    // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (pctError > 0){ // fwd
          toPower = ((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
          toPower /= 100; // convert to a pct
        }
        else { // rev
          toPower = -((100-fabs(pctError))/accelPeriod) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
          toPower /= 100; // convert to a pct
        }  
      }
      else { toPower = anglePID.calculate(pctError)/100; } // Standard PID response

      printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }

    refMotor->stop();
  }
}