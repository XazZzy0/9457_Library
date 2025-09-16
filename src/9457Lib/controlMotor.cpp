#include "9457Lib/controlMotor.h"

using namespace vex; // using namespace vex helps to type less

/*
███████╗███╗   ███╗ █████╗ ██████╗ ████████╗    ███╗   ███╗ ██████╗ ████████╗ ██████╗ ██████╗
██╔════╝████╗ ████║██╔══██╗██╔══██╗╚══██╔══╝    ████╗ ████║██╔═══██╗╚══██╔══╝██╔═══██╗██╔══██╗
███████╗██╔████╔██║███████║██████╔╝   ██║       ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝
╚════██║██║╚██╔╝██║██╔══██║██╔══██╗   ██║       ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔══██╗
███████║██║ ╚═╝ ██║██║  ██║██║  ██║   ██║       ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║  ██║
╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝       ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝  ╚═╝
*/


controlMotor::controlMotor( vex::motor *ptrMotor, const double sysRate ) : 
  refMotor(ptrMotor), refGroup(nullptr), refEncoder(nullptr), updateRate(sysRate) 
  {}

controlMotor::controlMotor( vex::motor *ptrMotor, vex::rotation *ptrRot, const double sysRate) :
  refMotor(ptrMotor), refGroup(nullptr), refEncoder(ptrRot), updateRate(sysRate) 
  {}

controlMotor::controlMotor( vex::motor_group *ptrGroup, vex::rotation *ptrRot, const double sysRate) :
  refMotor(nullptr), refGroup(ptrGroup), refEncoder(ptrRot), updateRate(sysRate) 
  {}

controlMotor::controlMotor( vex::motor_group *ptrGroup, const double sysRate ) :
  refMotor(nullptr), refGroup(ptrGroup), refEncoder(nullptr), updateRate(sysRate) 
  {}

// Setting the brake type of the chassis
void controlMotor::setBrake( brakeType type ) {
 if (refGroup){
    switch (type) {
      case brakeType::coast:
        refGroup->setStopping(coast);
        break;
      case brakeType::brake: 
        refGroup->setStopping(brake);
        break;
     case brakeType::hold: 
        refGroup->setStopping(hold);
        break;
     } 
 }
 else if (refMotor){
    switch (type) {
      case brakeType::coast:
        refMotor->setStopping(coast);
        break;
      case brakeType::brake: 
        refMotor->setStopping(brake);
        break;
     case brakeType::hold: 
        refMotor->setStopping(hold);
        break;
     } 
 }
}

void controlMotor::setPID( double pTerm, double iTerm, double dTerm ){
  PID_Coef[0] = pTerm;
  PID_Coef[1] = iTerm;
  PID_Coef[2] = dTerm;
}

void controlMotor::setTolBound( double seconds, double degrees ){
  tolBound_seconds = seconds;
  tolBound_degrees = degrees;
  tolBound_hz = updateRate*tolBound_seconds; // the time it takes to break out of the PID loop (default is 0.1 seconds)
}

void controlMotor::testSpin( void ){
  if (!refGroup) {
    refMotor->spin(vex::forward, 100, vex::velocityUnits::pct);
  } 
  else{
    refGroup->spin(vex::forward, 100, vex::velocityUnits::pct);
  }
}

void controlMotor::pidRotate( double target, double maxVel, double minVel ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);    // set PID response
  int breakout = 0;                                  // initialize the breakout counter
  
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double error = target;                             // initialize error
  double toPower;                                    // initialize the speed variable
  
  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response

  if ( !refGroup && !refEncoder ) { // Single motor case, no encoder 
    double startTic = refMotor->position( degrees ); // Establish a reference point for your encoders

    do{
      currAngle = refMotor->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error 

      toPower = anglePID.calculate(error);   // calculate PID response - based on angular error
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/updateRate); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound_degrees) && (currAngle <= (startTic + target) + tolBound_degrees )){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
    while(int(tolBound_hz) >= breakout);               // insert your conditional for when you want it to run (base it off of being always true)

    refMotor->stop();
  }

  else if ( !refMotor && !refEncoder ) { // Motor group case
    double startTic = refGroup->position( degrees );

    do{
      currAngle = refGroup->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error 

      toPower = anglePID.calculate(error);   // calculate PID response - based on angular error
      
      refGroup->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/updateRate); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound_degrees) && (currAngle <= (startTic + target) + tolBound_degrees )){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
    while(int(tolBound_hz) >= breakout);               // insert your conditional for when you want it to run (base it off of being always true)

    refGroup->stop();
  }

  else if ( !refGroup ) { // Single Motor and Encoder
    double startTic = refEncoder->position( degrees );

    do{
      currAngle = refEncoder->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error 

      toPower = anglePID.calculate(error);   // calculate PID response - based on angular error
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/updateRate); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound_degrees) && (currAngle <= (startTic + target) + tolBound_degrees )){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
    while(int(tolBound_hz) >= breakout);               // insert your conditional for when you want it to run (base it off of being always true)

    refMotor->stop();
  }
  
  else if ( !refMotor ) {  // Motor Group with Encoder 
    double startTic = refEncoder->position( degrees );

    do{
      currAngle = refEncoder->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error 

      toPower = anglePID.calculate(error);   // calculate PID response - based on angular error
      
      refGroup->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/updateRate); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound_degrees) && (currAngle <= (startTic + target) + tolBound_degrees )){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
    while(int(tolBound_hz) >= breakout);               // insert your conditional for when you want it to run (base it off of being always true)

    refMotor->stop(); // ensures that the motor stops so it doesn't draw power
  }
}

/*
void controlMotor::pidAccel( double target, double maxVel, double minVel, double accelPeriod ) {
  // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(PID_Coef[0], PID_Coef[1], PID_Coef[2]);    // set PID response
  int breakout = 0;                                  // initialize the breakout counter
  
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double error = target;                             // initialize error
  double accelBound = fabs(target * accelPeriod);          // initialize the acceleration bound of the manuever (default is 15% of manuever) 
  double toPower;                                    // initialize the speed variable
  
  anglePID.setVel(minVel, maxVel);         // Set the Minimum and Maximum Velocity of the response

  if (!refEncoder && !refGroup) {
    double startTic = refMotor->position( degrees ); // Establish a reference point for your encoders     
     
    do{
      currAngle = refMotor->position( degrees );    // grabs current position 
      error = (startTic + target) - currAngle;      // grabs current error 

      if ( (fabs(error) <= accelBound) ) { // Acceleration Period
        if (!signbit(target)){ // fwd
          toPower = ((error)/accelBound) * (maxVel-minVel) + minVel; // kickstart the Accel/PID at minVel (fwd)
        }
        else { // rev
          toPower = ((error)/accelBound) * (maxVel-minVel) - minVel; // kickstart the Accel/PID at minVel (rev)
        }  
      }
      else { toPower = anglePID.calculate(error); } // Standard PID response
      
      refMotor->spin(fwd, toPower, velocityUnits::pct);   // spin the motor (using voltage)

      task::sleep(1000/updateRate); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if ((currAngle >= (startTic + target) - tolBound_degrees) && (currAngle <= (startTic + target) + tolBound_degrees && (!runContinuous))){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
     }
    while(int(tolBound_hz) >= breakout);               // insert your conditional for when you want it to run (base it off of being always true)

    refMotor->stop();
  }
  else if (!refMotor && !refEncoder) {
    double startTic = refGroup->position( degrees ); // Establish a reference point for your encoders                    

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refGroup->position(degrees);    // grabs current position 
      error =  (startTic + target) - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

      if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
        if (!signbit(pctError)){ // fwd
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
        if (!signbit(pctError)){ // fwd
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
        if (!signbit(pctError)){ // fwd
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
  */