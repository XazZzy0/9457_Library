#include "vex.h"
#include "9457Lib.h"
using namespace vex;

// === PID Initialization ===
double angleSet[] = {0, 360, 180};
double stateError = 0.0;
int state = 0;

// === Object specification ===
competition Competition;
brain Brain;
motor Motor = motor(PORT5, false);            // The motor
motor Motorleft1 = motor(PORT2, false);       // The motor
motor Motorleft2 = motor(PORT2, false);       // The motor
motor Motorright1 = motor(PORT2, false);      // The motor
motor Motorright2 = motor(PORT2, false);      // The motor
motor_group leftSide = motor_group(Motorleft1, Motorleft2);     // leftSide
motor_group rightSide = motor_group(Motorright1, Motorright2);  // rightSide

rotation Rotation = rotation(PORT9, false);  // Rotation callout (for PID example)
rotation Rotation2 = rotation(PORT5, false);  // Rotation callout (for PID example)
rotation vDW = rotation(PORT2, false);       // vertical deadwheel 
rotation vDW_left = rotation(PORT2, false);  // vertical deadwheel (left side)
rotation vDW_right = rotation(PORT2, false); // vertical deadwheel (right side)
rotation hDW = rotation(PORT2, false);       // horizontal deadwheel 
inertial IMU = inertial(PORT2);              // For the inertial unit

// === Global Library specification ===
botOdom botTracking(2.75, 2.75, IMU);

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝

*/

// For pressing on the Brain and updating the angle state
void pressed_function( void ){
  ++state;
  if (state > 2)
  {
    state = 0;
  }

  stateError = angleSet[state] - Rotation.position(degrees);
  if ( stateError < 1 ) { stateError = 360; }
}

/* --- DRIVER PID FUNCTION ---
THIS IS AN EXAMPLE ON HOW TO DRIVE A MOTOR TO AN ANGLE WITH A PID LOOP
WILL NEED TO USE MULTI-THREADING TO IMPLEMENT PROPERLY WITH OTHER MOTORS
AND DO MULTIPLE THINGS AT ONCE

INPUTS:
vel    -> Max velocity of the motor (pct)
pTerm  -> Proportional coefficient (optional)
iTerm  -> Integral coefficient (optional)
dTerm  -> Derivative coefficient (optional)
*/
void PIDdriver (double vel, double pTerm = 3.15, double iTerm = 0.0, double dTerm = 0.225){
  // Initialize PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(pTerm, iTerm, dTerm); 
  int update_hz = 50; 

  // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)        
  Rotation.resetPosition();                        // reset the position of the rotation sensor, motor encoder, imu heading, etc...
  double currAngle = 0;                            // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double error = angleSet[state] - currAngle;      // initialize error
  double toPower;                                  // initialize the speed variable
  double pctError;                                 // initialize the pctError

  // Set the motor power to the specified amount
  anglePID.setVel(vel);

  // for divide by zero errors
  if ( stateError < 1 ) { stateError = 360; }

  while(true){ // insert your conditional for when you want it to run (base it off of being always true)  
    Brain.Screen.pressed( pressed_function );
    
    currAngle = Rotation.position(degrees);       // grabs current position 
    error = angleSet[state] - currAngle;          // grabs current error
    pctError = error / stateError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

    toPower = anglePID.calculate(pctError)/100; // calculate PID response

    printf("target: %f, State: %i \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, state, error, pctError, toPower);
    
    Motor.spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

    task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)
  }
  Motor.stop(); // ensures that the motor stops so it doesn't draw extra power
}

/* --- AUTON PID FUNCTION ---
THIS IS AN EXAMPLE ON HOW TO DRIVE A MOTOR TO AN ANGLE WITH A PID LOOP

INPUTS:
target -> Target angle (degrees)
vel    -> Max velocity of the motor (pct)
pTerm  -> Proportional coefficient (optional)
iTerm  -> Integral coefficient (optional)
dTerm  -> Derivative coefficient (optional)
*/
void PIDauto (double target, double vel, double pTerm = 3.15, double iTerm = 0.0, double dTerm = 0.225, int breakoutCount = 12){
  // Initialize PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(pTerm, iTerm, dTerm);
  int update_hz = 50;
  double tolBound = 5; 
  int breakout = 0;

  // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)
  Rotation.resetPosition();                          // reset the position of the rotation sensor, motor encoder, imu heading, etc...
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target - currAngle);      // initialize absolute total error of manuever
  double error = target - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  // Set the motor power to the specified amount
  anglePID.setVel(vel);

  while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
    currAngle = Rotation.position(degrees);       // grabs current position 
    error = target - currAngle;                   // grabs current error
    pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

    toPower = anglePID.calculate(pctError)/100; // calculate PID response

    printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
    
    Motor.spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

    task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

    if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
    else { breakout = 0; }
  }
  Motor.stop(); // ensures that the motor stops so it doesn't draw extra power
}

/* --- AUTON PID + ACCEL FUNCTION ---
THIS IS AN EXAMPLE ON HOW TO DRIVE A MOTOR TO AN ANGLE WITH A PID LOOP

INPUTS:
target -> Target angle (degrees)
vel    -> Max velocity of the motor (pct)
accel  -> acceleration period of your manuever (pct, optional);
pTerm  -> Proportional coefficient (optional)
iTerm  -> Integral coefficient (optional)
dTerm  -> Derivative coefficient (optional)
*/
void PIDaccel (double target, double vel, double accelPeriod = 15, double minVel = 10, double pTerm = 3.35, double iTerm = 0.0, double dTerm = 0.225, int breakoutCount = 12){
  // Initialize PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
  PID anglePID(pTerm, iTerm, dTerm);
  int update_hz = 50; 
  double tolBound = 5; 
  int breakout = 0;

  // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)
  Rotation.resetPosition();                          // reset the position of the rotation sensor, motor encoder, imu heading, etc...
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = fabs(target - currAngle);      // initialize absolute total error of manuever
  double error = target - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  // Set the motor power to the specified amount
  anglePID.setVel(vel);

  while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
    currAngle = Rotation.position(degrees);       // grabs current position 
    error = target - currAngle;                   // grabs current error
    pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

    if (100 - fabs(pctError) <= accelPeriod) { // Acceleration Period
      if (pctError > 0){ // fwd
        toPower = ((100-fabs(pctError))/accelPeriod*100) * ((100-minVel)/100) + minVel; // kickstart the Accel/PID at minVel (fwd)
        toPower /= 100; // convert to a pct
      }
      else { // rev
        toPower = -((100-fabs(pctError))/accelPeriod*100) * ((100-minVel)/100) - minVel; // kickstart the Accel/PID at minVel (rev)
        toPower /= 100; // convert to a pct
      }  
    }
    else { toPower = anglePID.calculate(pctError)/100; } // Standard PID response

    printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
    
    Motor.spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

    task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

    if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
    else { breakout = 0; }
  }
  Motor.stop(); // ensures that the motor stops so it doesn't draw extra power
}

/* --- ODOMETRY THREAD CALLBACK---
THIS IS AN EXAMPLE ON HOW TO DRIVE A MOTOR TO AN ANGLE WITH A PID LOOP

INPUTS:
target -> Target angle (degrees)
vel    -> Max velocity of the motor (pct)
pTerm  -> Proportional coefficient (optional)
iTerm  -> Integral coefficient (optional)
dTerm  -> Derivative coefficient (optional)
*/
void sysUpdate ( void ){
    Rotation.resetPosition(); // reset the tracking wheel positions before operation
    Rotation2.resetPosition();

  while(true){
    // Updating the Position and heading of the robot (use custom "update" for your system)
    //botTracking.update( vDW_left.position(degrees), hDW.position(degrees), IMU.rotation(degrees) ); // in this case - 1 horizontal, 1 vertical, 1 imu
    botTracking.update(Rotation.position(degrees), Rotation2.position(degrees), 0);

    // Print the position and heading of the robot for debugging purposes
    printf("Global Coordinates: [%.2f, %.2f, %.2f] \n", botTracking.xG, botTracking.yG, botTracking.tG*RAD2DEG);

    // Sleep the task for accurate update tracking.
    task::sleep(1000/botTracking.update_hz);
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
  IMU.calibrate(2);     // Remember to calibrate your IMU
  vDW.resetPosition();  // Set encoders to zero
  hDW.resetPosition();  // Set encoders to zero
}

void userControl( void ) {
  while( true ) {
    task::sleep(100);
  }
}

void autoControl( void ) {
  botTracking.setPose(0, 0, 0); // Sets the current location plus heading of the robot
}


int main() {
    //PIDauto(360, 20);
    Competition.drivercontrol( userControl );
    Competition.autonomous( autoControl );

    thread Odometry = thread( sysUpdate ); // Initate a seperate thread to run Odometry in the background.

    while(true) {
      task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
    }
}