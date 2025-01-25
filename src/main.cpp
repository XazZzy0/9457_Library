#include "vex.h"
#include "9457Lib.h"
using namespace vex;

// === Object specification ===
competition Competition;
brain Brain;

motor Motor = motor(PORT5, false);            // The motor
rotation Rotation = rotation(PORT9, false);  // Rotation callout (for PID example)
inertial IMU = inertial(PORT2);              // For the inertial unit

// === Global Library specification ===
botOdom botTracking;
controlMotor tempMotor(&Motor, &Rotation);

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝

*/

/* --- DRIVER PID FUNCTION ---
THIS IS AN EXAMPLE ON HOW TO DRIVE A MOTOR TO AN ANGLE WITH A PID LOOP
WILL NEED TO USE MULTI-THREADING TO IMPLEMENT PROPERLY WITH OTHER MOTORS
AND DO MULTIPLE THINGS AT ONCE

INPUTS:
vel    -> Max velocity of the motor (pct)
pTerm  -> Proportional coefficient (optional)
iTerm  -> Integral coefficient (optional)
dTerm  -> Derivative coefficient (optional)

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
}*/

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
  while(true){
    // Updating the Position and heading of the robot (use custom "update" for your system)
    //botTracking.update(Rotation.position(degrees), Rotation2.position(degrees), 0, 3.25, 3.25);

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

}

void userControl( void ) {
  while( true ) {
    // Do nothing
    task::sleep(100);
  }
}

void autoControl( void ) {
  // Do nothing
}

int main() {
  tempMotor.setPID(3.5);
  tempMotor.pidAccel(900, 100);

  Competition.drivercontrol( userControl );
  Competition.autonomous( autoControl );

  //thread Odometry = thread( sysUpdate ); // Initate a seperate thread to run Odometry in the background.

  while(true) {
    task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
  }
}