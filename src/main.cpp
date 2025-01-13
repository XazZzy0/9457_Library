#include "vex.h"
using namespace vex;

// === PID Initialization ===
double angleSet[] = {0, 360, 180};
double stateError = 0.0;
int state = 0;

// === Object specification ===
competition Competition;
brain Brain;
motor Motor = motor(PORT5, false);            // The motor
rotation Rotation = rotation(PORT9, false);   // Rotation callout (for PID example)
rotation vDW = rotation(PORT10, false);       // vertical deadwheel 
rotation vDW_left = rotation(PORT11, false);  // vertical deadwheel (left side)
rotation vDW_right = rotation(PORT12, false); // vertical deadwheel (right side)
rotation hDW = rotation(PORT13, false);       // horizontal deadwheel 
inertial IMU = inertial(PORT14);              // For the inertial unit

/* === PID Controller ===
THIS IS A PID OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE
*/
class PID {
    private:
        // settings for the PID
        double Kp = 0.0, Ki = 0, Kd = 0.0;        //coefficients
        double integral = 0.0, prev_error = 0.0;  //integral, previous error (for "I" and "D" coeff.)
        double maxPower = 100.0;                  // max output of the motors (typically 100%)

    public:
        // Initalizing external access variables
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0;

        // Initialize outside of class definition
        PID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}

        // Calculate the PID response (expects a percentage input from 0-100)
        double calculate( double error ) {
          Pterm = Kp * error;
          integral += error;
          Iterm = Ki * integral; 
          Dterm = Kd * (error - prev_error);

          double output = Pterm + Iterm + Dterm; // Sum all of the values

          // clamps the output
          if (output > maxPower)          { output = maxPower; }
          else if (output < -maxPower)    { output = -maxPower; }

          // sets the previous error to the new one
          prev_error = error;

          // return the PID output 
          return output;
        }

        void reset( void ){
          integral = 0;
          prev_error = 0;
        }

        void setVel(double toPower){
          maxPower = toPower;
        }
};

/* === Odometry Class ===
THIS IS A ODOMETRY OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE.
MULTI-THREADING WILL NEED TO BE UTILIZED IN THE BACKGROUND IF PROPER IMPLEMENTATION IS DESIRED.

THE BACK-END MATH IS BASED ON EUCLIDIAN DYNAMICS AND MAY BE PRONE TO ERROR. THIS IS FOR A 
BASIC CONTROL SYSTEM.
*/
class botOdom {
  private:
    double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev =0, tPrev = 0;  // Variables to represent the previous encoder positions
    double vdot = 0, hdot = 0, tdot = 0;                            // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]
    double vddot = 0, hddot = 0, tddot = 0;                         // Variables for the rotational acceleration deltas, (deg/hz^2, deg/hz^2, Rad/hz) [ROBOT FRAME]

  public:
   // creating neccessary variables for the class - updating and global/previous variables.
    double update_hz;
    double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]

    // Possible Constructors of the Odometry function
    // CASE 1 - 2 DW (perpedicular to eachother), 1 IMU
    botOdom( void ) :  
      xG(0), yG(0), tG(0),
      update_hz(50) 
      {} 
    
    // CASE 2 - 1 DW (parallel to wheelbase), 1 IMU
    botOdom( double offsetV ) : 
      xG(0), yG(0), tG(0),
      update_hz(50) 
      {} 

    // CASE 3 - 3 DW, no IMU
    botOdom( double wheelBase, double normBase ) : 
      xG(0), yG(0), tG(0),
      update_hz(50) 
      {}  

    /* --- SYSTEM UPDATE 1 FUNCTION ---
    THIS IS HOW TO UPDATE THE ROBOT FRAME SO THAT THE X, Y, AND HEADING
    OF THE ROBOT IS ACCURATELY REPRESENTED.

    ONLY USE WHEN 1 DEADWHEEL AND AN IMU ARE PRESENT, A VERTICAL ONE AND A 
    HORIZONTAL ONE,

    INPUTS:
    vWheel           -> Vertical Deadwheel position reading
    vWheel_Diameter  -> Vertical Deadwheel Diameter (optional input)
    hWheel           -> Horizontal Deadwheel position reading
    hWheel_Diameter  -> Horizontal Deadwheel Diameter (optional input)
    angle            -> IMU Reading 0-360
    */
    void update ( double vWheel, double hWheel, double angle, double vWheel_Diameter = 2.75,  double hWheel_Diameter = 2.75 ) {
      tdot = (tPrev - angle)*(3.14159/180);   // change in heading from the previous position.  [Radians/hz]
      vdot = (vWheel - vPrev)/360;            // change in the x location from the previous position. [rot/hz]
      hdot = (hWheel - hPrev)/360;            // change in the y location from the previous position. [rot/hz]

      tG += tdot;                                                                                 // update Inertial Frame [Radians]
      xG += (vdot*cos(tG)*3.14159*vWheel_Diameter) + (hdot*sin(tG)*3.14159*hWheel_Diameter);      // update Inertial Frame [Inches]
      yG += (vdot*sin(tG)*3.14159*vWheel_Diameter) - (hdot*cos(tG)*3.14159*hWheel_Diameter);      // update Inertial Frame [Inches]

      // update the previous positions to the current positions for the next iteration
      vPrev = vWheel; 
      hPrev = hWheel;
      tPrev = angle;
    } 

    /* --- SYSTEM UPDATE 2 FUNCTION ---
    THIS IS HOW TO UPDATE THE ROBOT FRAME SO THAT THE X, Y, AND HEADING
    OF THE ROBOT IS ACCURATELY REPRESENTED.

    ONLY USE WHEN 2 DEADWHEELS AND AN IMU ARE PRESENT, A VERTICAL ONE AND A 
    HORIZONTAL ONE,

    INPUTS:
    vWheel           -> Vertical Deadwheel position reading
    vWheel_Diameter  -> Vertical Deadwheel Diameter (optional input)
    hWheel           -> Horizontal Deadwheel position reading
    hWheel_Diameter  -> Horizontal Deadwheel Diameter (optional input)
    angle            -> IMU Reading 0-360
    */
    void update_2 ( double hWheel, double angle, double hWheel_Diameter = 2.75 ) {

    }

    /* --- SYSTEM UPDATE 3 FUNCTION ---
    THIS IS HOW TO UPDATE THE ROBOT FRAME SO THAT THE X, Y, AND HEADING
    OF THE ROBOT IS ACCURATELY REPRESENTED.

    ONLY USE WHEN 3 DEADWHEELS ARE PRESENT, TWO VERTICAL ONES AND A 
    HORIZONTAL ONE,

    INPUTS:
    vWheel           -> Vertical Deadwheel position reading
    vWheel_Diameter  -> Vertical Deadwheel Diameter (optional input)
    hWheel           -> Horizontal Deadwheel position reading
    hWheel_Diameter  -> Horizontal Deadwheel Diameter (optional input)
    angle            -> IMU Reading 0-360
    */
    void update_3 ( double vWheelL, double vWheelR, double hWheel, double vWheelL_Diameter = 2.75, double vWheelR_Diameter = 2.75, double hWheel_Diameter = 2.75 ) {

    }

    // How to declare your global starting point
    void setPose ( double xPose, double yPose, double tPose ) {
      xG = xPose;
      yG = yPose;
      tG = tPose;
    }

    // How to declare the update rate of the system. CANNOT GO ABOVE 100 HZ
    void change_rate ( double rate_hz ) 
    { update_hz = rate_hz; }

};
// Creation of the Global Odometry class - so that it can be accessed everywhere
botOdom botTracking; 

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
  Rotation.resetPosition();                        // reset the position of the rotation sensor
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
  int breakout = 0;

  // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)
  Rotation.resetPosition();                          // reset the position of the rotation sensor
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = abs(target - currAngle);       // initialize absolute total error of manuever
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

    printf("target: %f \t Pterm: %f \t Iterm: %f \t Dterm: %f \n", currAngle, anglePID.Pterm, anglePID.Iterm, anglePID.Dterm);
    
    Motor.spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

    task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

    if (currAngle >= target - 2 && currAngle <= target + 2){ ++breakout; } // Count up on the breakout period
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
  int breakout = 0;

  // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)
  Rotation.resetPosition();                          // reset the position of the rotation sensor
  double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
  double totalError = abs(target - currAngle);       // initialize absolute total error of manuever
  double error = target - currAngle;                 // initialize error
  double toPower;                                    // initialize the speed variable
  double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

  // Set the motor power to the specified amount
  anglePID.setVel(vel);

  while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
    currAngle = Rotation.position(degrees);       // grabs current position 
    error = target - currAngle;                   // grabs current error
    pctError = error / totalError * 100;          // calculate percent error of manuever (0 = end, 100 = beginning, 

    if (100 - abs(pctError) <= accelPeriod) { // Acceleration Period
      if (pctError > 0){ // fwd
        toPower = ((100-abs(pctError))/accelPeriod*100) * ((100-minVel)/100) + minVel; // kickstart the Accel/PID at minVel (fwd)
        toPower /= 100; // convert to a pct
      }
      else { // rev
        toPower = -((100-abs(pctError))/accelPeriod*100) * ((100-minVel)/100) - minVel; // kickstart the Accel/PID at minVel (rev)
        toPower /= 100; // convert to a pct
      }  
    }
    else { toPower = anglePID.calculate(pctError)/100; } // Standard PID response

    printf("target: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
    
    Motor.spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

    task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

    if (currAngle >= target - 2 && currAngle <= target + 2){ ++breakout; } // Count up on the breakout period
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
  while(true){
    // Updating the Position and heading of the robot (use custom "update" for your system)
    botTracking.update( vDW_left.position(degrees), hDW.position(degrees), IMU.heading(degrees) ); // in this case - 1 horizontal, 1 vertical, 1 imu

    // Print the position and heading of the robot for debugging purposes
    printf("X Position = [%.2f] \t Y Position = [%.2f] \t Theta Heading = [%.] \n", botTracking.xG, botTracking.yG, botTracking.tG);

    // Sleep the task for accurate update tracking.
    task::sleep(1000/botTracking.update_hz);
  }
}

// Pre-autonomous intial setups
void pre_auton ( void ){

}

void userControl( void ) {
  while( true ) {
    task::sleep(100);
  }
}

void autoControl( void ) {
  botTracking.setPose(0, 0, 0); 
}

int main() {
    botTracking.change_rate(50); // Set the update rate of the odometry tracking system (in Hz, or updates/sec), STD is 50.

    Competition.drivercontrol( userControl );
    Competition.autonomous( autoControl );

    thread Odometry = thread( sysUpdate ); // Initate a seperate thread to run Odometry in the background.

    while(true) {
      task::sleep(100); // prevent main from exiting with an infinite loop -> For task scheduling
    }
}