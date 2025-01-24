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
  maxPower = fabs(toPower);
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
// Possible Consrtuctors of the Odometry function
// CASE 1 - No DW, 1 IMU
botOdom::botOdom( vex::inertial &IMU_init ) :  
  update_hz(50),
  IMU(IMU_init),
  xG(0), yG(0), tG(0)
  {} 

// CASE 2 - 2 DW (perpedicular to eachother), 1 IMU
botOdom::botOdom( double v_Offset, double h_Offset, vex::inertial &IMU_init) :  
  update_hz(50),
  vOffset(v_Offset), hOffset(h_Offset), 
  IMU(IMU_init),
  xG(0), yG(0), tG(0)
  {} 

// CASE 3 - 1 DW (perp/parallel), 1 IMU -> bool is true if DW is perpendicular to Wheelbase, else false if parallel
botOdom::botOdom( bool wheelPerp, vex::inertial &IMU_init ) : 
  update_hz(50),
  IMU(IMU_init),
  xG(0), yG(0), tG(0) 
  {} 

// CASE 4 - 3 DW, no IMU
botOdom::botOdom( double LeftOffset, double RightOffset, double RearOffset, vex::inertial &IMU_init) : 
  update_hz(50),
  IMU(IMU_init),
  xG(0), yG(0), tG(0)
  {}  

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
void botOdom::update ( double vWheel, double hWheel, double angle, double vWheel_Diameter,  double hWheel_Diameter ) {
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

/* --- SYSTEM UPDATE 2 FUNCTION ---
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
void botOdom::update_2 ( double hWheel, double angle, double hWheel_Diameter ) {

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
void botOdom::update_3 ( double vWheelL, double vWheelR, double hWheel, double vWheelL_Diameter, double vWheelR_Diameter, double hWheel_Diameter ) {

}

// How to declare your global starting point
void botOdom::setPose ( double xPose, double yPose, double tPose ) {
  xG = xPose;
  yG = yPose;
  tG = tPose;
  IMU.setRotation(tPose, degrees);
}

// How to declare the update rate of the system. CANNOT GO ABOVE 100 HZ
void botOdom::change_rate ( double rate_hz ) 
{ update_hz = rate_hz; }

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
controlDrive::controlDrive( vex::motor_group &leftGroup, vex::motor_group &rightGroup, vex::inertial &botIMU ) : 
    left(leftGroup), right(rightGroup), IMU(botIMU)
    {}

void controlDrive::driveFwd( double dist, double vel, bool waitCompletion ) {

}

void controlDrive::pointTurn( double degrees, double vel, bool waitCompletion ) {

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
  refMotor(ptrMotor), refEncoder(nullptr) 
  {}

controlMotor::controlMotor( vex::motor *ptrMotor, vex::rotation *ptrRot) :
  refMotor(ptrMotor), refEncoder(ptrRot)
  {}

void controlMotor::testSpin( void ){
  refMotor->spin(vex::forward, 100, vex::velocityUnits::pct);
}

void controlMotor::pidRotate( double target, double maxVel, double pTerm, double iTerm, double dTerm, int breakoutCount ){
  if (!refEncoder) { // nullptr case
    // Do something - need to write lol
  }
  else {  
    // Initialize local PID, absolute minimum velocity, and How many times it should update in a second (std = 50 hz, don't go above 100 hz)
    PID anglePID(pTerm, iTerm, dTerm);
    int update_hz = 50;
    double tolBound = 5; 
    int breakout = 0;

    // Reset the encoder to zero, gather the new manuever distance, can be positve or negative (indicates cw/ccw rotation)
    refEncoder->resetPosition();                        // reset the position of the rotation sensor, motor encoder, imu heading, etc...
    double currAngle = 0;                              // initialize current angle variable - FYI, pre-initalization makes the code slightly faster when running
    double totalError = fabs(target - currAngle);      // initialize absolute total error of manuever
    double error = target - currAngle;                 // initialize error
    double toPower;                                    // initialize the speed variable
    double pctError = error / totalError * 100;        // initalize the percent error of the manuever (-100% or 100%, can be positve/negative for cw/ccw rotation)

    // Set the motor power to the specified amount
    anglePID.setVel(maxVel);

    while(breakout < breakoutCount){ // insert your conditional for when you want it to run (base it off of being always true)
      currAngle = refEncoder->position(degrees);       // grabs current position 
      error = target - currAngle;                   // grabs current error
      pctError = error / totalError * 100;          // calculate percent error of manuever (0 = beginning, 100 = end, 

      toPower = anglePID.calculate(pctError)/100; // calculate PID response

      printf("target: %f \t Error: %f \t pctError: %f \t toPower: %f \n", currAngle, error, pctError, toPower);
      
      refMotor->spin(fwd, toPower*12, voltageUnits::volt);   // spin the motor (using voltage)

      task::sleep(1000/update_hz); // required, need to sleep the task for a bit - otherwise you will get multi-threading scheduling errors (if multi-threading)

      if (fabs(currAngle) >= fabs(target - tolBound) && fabs(currAngle) <= fabs(target + tolBound)){ ++breakout; } // Count up on the breakout period
      else { breakout = 0; }
    }
  }

  //refMotor->stop(); // ensures that the motor stops so it doesn't draw extra power
  
}
