#include "vex.h"
#include "9457Lib.h"

/*===================================================================
                            9457 CODING LIBRARY

                    PROPRIETARY TO MANKATO EAST ROBOTICS:
    (NOT INTENDED FOR ANY DISTRIBUTION WITHOUT COACHES EXPLICIT AUTHORIZATION ) 

Summary: 
    This coding library is intended to act as a guide to assist students in
    understanding how typical control systems operate. It is not meant as a 
    copy and paste guide to make your autonomous work. Each section will likely
    require revisons, but this presents a basic outline to ease the student into
    the workflow rather than pushing a "trial by fire method".

CREATED BY: Jacob Wood (Coach) and Nico Wood (Coach).
LAST MODIFED: (See Github)
===================================================================*/


/*
██████╗ ██╗██████╗      ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗     ███████╗██████╗
██╔══██╗██║██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║     ██╔════╝██╔══██╗
██████╔╝██║██║  ██║    ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║     █████╗  ██████╔╝
██╔═══╝ ██║██║  ██║    ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ██║     ██╔══╝  ██╔══██╗
██║     ██║██████╔╝    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗███████╗███████╗██║  ██║
╚═╝     ╚═╝╚═════╝      ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝

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

THIS IS A ODOMETRY OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE.
MULTI-THREADING WILL NEED TO BE UTILIZED IN THE BACKGROUND IF PROPER IMPLEMENTATION IS DESIRED.

THE BACK-END MATH IS BASED ON EUCLIDIAN DYNAMICS AND MAY BE PRONE TO ERROR. THIS IS FOR A 
BASIC CONTROL SYSTEM AND DEALS WITH ALL THE BACKEND UPDATING IN TERMS OF TOTALS/ABSOLUTES.
IMU.ROTATION() WILL NEED TO BE CALLED ALONG WITH MOTOR.POSITION(), INSTEAD OF IMU.HEADING()
AND MOTOR.ANGLE().
*/

// Possible Constructors of the Odometry function
// CASE 1 - No DW, 1 IMU
botOdom::botOdom( vex::inertial IMU_init ) :  
  xG(0), yG(0), tG(0),
  update_hz(50),
  IMU(IMU_init)
  {} 

// CASE 2 - 2 DW (perpedicular to eachother), 1 IMU
botOdom::botOdom( double v_Offset, double h_Offset, vex::inertial IMU_init) :  
  xG(0), yG(0), tG(0),
  update_hz(50),
  vOffset(v_Offset), hOffset(h_Offset), 
  IMU(IMU_init)
  {} 

// CASE 3 - 1 DW (perp/parallel), 1 IMU -> bool is true if DW is perpendicular to Wheelbase, else false if parallel
botOdom::botOdom( bool wheelPerp, vex::inertial IMU_init ) : 
  xG(0), yG(0), tG(0),
  update_hz(50), 
  IMU(IMU_init)
  {} 

// CASE 4 - 3 DW, no IMU
botOdom::botOdom( double LeftOffset, double RightOffset, double RearOffset, vex::inertial IMU_init) : 
  xG(0), yG(0), tG(0),
  update_hz(50), 
  IMU(IMU_init)
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
███████╗███╗   ███╗ █████╗ ██████╗ ████████╗██████╗ ██████╗ ██╗██╗   ██╗███████╗
██╔════╝████╗ ████║██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔══██╗██║██║   ██║██╔════╝
███████╗██╔████╔██║███████║██████╔╝   ██║   ██║  ██║██████╔╝██║██║   ██║█████╗
╚════██║██║╚██╔╝██║██╔══██║██╔══██╗   ██║   ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝
███████║██║ ╚═╝ ██║██║  ██║██║  ██║   ██║   ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗
╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝

Currently in work. Relative coordinate system.
working to implement motor plug-ins and more advanced manuevers.
Needs Odom to work as of the moment
*/
controlDrive::controlDrive( motor_group leftGroup, motor_group rightGroup, inertial botIMU ) : 
    left(leftGroup), right(rightGroup), IMU(botIMU)
    {}

void controlDrive::driveFwd( double dist, double vel, bool waitCompletion ) {

}

void controlDrive::pointTurn( double degrees, double vel, bool waitCompletion ) {

}