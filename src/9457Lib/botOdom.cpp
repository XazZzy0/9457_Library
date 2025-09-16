#include "9457Lib/botOdom.h"

using namespace vex; // using namespace vex helps to type less

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
  xG(0), yG(0), tG(0),
  IMU(nullptr)
  {} 

// CASE 2 - 2 DW (perpedicular to eachother), 1 IMU
botOdom::botOdom( rotation *vertWheel, double offset_V, rotation *horWheel, double offset_H, inertial *setIMU, const double sysRate ) :  
  vOffset(offset_V), hOffset(offset_H),  
  vWheel(vertWheel), hWheel(horWheel),
  xG(0), yG(0), tG(0),
  update_hz(sysRate),
  IMU(setIMU)
  {} 

void botOdom::initializeSystem( void ){ // used to completely set the odom class for start
  if (IMU){
    IMU->calibrate( int32_t(1.5) );
    while(IMU->isCalibrating()) { task::sleep(100); }
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nSTART PROGRAM: \n");

    IMU->setRotation(0, degrees);
  
    if (vWheel) { vWheel->resetPosition(); }
    if (vLWheel) { vLWheel->resetPosition(); }
    if (vRWheel) { vRWheel->resetPosition(); }
    if (hWheel) { hWheel->resetPosition(); }

    if (vWheel_Diameter == 0) { printf("VERTICAL WHEEL DIAMETER NEEDS TO BE SET FOR ODOM TO CALIBRATE PROPERLY \n"); }
    if (hWheel_Diameter == 0) { printf("HORIZONTAL WHEEL DIAMETER NEEDS TO BE SET TO CALIBRATE PROPERLY \n"); }
    if (baseWidth == 0) { printf("BASE WIDTH NEEDS TO BE SET TO CALIBRATE PROPERLY\n"); }
    if (baseLength == 0) { printf("BASE LENGTH NEEDS TO BE SET TO CALIBRATE PROPERLY\n"); }

    xG = 0, yG = 0, tG = 0;

    isCalibrated = true; // init passed
  }
  else{ // initialization failed - NO IMU
    xG = 0, yG = 0, tG = 0;
    if (vWheel) { vWheel->resetPosition(); }
    if (hWheel) { hWheel->resetPosition(); }
      
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nSTART PROGRAM: \n");
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
  tdot = (angle/180*M_PI) - tPrev;                          // change in heading from the previous position.  [Radians/hz]
  vdot = (vWheel - vPrev)/360*M_PI*vWheel_Diameter;            // change in the x location from the previous position. [in/hz]
  hdot = (hWheel - hPrev)/360*M_PI*vWheel_Diameter;            // change in the y location from the previous position. [in/hz]
  if (tdot == 0) {tdot = 1E-8;};                             // prevent divide by zero errors 

  // All the following calculations are taken from the Pilons position tracking document
  double lx = 2*sin(tdot/2)*(hdot/tdot + hOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
  double ly = 2*sin(tdot/2)*(vdot/tdot + vOffset);  //Local (Robot) Odometry Frame shifted by tdot/2
                                                                                
  xG += (ly*cos(tG + tdot/2)) + (lx*sin(tG + tdot/2));     // update Inertial Frame [Inches] - aka field frame
  yG += (ly*sin(tG + tdot/2)) - (lx*cos(tG + tdot/2));     // update Inertial Frame [Inches] - aka field frame
  tG += tdot/M_PI*180;                                   // update Inertial Frame [Degrees] - aka field frame

  if      (tG > 360) { tG = 0; }    // checks to see if the global is outside of normal bounds (over-travel)
  else if (tG < 0)   { tG = 360; }  // checks to see if the global is outside of normal bounds (under-travel)

  // update the previous positions to the current positions for the next iteration
  vPrev = vWheel; 
  hPrev = hWheel;
  tPrev = angle/180*M_PI; // convert to radians for the next iteration
} 