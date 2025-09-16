#ifndef EASTLIBRARY     // Header guard
#define EASTLIBRARY
// ===============================================================================================================================================================================

// Here is a fantastic library of resources by team 914. It is the golden standard for 
// What should be implemented on a high level competition robot:
// https://github.com/team914/autolib-pdfs/blob/master/pilons-position-tracking.pdf


/*
 ██████╗██╗      █████╗ ███████╗███████╗███████╗███████╗
██╔════╝██║     ██╔══██╗██╔════╝██╔════╝██╔════╝██╔════╝
██║     ██║     ███████║███████╗███████╗█████╗  ███████╗
██║     ██║     ██╔══██║╚════██║╚════██║██╔══╝  ╚════██║
╚██████╗███████╗██║  ██║███████║███████║███████╗███████║
 ╚═════╝╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝╚══════╝╚══════╝
*/

// Classes can be better defined by https://www.geeksforgeeks.org/c-classes-and-objects/. 
// They are the foundation of object oriented programming (OOP) and are 'objects' which can hold both 
// functions and variables. They help make everything more organized when you have a huge library such as this!

#include "9457Lib/chassis.h"    // The chassis class - to control the entire robot and the motors

/*
██╗   ██╗ █████╗ ██████╗ ██╗ █████╗ ██████╗ ██╗     ███████╗███████╗     █████╗ ███╗   ██╗██████╗     ██████╗ ███████╗███████╗██╗███╗   ██╗██╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██║   ██║██╔══██╗██╔══██╗██║██╔══██╗██╔══██╗██║     ██╔════╝██╔════╝    ██╔══██╗████╗  ██║██╔══██╗    ██╔══██╗██╔════╝██╔════╝██║████╗  ██║██║╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
██║   ██║███████║██████╔╝██║███████║██████╔╝██║     █████╗  ███████╗    ███████║██╔██╗ ██║██║  ██║    ██║  ██║█████╗  █████╗  ██║██╔██╗ ██║██║   ██║   ██║██║   ██║██╔██╗ ██║███████╗
╚██╗ ██╔╝██╔══██║██╔══██╗██║██╔══██║██╔══██╗██║     ██╔══╝  ╚════██║    ██╔══██║██║╚██╗██║██║  ██║    ██║  ██║██╔══╝  ██╔══╝  ██║██║╚██╗██║██║   ██║   ██║██║   ██║██║╚██╗██║╚════██║
 ╚████╔╝ ██║  ██║██║  ██║██║██║  ██║██████╔╝███████╗███████╗███████║    ██║  ██║██║ ╚████║██████╔╝    ██████╔╝███████╗██║     ██║██║ ╚████║██║   ██║   ██║╚██████╔╝██║ ╚████║███████║
  ╚═══╝  ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚══════╝    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝     ╚═════╝ ╚══════╝╚═╝     ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

// These below are known as Macros, they are basically shortcuts to information.
// Enum, is short for ENUMERATOR - It is an easy way to make it so that you can set specific keywords to values instead of directly setting variables or creating macros.
enum TEAMCOLOR {RED, BLUE, emptyColor};                // This is to hold your team color at the start of match
enum AUTONSET  {LEFT, RIGHT, SKILLS, emptyAuton};      // This is to hold the variable for which auton you are using.

const auto systemHz = 50.0;                       // The update rate of the system (in hz) - this is the default value for the system
const auto placeholder = 0.0;

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

extern void odomTrackCall(botOdom *botObject, vex::rotation *verticalDW, vex::rotation *horizontalDW, vex::inertial *imuObject, bool printData);       // For odometry updates - multi-threading needs a callback function
extern int DEG2RAD(double degrees);                                                                                       // Degrees to Radians functions
extern int RAD2DEG(double radians);                                                                                       // Degrees to Radians functions

//extern double findVecDist(std::vector<double> p1, vector<double> p2);                                                                  // Find the distance (only for vectors)
//extern double findVecDet(vector<double> p1, vector<double> p2);                                                                        // Find the determinant (only for vectors)
//extern int sgn( double val );                                                                                                          // determine the sign, output: (1 or -1)

// Path Pursuit Point injection and Algorithms
//extern vector<vector<double>> linInject (vector<double> startCoords, vector<double> endCoords, double division = 20);                                                               // Inject a linear path
//extern vector<vector<double>> bezInject (vector<double> startCoords, vector<double> controlCoords1, vector<double> controlCoords2, vector<double> endCoords, double division = 20); // Inject a bezier curve path 
//extern void pursuePath (vector<double> currPos, vector<vector<double>> path, double velocity, double lookAhead);


#endif // End of File //
