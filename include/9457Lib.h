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
commented so ANY student may reverse engineer and better understand the process workflow and math for these concepts. 

Nobody is an expert when they first start - so sometimes it is best to work from an example and deconstruct the solution.

~ Signed by: Jacob Wood [2025-02-07 : 15:27], Change Rev: more updates header/main documentation

*/

#ifndef EASTLIBRARY     // Header guard
#include "vex.h"        // The vex functions - to call the standard vex functions
#include <vector>       // The vector std library, makes the path persuit much easier.
using namespace vex;    // using namespace vex helps to type less
using namespace std;    // using namespace std helps to type less


/*
██████╗ ███████╗███████╗██╗███╗   ██╗██╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔══██╗██╔════╝██╔════╝██║████╗  ██║██║╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
██║  ██║█████╗  █████╗  ██║██╔██╗ ██║██║   ██║   ██║██║   ██║██╔██╗ ██║███████╗
██║  ██║██╔══╝  ██╔══╝  ██║██║╚██╗██║██║   ██║   ██║██║   ██║██║╚██╗██║╚════██║
██████╔╝███████╗██║     ██║██║ ╚████║██║   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═════╝ ╚══════╝╚═╝     ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

// These below are known as Macros, they are basically shortcuts to information.
// === GLOBAL MACROS ===
#define EASTLIBRARY
#define PI          3.14159 // Pi
#define DEG2RAD     PI/180  // Degrees to Radians
#define RAD2DEG     180/PI  // Radians to Degrees

// Enum, is short for ENUMERATOR - It is an easy way to make it so that you can set specific keywords to values instead of directly setting variables or creating macros.
enum TEAMCOLOR {RED, BLUE, emptyColor};                // This is to hold your team color at the start of match
enum AUTONSET  {LEFT, RIGHT, SKILLS, emptyAuton};      // This is to hold the variable for which auton you are using.

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
// They are the foundation of object oriented programming and are 'objects' which can hold both 
// functions and variables. They help make everything more organized when you have a huge library such as this!

/** ==============================================================================================================================================================================
 * @class PID
 * @details 
 * 
 * This is a class-based PID. It is a multi-purpose feedback control system which is based on positional error from your target.
 * It's pretty close to how the motors function when you call a spinTo or a spinFor function on the motors but you can put the target
 * as anything once you get the hang of how it behaves.
 * 
 * In more general terms, it operates by giving power to the motors depending where it is, compared to where it isn't!
 * ==============================================================================================================================================================================
 */
// Read "manual tuning" section to understand how the variables impact behavior: 
// https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller

class PID {
    // Private means that these variables/methods can only be accessed from the methods within the class itself
    private:                                      
        double Kp = 0.0, Ki = 0, Kd = 0.0;        // The P.I.D. gain coefficients
        double integral = 0.0, prevError = 0.0;   // integral, previous error (for "I" and "D" coeff.)
        double windup = 100.0;                    // the integral windup term - prevents the I response from building up too high.
        double maxPower = 100.0;                  // max output of the motors (typically 100%)
        double minPower = 0.0;                    // minimum output of the motors (you can't move a robot at 0% motor power after all!)

    // Public means that you can call/access everything in main.
    public:
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0; // Initalizing external access variables
        double initI = 50.0;                          // The starting I sum term - lower it for the I to have less effect on the system
        
        PID(double kp, double ki, double kd);         // A constructor - this is how you specify creating the object

        double calculate ( double error );                   // calculate the PID response
        void reset( void );                                  // reset the integral and previous error
        void adjPID(double kP, double kI, double kD);        // adjust the objects PID gain terms
        void adjWindup( double wTerm = 100.0 );              // adjust the windup term (default to 100)
        void setVel( double toMinPower, double toMaxPower ); // set the maximum velocity of the response
};

/** ==============================================================================================================================================================================
 * @class botOdom
 * @details 
 * 
 * This is an odometry object. Odometry, generally, is the tracking of a specific distance. In robotics the terminology gets intermixed
 * with position tracking - Overall, what this class holds is a bunch of information regarding the position of your robot with respect to
 * the field. These include things such as your X position, Y position, and Theta postion. AKA, now you know exactly where your robot is 
 * on the field at all points.
 * ==============================================================================================================================================================================
 */
// See this document for more info:
// https://github.com/team914/autolib-pdfs/blob/master/pilons-position-tracking.pdf

class botOdom {
    private:
        inertial *IMU;                                                   // A pointer which holds the vex imu class - pointers make it so that you do not have to "copy" the entire class into a function.

        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]

    public:
        rotation *vWheel, *vLWheel, *vRWheel, *hWheel;                   // pointers which hold the rotation wheels - can leave empty(null) if they do not exist!
        bool isCalibrated = false;                                       // Verifies bot initalization - is false if calibration doesn't occur.
        double vWheel_Diameter = 0, hWheel_Diameter = 0;                 // Variables which specify the vert/horizontal wheel diameters
        double baseWidth = 0, baseLength = 0;                            // Variables which represent the Drivebase Length/Width
        double update_hz = 50;                                           // The update rate of the robot
        double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]             // The Global X, Y, and Theta location.
        
        botOdom( void );                                                                           // Empty Odom Constructor - mainly for testing
        botOdom( rotation *vertWheel, double offset_V, rotation *horWheel, double offset_H );      // Primary Odom Constructor - 2 Deadwheels with offsets from the tracking center

        void initializeSystem( void );                                       // Used as an initialization function - change as you see fit.

        void setVerticalDiameter( double Diameter );                         // Set the vertical wheel diameter
        void setHorizontalDiameter( double Diameter );                       // Set the horizontal wheel diameter
        void setBotSize( double width, double length );                      // Set the bot sizing.
        void setPose( double xPose, double yPose, double tPose );            // Set the X, Y, Theta position
        void setRate( double rate_hz );                                      // Set the update rate of the system

        void trackLocation( double vWheel, double hWheel, double angle );    // The callback to track the location.
};

/** ==============================================================================================================================================================================
 * @class chassis
 * @details 
 * 
 * This is the chassis class. This is an object which holds all of your drivebase(DB) motors and IMU, therefore all you have to do is tell all of your 
 * motors to spin a certain way to move your entire robot. Along with this, if you have odom, you are able to call the odom class as a pointer
 * to drive a certain distance instead of motor degrees!
 * 
 * Some example functions are pointTurn, where you can call a certain amount of degrees and your motors will spin to rotate your robot to that degree 
 * position.
 * ==============================================================================================================================================================================
 */
class chassis { 
    private:
        motor_group *left, *right;                // a motor_group containing the left and right side of the drivebase
        inertial *IMU;                            // an inertial class containing the IMU info
        botOdom *ODOM;                            // an inertial class
        double drivePID[3] = {0.675, 0, 0.225};   // the PID Coeff storage for driving manuevers.
        double turnPID[3] = {0.55, 0.0, 0.225};   // the PID Coeff storage for turning manuevers.
        double headingPID[3] = {0.0, 0.0, 0.0};   // The PID Coeff storage for heading manuevers.
        double swingPID[3] = {0.0, 0.0, 0.0};     // The PID Coeff storage for arc/swing manuevers.
        int updateRate = 50.0;                    // the update rate of the system.

    public:
        chassis( motor_group *leftGroup, motor_group *rightGroup ); // Constructor for the Chassis object - No IMU
        chassis( motor_group *leftGroup, motor_group *rightGroup, inertial *botIMU ); // Constructor for the Chassis object

        void setBrake( int type = 1 ); // Set the brake type of your robot. [0 = coast, 1 = brake, 2 = hold];
        void setODOM(botOdom *botODOM);  // pointer to the odometry objet, can be used for pure pursuit.                                           
        void initialize( void );  // Function to be used to initalize the chassis
        void setDrivePID(double pTerm, double iTerm, double dTerm); // PID gain response to driving
        void setTurnPID(double pTerm, double iTerm, double dTerm); // PID gain response to turning
        void setHeadingPID(double pTerm, double iTerm, double dTerm); // PID gain response to heading changes
        void setSwingPID(double pTerm, double iTerm, double dTerm); // PID gain response to swing/arc turns           

        void driveFwd( double dist, double vel, double minVel = 3, int brakoutCount = 8, bool waitForCompletion = true ); // The PID drive forward command 
        void driveAccel( double dist, double vel, double accelPeriod = 15, double minVel = 3, int breakoutCount = 8, bool waitForCompletion = true); // The PID accelerate command 
        void pointTurn( double degrees, double vel, double minVel = 3, int breakoutCount = 12, bool waitForCompletion = true); // The PID point turn command 
        void swingTurn( double rVel, double lVel, int runTime_ms );

        void arcadeDrive( controller *Controller, float deadband = 5.0 );                  // Default Arcade Control for driving
        void arcadeDrive( controller *Controller, float spline, float deadband = 5.0 );    // Accel based Arcade Control for driving - customizable!
        void tankDrive( controller *Controller, float deadband = 5.0 );                    // Default Tank Control for driving
};

/** ==============================================================================================================================================================================
 * @class controlMotor
 * @details 
 * 
 * 
 * This is how you would control a single motor or motor group with a PID function. It is similar to the chassis class but with a single motor or a group of motors.
 * Examples for how this would be used are for things such as a lift mechanism, or an arm which you need to keep at a certain angle.
 * ==============================================================================================================================================================================
 */

class controlMotor {
    private:
        motor *refMotor;                        // pointer to specific motor which will be controlled
        motor_group *refGroup;                  // pointer to specific motor group which will be controlled
        rotation *refEncoder;                   // pointer to the specific encoder.
        double PID_Coef[3] = {1.1, 0, 0.225};   // the PID Coeff storage for the specific manuever. 
        int updateRate = 50.0;                  // the update Rate of the controller (max is 100 hz)

    public:
        controlMotor( motor *ptrMotor );                           // Constructor for a single motor
        controlMotor( motor *ptrMotor, rotation *ptrRot );         // Constructor for a single motor attached to a rotation sensor
        controlMotor( motor_group *ptrGroup );                     // Constructor for a single motor group
        controlMotor( motor_group *ptrGroup, rotation *ptrRot );   // Constructor for a single motor group attached to a rotation sensor

        void setBrake( int type = 1 ); // Set the brake type of your motor(s). [0 = coast, 1 = brake, 2 = hold]; - this is reduntant code for ease (compared to vex commands)
        void setPID( double pTerm = 1.1, double iTerm = 0, double dTerm = 0.225 ); // Set the default PID variables
        void setRate( double rate_hz ); // Set the update rate of the system
        
        void testSpin( void ); // a test spin function
        void pidRotate( double target, double maxVel, double minVel = 3.0, int breakoutCount = 6 ); // a PID rotate command
        void pidAccel( double target, double maxVel, double minVel = 3.0, double accelPeriod = 15.0, int breakoutCount = 12 ); // a PID acceleration command
        void pidTurn( double target, double maxVel, double minVel = 3.0, int breakoutCount = 12 ); // a PID turn command
};

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

void odomTrackCall(botOdom *botObject, rotation *verticalDW, rotation *horizontalDW, inertial *imuObject);      // For odometry updates - multi-threading needs a callback function
double findVecDist(vector<double> p1, vector<double> p2);                                                       // Find the distance (only for vectors)
double findVecDet(vector<double> p1, vector<double> p2);                                                        // Find the determinant (only for vectors)
int sgn( double val );                                                                                          // determine the sign, output: (1 or -1)

// Path Pursuit Point injection and Algorithms
vector<vector<double>> linInject (vector<double> startCoords, vector<double> endCoords, double division = 20);                                                               // Inject a linear path
vector<vector<double>> bezInject (vector<double> startCoords, vector<double> controlCoords1, vector<double> controlCoords2, vector<double> endCoords, double division = 20); // Inject a bezier curve path 
void pursuePath (vector<double> currPos, vector<vector<double>> path, double velocity, double lookAhead);

#endif // End of File //
