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
it goes against the student-centered policies (especially since many were written by "adults" - per VEX 
rulebook definition) such as Lemlib, jar template, etc... The rules are unenforced and abused and it has devalued 
the effort that many other very talented teams have put into their programming - and students are unable to re-write and 
explain these concepts. (See below for REC ruling)
"Students use a custom library developed by another team, but cannot create their own custom libraries."

This library was written to properly document it, explain the concepts, and verify it works so I may guide my students 
through something that is known to be difficult. Hence, this is given as a line by line general use-case and is fully 
commented so any student may reverse engineer and better understand the process workflow and math for these concepts. 

Nobody is an expert when they first start - so sometimes it is best to work from an example and deconstruct the solution.

~ Signed by: Jacob Wood [2025-02-06 : 13:26], Change Rev: Updates to header/main documentation

*/

#ifndef EASTLIBRARY // Header guard
#include "vex.h"
using namespace vex;


/*
██████╗ ███████╗███████╗██╗███╗   ██╗██╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔══██╗██╔════╝██╔════╝██║████╗  ██║██║╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
██║  ██║█████╗  █████╗  ██║██╔██╗ ██║██║   ██║   ██║██║   ██║██╔██╗ ██║███████╗
██║  ██║██╔══╝  ██╔══╝  ██║██║╚██╗██║██║   ██║   ██║██║   ██║██║╚██╗██║╚════██║
██████╔╝███████╗██║     ██║██║ ╚████║██║   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═════╝ ╚══════╝╚═╝     ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

#define EASTLIBRARY
#define PI          3.14159 // Pi
#define DEG2RAD     PI/180  // Degrees to Radians
#define RAD2DEG     180/PI  // Radians to Degrees

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
// functions and variables. They help make everything more organized

/** ==============================================================================================================================================================================
 * @class PID
 * @details 
 * 
 * The PID: A multi-purpose feedback control system which is based on positional error.
 * ==============================================================================================================================================================================
 */
// Read "manual tuning" section for behavior: https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
class PID {
    private: 
        double Kp = 0.0, Ki = 0, Kd = 0.0;        // The P.I.D. gain coefficients
        double integral = 0.0, prevError = 0.0;   // integral, previous error (for "I" and "D" coeff.)
        double windup = 100.0;                    // the integral windup term - prevents the I response from building up too high.
        double maxPower = 100.0;                  // max output of the motors (typically 100%)
        double minPower = 0.0;                    // minimum output of the motors (you can't move a robot at 0% motor power after all!)

    public:
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0; // Initalizing external access variables
        
        PID(double kp, double ki, double kd);         // A constructor - this is how you specify creating the object

        double calculate ( double error );             // calculate the PID response
        void reset( void );                            // reset the integral and previous error
        void adjPID(double kP, double kI, double kD);  // adjust the objects PID gain terms
        void adjWindup( double wTerm = 100.0 );        // adjust the windup term (default to 100)
        void setVel( double toPower );                 // set the maximum velocity of the response
};

/** ==============================================================================================================================================================================
 * @class botOdom
 * @details 
 * 
 * THIS IS A ODOMETRY OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE.
 * MULTI-THREADING WILL NEED TO BE UTILIZED IN THE BACKGROUND IF PROPER IMPLEMENTATION IS DESIRED.
 * THE BACK-END MATH IS BASED ON EUCLIDIAN DYNAMICS AND MAY BE PRONE TO ERROR. THIS IS FOR A 
 * BASIC CONTROL SYSTEM AND DEALS WITH ALL THE BACK-END UPDATING IN TERMS OF TOTALS/ABSOLUTES.
 * IMU.ROTATION() WILL NEED TO BE CALLED ALONG WITH MOTOR.POSITION()
 * ==============================================================================================================================================================================
 */
// This is a class used to declare robot odometry, it is used for robot tracking. See this document for more info:
// https://github.com/team914/autolib-pdfs/blob/master/pilons-position-tracking.pdf
class botOdom {
    private:
        inertial *IMU;                                                  // A pointer which holds the vex imu class
        rotation *vWheel, *vLWheel, *vRWheel, *hWheel;                  // pointers which hold the rotation wheels - can leave empty(null) if they do not exist!

        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]

    public:
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
 * @class controlDrive
 * @details 
 * 
 * [Insert description here]
 * ==============================================================================================================================================================================
 */
class chassis { // Used for pathing manuevers with Odometry
    private:
        motor_group *left, *right;                // a motor_group containing the left and right side of the drivebase
        inertial *IMU;                            // an inertial class containing the IMU info
        botOdom *ODOM;                            // an inertial class
        double drivePID[3] = {0.675, 0, 0.225};   // the PID Coeff storage for driving manuevers.
        double turnPID[3] = {0.55, 0.0, 0.225};   // the PID Coeff storage for turning manuevers.
        int updateRate = 50.0;                    // the update rate of the system.

    public:
        chassis( motor_group *leftGroup, motor_group *rightGroup, inertial *botIMU ); // Constructor for the Chassis object
        
        void setODOM(botOdom *botODOM);  // pointer to the odometry objet, can be used for pure pursuit.                                           
        void initialize( void );  // Function to be used to initalize the chassis
        void setDrivePID(double pTerm  = 0.85, double iTerm = 0, double dTerm = 0.225); //Default PID gain response to driving
        void setTurnPID(double pTerm  = 0.6, double iTerm = 0, double dTerm = 0.225); //Default PID gain response to turning

        void driveFwd( double dist, double vel, double minVel = 3, int brakoutCount = 8, bool waitForCompletion = true ); // The PID drive forward command 
        void driveAccel( double dist, double vel, double accelPeriod = 15, double minVel = 3, int breakoutCount = 8, bool waitForCompletion = true); // The PID accelerate command 
        void pointTurn(double degrees, double vel, double minVel = 3, int breakoutCount = 12, bool waitForCompletion = true); // The PID point turn command 
};

/** ==============================================================================================================================================================================
 * @class controlMotor
 * @details 
 * 
 * 
 * [Insert description here]
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
        
        void setPID( double pTerm = 1.1, double iTerm = 0, double dTerm = 0.225 ); // Set the default PID variables
        void setRate( double rate_hz ); // Set the update rate of the system
        
        void testSpin( void ); // a test spin function
        void pidRotate( double target, double maxVel, int breakoutCount = 6 ); // a PID rotate command
        void pidAccel( double target, double maxVel, double accelPeriod = 15, double minVel = 10, int breakoutCount = 12 ); // a PID acceleration command
        void pidTurn( double target, double maxVel, int breakoutCount = 12 ); // a PID turn command
};

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/

void odomTrackCall(botOdom *botObject, rotation *verticalDW, rotation *horizontalDW, inertial *imuObject); // For odometry updates

#endif // End of File //
