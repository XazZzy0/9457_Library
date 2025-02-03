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



/*
 ██████╗██╗      █████╗ ███████╗███████╗███████╗███████╗
██╔════╝██║     ██╔══██╗██╔════╝██╔════╝██╔════╝██╔════╝
██║     ██║     ███████║███████╗███████╗█████╗  ███████╗
██║     ██║     ██╔══██║╚════██║╚════██║██╔══╝  ╚════██║
╚██████╗███████╗██║  ██║███████║███████║███████╗███████║
 ╚═════╝╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝╚══════╝╚══════╝
*/

/** ==============================================================================================================================================================================
 * @class PID
 * @details 
 * 
 * The PID: A multi-purpose feedback control system which is based on positional error.
 * ==============================================================================================================================================================================
 */
// The PID: A multi-purpose feedback control system which is based on positional error.
class PID {
    private: 
        double Kp = 0.0, Ki = 0, Kd = 0.0;        //coefficients
        double integral = 0.0, prevError = 0.0;   //integral, previous error (for "I" and "D" coeff.)
        double maxPower = 100.0;                  // max output of the motors (typically 100%)
        double minPower = 0.0;                    // minimum output of the motors

    public:
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0; // Initalizing external access variables
        
        PID(double kp, double ki, double kd);

        double calculate ( double error );
        void adjPID(double kP, double kI, double kD);
        void reset( void );
        void setVel( double toPower );
        void setMinVel( double toMinPower );
};

/** ==============================================================================================================================================================================
 * @class botOdom
 * @details 
 * 
 * THIS IS A ODOMETRY OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE.
 * MULTI-THREADING WILL NEED TO BE UTILIZED IN THE BACKGROUND IF PROPER IMPLEMENTATION IS DESIRED.
 * THE BACK-END MATH IS BASED ON EUCLIDIAN DYNAMICS AND MAY BE PRONE TO ERROR. THIS IS FOR A 
 * BASIC CONTROL SYSTEM AND DEALS WITH ALL THE BACK-END UPDATING IN TERMS OF TOTALS/ABSOLUTES.
 * IMU.ROTATION() WILL NEED TO BE CALLED ALONG WITH MOTOR.POSITION(), INSTEAD OF IMU.HEADING()
 * AND MOTOR.ANGLE().
 * ==============================================================================================================================================================================
 */
// This is a class used to declare robot odometry, it is used for robot tracking.
class botOdom {
    private:
        inertial *IMU;
        rotation *vWheel, *vLWheel, *vRWheel, *hWheel;

        double vWheel_Diameter = 0, hWheel_Diameter = 0;
        double baseWidth = 0, baseLength = 0; 
        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]

    public:
        bool isCalibrated = false;                                       // Verifies bot initalization
        double update_hz = 50;
        double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]
        
        botOdom( void );
        botOdom( rotation *vertWheel, double offset_V, rotation *horWheel, double offset_H );

        void initializeSystem( void );

        void setVerticalDiameter( double Diameter );
        void setHorizontalDiameter( double Diameter );
        void setBotSize( double width, double length );
        void setPose( double xPose, double yPose, double tPose );
        void setRate( double rate_hz );

        void trackLocation( double vWheel, double hWheel, double angle );
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
        motor_group *left, *right;              // a motor_group containing the left and right side of the drivebase
        inertial *IMU;                          // an inertial class containing the IMU info
        botOdom *ODOM;                          // an inertial class
        double drivePID[3] = {0.675, 0, 0.225};   // the PID Coeff storage for driving manuevers.
        double turnPID[3] = {0.55, 0.0, 0.225}; // the PID Coeff storage for turning manuevers.
        int updateRate = 50.0;                  // the update rate of the system.

    public:
        chassis( motor_group *leftGroup, motor_group *rightGroup, inertial *botIMU );

        void setODOM(botOdom *botODOM);
        void initialize( void );
        void setDrivePID(double pTerm  = 0.85, double iTerm = 0, double dTerm = 0.225);
        void setTurnPID(double pTerm  = 0.6, double iTerm = 0, double dTerm = 0.225);

        void driveFwd( double dist, double vel, double minVel = 3, int brakoutCount = 8, bool waitForCompletion = true );
        void driveAccel( double dist, double vel, double accelPeriod = 15, double minVel = 3, int breakoutCount = 8, bool waitForCompletion = true);
        void pointTurn(double degrees, double vel, double minVel = 3, int breakoutCount = 12, bool waitForCompletion = true);
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
        controlMotor( motor *ptrMotor );
        controlMotor( motor *ptrMotor, rotation *ptrRot );
        controlMotor( motor_group *ptrGroup );
        controlMotor( motor_group *ptrGroup, rotation *ptrRot );
        
        void setPID( double pTerm = 1.1, double iTerm = 0, double dTerm = 0.225 ); // Set the default PID variables
        void setRate( double rate_hz ); // Set the update rate of the system
        
        void testSpin( void );
        void pidRotate( double target, double maxVel, int breakoutCount = 6 );
        void pidAccel( double target, double maxVel, double accelPeriod = 15, double minVel = 10, int breakoutCount = 12 );
        void pidTurn( double target, double maxVel, int breakoutCount = 12 );
};

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
*/
// For odometry updates
void odomTrackCall(botOdom *botObject, rotation *verticalDW, rotation *horizontalDW, inertial *imuObject);

#endif // End of File //