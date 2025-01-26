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

/** ==============================================================================================================================================================================
 * @class PID
 * @details 
 * 
 * [Insert description here]
 * ==============================================================================================================================================================================
 */
class PID {
    private: 
        double Kp = 0.0, Ki = 0, Kd = 0.0;        //coefficients
        double integral = 0.0, prevError = 0.0;   //integral, previous error (for "I" and "D" coeff.)
        double maxPower = 100.0;                  // max output of the motors (typically 100%)

    public:
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0; // Initalizing external access variables
        
        PID(double kp, double ki, double kd);

        double calculate ( double error );
        void reset( void );
        void setVel( double toPower );
};

/** ==============================================================================================================================================================================
 * @class botOdom
 * @details 
 * 
 * THIS IS A ODOMETRY OBJECT, CREATE A NEW INSTANCE WHENEVER YOU WOULD LIKE A UNIQUE RESPONSE.
 * MULTI-THREADING WILL NEED TO BE UTILIZED IN THE BACKGROUND IF PROPER IMPLEMENTATION IS DESIRED.
 * THE BACK-END MATH IS BASED ON EUCLIDIAN DYNAMICS AND MAY BE PRONE TO ERROR. THIS IS FOR A 
 * BASIC CONTROL SYSTEM AND DEALS WITH ALL THE BACKEND UPDATING IN TERMS OF TOTALS/ABSOLUTES.
 * IMU.ROTATION() WILL NEED TO BE CALLED ALONG WITH MOTOR.POSITION(), INSTEAD OF IMU.HEADING()
 * AND MOTOR.ANGLE().
 * ==============================================================================================================================================================================
 */
class botOdom {
    private:
        inertial *IMU;
        rotation *vWheel, *vLWheel, *vRWheel, *hWheel;

        double vWheel_Diameter, hWheel_Diameter;
        double baseWidth, baseLength; 
        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]

    public:
        double update_hz = 50;
        double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]
        
        botOdom( void );
        botOdom( rotation *vertWheel, double offset_V, rotation *horWheel, double offset_H );

        void setVerticalDiameter( double Diameter );
        void setHorizontalDiameter( double Diameter );
        void setBotSize( double width, double length );
        void setPose( double xPose, double yPose, double tPose );
        void setRate( double rate_hz );

        void update( double vWheel, double hWheel, double angle );
        
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
        motor_group *left, *right;   // a motor_group containing the left and right side of the drivebase
        inertial *IMU;       // an inertial class containing the IMU info
        double drivePID[3] = {1.1, 0, 0.225};
        double turnPID[3] = {0.7, 0.001, 0.225};
        int updateRate = 50.0;

    public:
        chassis( motor_group *leftGroup, motor_group *rightGroup, inertial *botIMU );

        void driveFwd( double dist, double vel, bool waitCompletion = true );
        void pointTurn(double degrees, double vel, int breakoutCount = 12, bool waitCompletion = true);
};

/** ==============================================================================================================================================================================
 * @class controlMotor
 * @details 
 * 
 * [Insert description here]
 * ==============================================================================================================================================================================
 */

class controlMotor {
    private:
        motor *refMotor;                        // pointer to specific motor which will be controlled
        motor_group *refGroup;                  // pointer to specific motor group which will be controlled
        rotation *refEncoder;                   // pointer to the specific encoder.
        double PID_Coef[3] = {1.1, 0, 0.225};   // the PID Coeff callback for the specific manuever. 
        int updateRate = 50.0;                  // the update Rate of the controller (max is 100 hz)

    public:
        controlMotor( motor *ptrMotor );
        controlMotor( motor *ptrMotor, rotation *ptrRot );
        controlMotor( motor_group *ptrGroup );
        controlMotor( motor_group *ptrGroup, rotation *ptrRot );
        
        void setPID( double pTerm = 1.1, double iTerm = 0, double dTerm = 0.225 ); // Set the default PID variables
        void setRate( double rate_hz ); // Set the update rate of the system
        
        void testSpin( void );
        void pidRotate( double target, double maxVel, int breakoutCount = 12 );
        void pidAccel( double target, double maxVel, double accelPeriod = 15, double minVel = 10, int breakoutCount = 12 );
        void pidTurn( double target, double maxVel, int breakoutCount = 12 );
};

#endif // End of File //