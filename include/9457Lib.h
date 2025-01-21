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
        // Variables
        // settings for the PID
        double Kp = 0.0, Ki = 0, Kd = 0.0;        //coefficients
        double integral = 0.0, prevError = 0.0;   //integral, previous error (for "I" and "D" coeff.)
        double maxPower = 100.0;                  // max output of the motors (typically 100%)

        // Methods - EMPTY
    public:
        // Initalizing external access variables
        double Pterm = 0.0, Iterm = 0.0, Dterm = 0.0;
        
        // Methods
        PID(double kp, double ki, double kd);

        double calculate ( double error );
        void reset( void );
        void setVel( double toPower );
};

/** ==============================================================================================================================================================================
 * 
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
        inertial IMU;

        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]

    public:
        // creating neccessary variables for the class - updating and global/previous variables.
        double update_hz;
        double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]
        
        // Methods
        botOdom( inertial IMU_init );
        botOdom( double v_Offset, double h_Offset, inertial IMU_init );
        botOdom( bool wheelPerp, inertial IMU_init );
        botOdom( double LeftOffset, double RightOffset, double RearOffset, inertial IMU_init );

        void update( double vWheel, double hWheel, double angle, double vWheel_Diameter = 2.75,  double hWheel_Diameter = 2.75 );
        void update_2( double hWheel, double angle, double hWheel_Diameter = 2.75 );
        void update_3( double vWheelL, double vWheelR, double hWheel, double vWheelL_Diameter = 2.75, double vWheelR_Diameter = 2.75, double hWheel_Diameter = 2.75 );
        void setPose( double xPose, double yPose, double tPose );
        void change_rate( double rate_hz );
};

/** ==============================================================================================================================================================================
 * @class controlDrive
 * @details 
 * 
 * [Insert description here]
 * ==============================================================================================================================================================================
 */
class controlDrive { // Used for pathing manuevers with Odometry
    private:
        motor_group left;   // a motor_group containing the left side of the drivebase
        motor_group right;  // a motor_group containing the right side of the drivebase
        inertial IMU;       // an inertial class containing the IMU info

    public:
        controlDrive( motor_group leftGroup, motor_group rightGroup, inertial botIMU );

        void driveFwd( double dist, double vel, bool waitCompletion = true );
        void pointTurn(double degrees, double vel, bool waitCompletion = true);
};

#endif // End of File //