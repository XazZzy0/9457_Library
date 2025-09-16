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

#ifndef CHASSIS_H
#define CHASSIS_H
#include "9457Lib/botOdom.h"    // The odom class - to track the position of the robot
#include "9457Lib/controlMotor.h"         // The PID class - to control a single motor or motor group with PID

class chassis { 
    private:
        vex::motor_group *leftDB, *rightDB;            // a motor_group containing the left and right side of the drivebase
        vex::inertial *IMU;                            // an inertial class containing the IMU info
        botOdom *ODOM;                            // an inertial class
        double drivePID[3] = {0.0, 0, 0.0};       // the PID Coeff storage for driving manuevers.
        double turnPID[3] =  {0.0, 0.0, 0.0};     // the PID Coeff storage for turning manuevers.
        double headingPID[3] = {0.0, 0.0, 0.0};   // The PID Coeff storage for heading manuevers.
        double swingPID[3] = {0.0, 0.0, 0.0};     // The PID Coeff storage for arc/swing manuevers.
        double updateRate = NAN;                    // the update rate of the system.

    public:
        chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, const double updateRate ); // Constructor for the Chassis object - No IMU
        chassis( vex::motor_group *leftGroup, vex::motor_group *rightGroup, vex::inertial *botIMU, const double updateRate ); // Constructor for the Chassis object

        void setBrake( vex::brakeType type = vex::brakeType::brake ); // Set the brake type of your robot. [0 = coast, 1 = brake (default), 2 = hold];
        void setODOM(botOdom *botODOM);  // pointer to the odometry objet, can be used for pure pursuit.                                           
        void initialize( void );  // Function to be used to initalize the chassis
        void setDrivePID(double pTerm, double iTerm, double dTerm); // PID gain response to driving
        void setTurnPID(double pTerm, double iTerm, double dTerm); // PID gain response to turning
        void setHeadingPID(double pTerm, double iTerm, double dTerm); // PID gain response to heading changes
        void setSwingPID(double pTerm, double iTerm, double dTerm); // PID gain response to swing/arc turns           

        void testSpin();        // For debugging purposes
        void driveFwd( double dist, double vel, double minVel = 3, int brakoutCount = 8, bool waitForCompletion = true ); // The PID drive forward command 
        void driveAccel( double dist, double vel, double accelPeriod = 15, double minVel = 3, int breakoutCount = 8, bool waitForCompletion = true); // The PID accelerate command 
        void pointTurn( double degrees, double vel, double minVel = 3, int breakoutCount = 12, bool waitForCompletion = true); // The PID point turn command 
        void swingTurn( vex::turnType dir, double toTic, double vel, bool waitForCompletion = true );
        void diffDrive( double leftPower, double rightPower, double tics);

        void arcadeDrive( vex::controller *Controller, float deadband = 5.0 );                  // Default Arcade Control for driving
        void arcadeDrive( vex::controller *Controller, float spline, float deadband = 5.0 );    // Accel based Arcade Control for driving - customizable!
        void tankDrive( vex::controller *Controller, float deadband = 5.0 );                    // Default Tank Control for driving
};

#endif // CHASSIS_H