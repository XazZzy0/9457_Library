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

#ifndef BOTODOM_H
#define BOTODOM_H
#include "vex.h"        // The vex functions - to call the standard vex functions

class botOdom {
    private:
        vex::inertial *IMU;                                              // A pointer which holds the vex imu class - pointers make it so that you do not have to "copy" the entire class into a function.

        double vOffset = 0, hOffset = 0;                                 // Variables to represent the deadwheel offsets
        double vPrev = 0, vLPrev = 0, vRPrev = 0, hPrev = 0, tPrev = 0;  // Variables to represent the previous encoder positions
        double vdot = 0, hdot = 0, tdot = 0;                             // Variables for the rotational velocity deltas, (deg/hz, deg/hz, Rad/hz) [ROBOT FRAME]
        double update_hz = NAN;                                          // The update rate of the robot

    public:
        vex::rotation *vWheel, *vLWheel, *vRWheel, *hWheel;              // pointers which hold the rotation wheels - can leave empty(null) if they do not exist!
        bool isCalibrated = false;                                       // Verifies bot initalization - is false if calibration doesn't occur.
        double vWheel_Diameter = 0, hWheel_Diameter = 0;                 // Variables which specify the vert/horizontal wheel diameters
        double baseWidth = 0, baseLength = 0;                            // Variables which represent the Drivebase Length/Width
        double xG, yG, tG; // (In, In, Rad) [INERTIAL FRAME]             // The Global X, Y, and Theta location.
        
        botOdom( void );                                                                                                            // Empty Odom Constructor - mainly for testing
        botOdom( vex::rotation *vertWheel, double offset_V, vex::rotation *horWheel, double offset_H, vex::inertial *setIMU, const double updateRate );      // Primary Odom Constructor - 2 Deadwheels with offsets from the tracking center

        void initializeSystem( void );                                       // Used as an initialization function - change as you see fit.

        void setVerticalDiameter( double Diameter );                         // Set the vertical wheel diameter
        void setHorizontalDiameter( double Diameter );                       // Set the horizontal wheel diameter
        void setBotSize( double width, double length );                      // Set the bot sizing.
        void setPose( double xPose, double yPose, double tPose );            // Set the X, Y, Theta position

        void trackLocation( double vWheel, double hWheel, double angle );    // The callback to track the location.
};

#endif // BOTODOM_H