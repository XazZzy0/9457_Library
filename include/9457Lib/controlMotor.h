/** ==============================================================================================================================================================================
 * @class controlMotor
 * @details 
 * 
 * 
 * This is how you would control a single motor or motor group with a PID function. It is similar to the chassis class but with a single motor or a group of motors.
 * Examples for how this would be used are for things such as a lift mechanism, or an arm which you need to keep at a certain angle.
 * ==============================================================================================================================================================================
 */


#ifndef CONTROLMOTOR_H
#define CONTROLMOTOR_H
#include "9457Lib/PID.h"       // The PID class - to control a single motor or motor group with PID

class controlMotor {
    private:
        vex::motor *refMotor;                               // pointer to specific motor which will be controlled
        vex::motor_group *refGroup;                         // pointer to specific motor group which will be controlled
        vex::rotation *refEncoder;                          // pointer to the specific encoder.
        
        double PID_Coef[3] = {0.0f, 0.0f, 0.0f};            // the PID Coeff storage for the specific manuever. 
        double updateRate = NAN;                            // the update Rate of the controller (max is 100 hz)
        double tolBound_seconds = 0.1;                      // the time it takes to break out of the PID loop (default is 0.1 seconds)
        double tolBound_hz = updateRate*tolBound_seconds;   // the time it takes to break out of the PID loop (default is 0.1 seconds)
        double tolBound_degrees = 5;                       // the tic offset bound of the target -> larger = easier to breakout (default is 5 degrees)

    public:
        controlMotor( vex::motor *ptrMotor, const double sysRate );                                // Constructor for a single motor
        controlMotor( vex::motor *ptrMotor, vex::rotation *ptrRot, const double sysRate );         // Constructor for a single motor attached to a rotation sensor
        controlMotor( vex::motor_group *ptrGroup, const double sysRate );                          // Constructor for a single motor group
        controlMotor( vex::motor_group *ptrGroup, vex::rotation *ptrRot, const double sysRate );   // Constructor for a single motor group attached to a rotation sensor

        void setBrake( vex::brakeType type = vex::brakeType::brake ); // Set the brake type of your motor(s). [0 = coast, 1 = brake, 2 = hold]; - this is reduntant code for ease (compared to vex commands)
        void setPID( double pTerm, double iTerm, double dTerm ); // Set the default PID variables
        void setTolBound( double seconds, double degrees ); // Set the default tolerance bounds for the PID loop (default is 0.25 seconds and 10 degrees)
        
        void testSpin( void ); // a test spin function
        void pidRotate( double target, double maxVel, double minVel = 0.0); // a PID rotate command
        void pidAccel( double target, double maxVel, double minVel = 0.0, double accelPeriod = 15.0 ); // a PID acceleration command
        void pidTurn( double target, double maxVel, double minVel = 0.0 ); // a PID turn command
};

#endif // CONTROLMOTOR_H