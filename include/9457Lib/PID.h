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

#ifndef PID_H
#define PID_H
#include "vex.h"        // The vex functions - to call the standard vex functions

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

#endif // PID_H;