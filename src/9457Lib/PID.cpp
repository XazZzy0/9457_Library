#include "9457Lib/PID.h"

/*
██████╗ ██╗██████╗      ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗     ███████╗██████╗
██╔══██╗██║██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║     ██╔════╝██╔══██╗
██████╔╝██║██║  ██║    ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║     █████╗  ██████╔╝
██╔═══╝ ██║██║  ██║    ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ██║     ██╔══╝  ██╔══██╗
██║     ██║██████╔╝    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗███████╗███████╗██║  ██║
╚═╝     ╚═╝╚═════╝      ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝

Self Explanatory
*/
// Initialize outside of class definition
PID::PID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}

// Calculate the PID response (expects an error input, in degrees or pct manuever)
double PID::calculate( double error ) {
  Pterm = Kp * error;               // This is the P response - acts like a spring
  Iterm = Ki * integral;            // This is the I response - pushes error to 0 over time.
  Dterm = Kd * (error - prevError); // This is the D response - acts like a Dampener

 // The I variable sometimes has issues if it runs a command for a long time, here are some fixes which mitigate the I term from impacting too much:
 // Integral Fix 1: only count up when the error is less than the start I term;
 if (fabs(error) < initI)  { integral += error; }           // this is the I summation - totals the error over time
 // Integral Fix 2: Preventing integral windup - when the integral term gets too high and the system cannot respond to the next manuever because of the Iterm.
 if (integral > windup)      { integral = windup; }
 else if (integral < -windup) { integral = -windup; }
 // Integral Fix 3: Setting integral to 0 when error crosses zero - reduce the impact of the integral term.
 if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0))  { integral = 0; }
 
  double output = Pterm + Iterm + Dterm; // Sum all of the values - the PID response

  // clamps the output
  if (output > maxPower)          { output = maxPower; }
  else if (output < -maxPower)    { output = -maxPower; }

  // clamps the min speed if there is one
  if (minPower != 0){
    if (fabs(output) < minPower){
      if (signbit(output)) {output = -minPower;}
      else                 {output = minPower;}
    }
  }

  // sets the previous error to the new one
  prevError = error;

  // return the PID output 
  return output;
}

// Reset the PID internal variables
void PID::reset( void ){
  integral = 0;
  prevError = 0;
  return;
}

// Adjust the PID coefficients
void PID::adjPID(double kP, double kI, double kD){
  Kp = kP;
  Ki = kI;
  Kd = kD;
  return;
}

//adjust the windup behavior of the PID
void PID::adjWindup( double wTerm ) { windup = wTerm; }

//set the maximum and the minimum velocity output of the pid
void PID::setVel(double toMinPower, double toMaxPower){
  if (fabs(toMaxPower) > 100)  { toMaxPower = 100; } // Max speed is 100 pct
  if (fabs(toMinPower) > 100)  { toMinPower = 100; } // Min speed is 100 pct - don't do this please
  if (toMinPower > toMaxPower) { return; } 

  minPower = fabs(toMinPower); // No negative signs for power, standard is positive
  maxPower = fabs(toMaxPower); // No negative signs for power, standard is positive
  return;
}