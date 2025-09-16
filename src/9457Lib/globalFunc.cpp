# include "9457Lib.h"

using namespace vex; // using namespace vex helps to type less

/*
███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
█████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝


--- ODOMETRY THREAD CALLBACK---
*/

void odomTrackCall(botOdom *botObject, rotation *verticalDW, rotation *horizontalDW, inertial *imuObject, bool printData){
    if(printData){
      printf("Global Coordinates [Time, X, Y, T]:\n");
      double globalTime = 0.00;
      while(true){
        // Insert for your robot accordingly for the thread
        // --- Updating the Position and heading of the robot (use custom "update" for your system)
        botObject->trackLocation(verticalDW->position( degrees ), horizontalDW->position( degrees ), imuObject->rotation( degrees ) );
  
        // --- Print the position and heading of the robot for debugging purposes
        printf("%.2f, %.2f, %.2f, %.2f \n", globalTime, botObject->xG, botObject->yG, (botObject->tG));
  
        // --- Sleep the task for accurate update tracking.
        task::sleep(1000/systemHz);
        globalTime += 1/systemHz;
      }
    }
    else{
      while(true){
        // Insert for your robot accordingly for the thread
        // --- Updating the Position and heading of the robot (use custom "update" for your system)
        botObject->trackLocation(verticalDW->position( degrees ), horizontalDW->position( degrees ), imuObject->rotation( degrees ) );
  
        // --- Sleep the task for accurate update tracking.
        task::sleep(1000/systemHz);
      }
    }
  }

  int DEG2RAD(double degrees) {
    return degrees/180.0*M_PI;
  } 
  
  int RAD2DEG(double radians) {
    return radians/M_PI*180.0;
  }

void AsyncMotor(){ // Asynchonous Motor Control
  
}

  /*
  vector<vector<double>> linInject (vector<double> startCoords, vector<double> endCoords, double division ) {
    double m = (endCoords[1] - startCoords[1])/(endCoords[0] - startCoords[0]);
    double b = startCoords[1] - m*startCoords[0];
    double dx = (endCoords[0] - startCoords[0])/division;
    
    vector<vector<double>> pathCoords;
    double tempX = startCoords[0];
    double tempY = startCoords[1];
    vector<double> tempPosition = {tempX, tempY};
    
    for (int iX = 1; iX <= division; iX++) {
      tempX += dx;
      tempY = m*tempX - b;
      //printf("lin, Step %i --- X: %f, Y: %f \n", iX, tempX, tempY);
      tempPosition = {tempX, tempY};
      pathCoords.push_back(tempPosition);
    }
  
    return pathCoords;
  }
  
  vector<vector<double>> bezInject (vector<double> startCoords, vector<double> controlCoords1, vector<double> controlCoords2, vector<double> endCoords, double division) {
    vector<vector<double>> pathCoords;
    double tempX = startCoords[0];
    double tempY = startCoords[1];
    vector<double> tempPosition = {tempX, tempY};
    double t = 1/division;
    
    for(int iX = 1; iX <= division; iX++) {
      tempX = pow(1-t,3)*startCoords[0] + 3*pow(1-t, 2)*t*controlCoords1[0] + 3*(1-t)*pow(t,2)*controlCoords2[0] + pow(t, 3)*endCoords[0];
      tempY = pow(1-t,3)*startCoords[1] + 3*pow(1-t, 2)*t*controlCoords1[1] + 3*(1-t)*pow(t,2)*controlCoords2[1] + pow(t, 3)*endCoords[1];
      
      //printf("bez4, Step %i, t:%.2f  --- X: %f, Y: %f \n", iX, t, tempX, tempY);
      
      t += 1/division;
      tempPosition = {tempX, tempY};
      pathCoords.push_back(tempPosition);
    }
    
    return pathCoords;
  }
  
  double findVecDist(vector<double> p1, vector<double> p2) {
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
  }
  
  double findVecDet(vector<double> p1, vector<double> p2) {
    return sqrt(p1[0]*p2[1] - p2[0]*p1[1]);
  }
  
  int sgn( double val ) {
    if (val >= 0) { return  1; }
    else          { return -1; }
  }
    
  void pursuePath (vector<double> currPos, vector<vector<double>> path, double velocity, double lookAhead) {
    for (int iX = 0; iX < path.size(); iX++) {
      bool intersectFound = false;
      double x1_offset = path[iX][1] - currPos[1];
      double y1_offset = path[iX][2] - currPos[2];
      double x2_offset = path[iX+1][1] - currPos[1];
      double y2_offset = path[iX+1][2] - currPos[2];
      
      double dx = x2_offset - x1_offset;      
      double dy = y2_offset - y1_offset;
      double dr = sqrt(pow(dx, 2) + pow(dy, 2));
      double D = x1_offset*y2_offset - x2_offset*y1_offset;
      double discriminant = pow(lookAhead, 2) * pow(dr, 2) - pow(D, 2);
      
      if (discriminant >= 0){
        intersectFound = true;
          
        double x1Sol = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
        double x2Sol = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
        double y1Sol = (- D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
        double y2Sol = (- D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);  
          
        printf("Disc: %f \t pt1: [%f, %f] \t pt2: [%f, %f] \n", discriminant, x1Sol, y1Sol, x2Sol, y2Sol);
      }
    }
  }
    */