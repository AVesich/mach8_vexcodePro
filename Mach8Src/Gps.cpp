#include "vex.h"

using namespace mh8_Variables;

#define INCH_PER_M 39.3701

void mh8_Drivetrain::mh8_initGps(double xOffset, double yOffset, double rotation) {
  mh8Gps.setOrigin(xOffset, yOffset); // Set the offset from the point of rotation
  mh8Gps.resetRotation(); // Reset the preset rotation
  mh8Gps.setRotation(rotation, rotationUnits::deg); // Set the sensor's rotation
}

void mh8_Drivetrain::mh8_driveToCoord(double x, double y, double angle, double maxTurnSp, bool reversed) {
  // Turn
  mh8_turnWithGPS(angle, maxTurnSp);
  wait(100, msec);

  // Drive to the target
  double initX = mh8Gps.xPosition();
  double initY = mh8Gps.yPosition();
  //cout << "init X: " << initX << endl;
  //cout << "init Y: " << initY << endl;

  double delX = deltaX(initX, (x*1000));
  double delY = deltaY(initY, (y*1000));

  // Determine whether we are far enough away from the target to require driving to it
  if (fabs(delX) < 0.01 && fabs(delY) < 0.01) {
    return;
  }

  double distMMeters = distanceBetween(delX, delY); // Convert mm to m
  double distInches = distMMeters/25.4;
  Brain.Screen.clearScreen();
  Brain.Screen.print(distInches);

  char driveDir;
  if (reversed)
    driveDir = 'b';
  else
    driveDir = 'f';

  /*while (true) {
    Brain.Screen.clearLine(1);
    Brain.Screen.print(mh8Gps.yaw());
   wait(20, msec);
  }*/

  mh8_DriveStraight(distInches, 500, driveDir);
}

void mh8_Drivetrain::mh8_turnWithGPS(double angle, double maxTurnSp) {

  // Determine which way to turn
  char dir; // Init value
  double currHeading = mh8Gps.heading(); // Current heading - updated every iteration
  double initHeading = currHeading; // Init heading - never updated

  double shortestAngle = fmod((angle-currHeading+540.0), 360.0)-180.0;
  double initShortestAngle = shortestAngle;

  if (shortestAngle < 0) { // Negative = counterclockwise (left turn)
    dir = 'l';
  } else if (shortestAngle > 0) { // Positive = clockwise (right turn)
    dir = 'r'; // Make a right turn
  } else { // Default to left turn if there is an error
    dir = 'l';
  }
  //const double GPSTURNCONST = 0.8888888;
  mh8_resetDrive();

  double speed = 0.0;

  while (fabs(shortestAngle) > 1) {
    currHeading = mh8Gps.heading(); // Current heading - updated every iteration
    shortestAngle = fmod((angle-currHeading+540.0), 360.0)-180.0;

    speed = shortestAngle*(100/initShortestAngle); // Multiplies the shortest angle by 100 divided by the initial calculated shortest angle so that the drive starts at 100 and will gradually get lower as the target is neared
    if (speed < 15);
      speed = 15;

    if (shortestAngle < 0) { // Negative = counterclockwise (left turn)
      mh8_driveLeft(-speed);
      mh8_driveRight(speed);
    } else if (shortestAngle > 0) { // Positive = clockwise (right turn)
      mh8_driveLeft(speed);
      mh8_driveRight(-speed);
    } else { // Default to not turn if there is an error

    }
  }

  mh8_resetDrive();

  //shortestAngle *= 1.13*(M_PI*WHEEL_SIZE/2);

  //mh8_Turn(shortestAngle, maxTurnSp, dir);
}

double mh8_Drivetrain::deltaX(double x1, double x2) {
  double delX = x2-x1;
  
  //cout << "delta X: " << delX << endl;
  return delX;
}

double mh8_Drivetrain::deltaY(double y1, double y2) {
  double delY = y2-y1;
  //cout << "delta Y: " << delY << endl;
  return delY;
}

double mh8_Drivetrain::distanceBetween(double delX, double delY) {
  double distance = sqrt(pow(delX, 2)+pow(delY, 2));
  //cout << "distance: " << distance << endl;
  return distance;
}
