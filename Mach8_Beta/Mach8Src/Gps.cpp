#include "vex.h"

using namespace mh8_Variables;
using namespace mh8_Math;

#define INCH_PER_M 39.3701

void mh8_Drivetrain::initGps(double xOffset, double yOffset, double rotation) {
  mh8Gps.setOrigin(xOffset, yOffset); // Set the offset from the point of rotation
  mh8Gps.resetRotation(); // Reset the preset rotation
  mh8Gps.setRotation(rotation, rotationUnits::deg); // Set the sensor's rotation
}

void mh8_Drivetrain::driveToCoord(double x, double y, double angle, double maxTurnSp, bool reversed) {
  // Turn
  turnWithGPS(angle, maxTurnSp);
  wait(100, msec);

  // Drive to the target
  double initX = mh8Gps.xPosition();
  double initY = mh8Gps.yPosition();

  double delX = Delta(initX, (x*1000));
  double delY = Delta(initY, (y*1000));

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

  driveStraight(distInches, 500, driveDir);
}

void updatedDriveToCoord(double x, double y, double drivePower) {
  // Drive to the target
  double initX = mh8Gps.xPosition();
  double initY = mh8Gps.yPosition();

  double delX = Delta(initX, (x*1000));
  double delY = Delta(initY, (y*1000));

  // Distance to drive to the point
  double distToTarget = distanceBetween(delX, delY);

  // Angle from 0,0 to the point
  double absoluteAngleToTarget = atan2(delY, delX)*(180/M_PI);

  double relAngleToPoint = 0;

   
}

void mh8_Drivetrain::turnWithGPS(double angle, double maxTurnSp) {

  // Determine which way to turn
  char dir; // Init value
  double currHeading = mh8Gps.heading(); // Current heading - updated every iteration

  double shortestAngle = fmod((angle-currHeading+540.0), 360.0)-180.0;
  double initShortestAngle = shortestAngle;

  if (shortestAngle < 0) { // Negative = counterclockwise (left turn)
    dir = 'l';
  } else if (shortestAngle > 0) { // Positive = clockwise (right turn)
    dir = 'r'; // Make a right turn
  } else { // Default to left turn if there is an error
    dir = 'l';
  }

  resetDrive();

  double speed = 0.0;

  while (fabs(shortestAngle) > 1) {
    currHeading = mh8Gps.heading(); // Current heading - updated every iteration
    shortestAngle = fmod((angle-currHeading+540.0), 360.0)-180.0;

    speed = shortestAngle*(100/initShortestAngle); // Multiplies the shortest angle by 100 divided by the initial calculated shortest angle so that the drive starts at 100 and will gradually get lower as the target is neared
    if (speed < 15)
      speed = 15;

    if (shortestAngle < 0) { // Negative = counterclockwise (left turn)
      driveLeft(-speed);
      driveRight(speed);
    } else if (shortestAngle > 0) { // Positive = clockwise (right turn)
      driveLeft(speed);
      driveRight(-speed);
    } else { // Default to not turn if there is an error

    }
  }

  resetDrive();
}