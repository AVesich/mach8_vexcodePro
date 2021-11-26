#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "Mach8Inc/Config.h"

using namespace mh8_Variables;

namespace mh8_Drive {
  class mh8_Drivetrain {
    // Motor declarations
    /*motor lFront = motor(3, ratio6_1, true);
    motor rFront = motor(13, ratio6_1, true);
    motor lBack = motor(4, ratio6_1, true);
    motor rBack = motor(14, ratio6_1, true);*/

    public:
      // Basic control
      void mh8_driveLeftVolt(double vltg);
      void mh8_driveRightVolt(double vltg);
      void mh8_driveLeft(double pct);
      void mh8_driveRight(double pct);

      // DriveOp
      void mh8_tankDrive(int lStick, int rStick);
      void mh8_arcadeDrive(int xInput, int yInput);

      // Sensor-based driving
      void mh8_driveToObject(float maxPower, float curveTime, double sensitivity);

      // Normal inch-based funcs
      void mh8_DriveStraight(double inches, double maxRpm, char dir);
      void mh8_Turn(double deg, int maxSp, char dir);
      void mh8_Arc(double lInches, double rInches, double lSpeed, double rSpeed, char dir);

      // Gps-based funcs
      void mh8_initGps(double xOffset, double yOffset, double rotation);

      void mh8_driveToCoord(double x, double y, double angle, double maxTurnSp, bool reversed);
      void mh8_turnWithGPS(double angle, double maxTurnSp);
      double deltaX(double x1, double x2);
      double deltaY(double y1, double y2);
      double distanceBetween(double delX, double delY);
      //double angleBetween(double delX, double delY);

      // Utility
      int mh8_getAvgDriveSideDeg(char side);
      void mh8_resetDrive();
      void mh8_setBrake(char mode);
      bool driving();
  };
}

#endif