#ifndef CONFIG_H
#define CONFIG_H

using namespace vex;

namespace mh8_Variables {
  // Drivetrain motor ports
  const int L_FRONT = 3;
  const bool L_FRONT_REV = true;

  const int L_BACK = 4;
  const bool L_BACK_REV = true;

  const int R_FRONT = 13;
  const bool R_FRONT_REV = false;

  const int R_BACK = 14;
  const bool R_BACK_REV = false;

  // Drivetrain variables
  const double WHEEL_SIZE = 4.00; // Robot's wheel size
  const double GEAR_RATIO = 2.33333333333; // External gear ratio compensation (gear on motor/gear on wheel)

  const double TURN_CONST = 1.5; // Turn constant

  const double FRICTION = 1; // 1 is standard, higher than 1 used for low friction, lower than 1 used for high friction
}

#endif