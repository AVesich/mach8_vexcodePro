#include "vex.h"

using namespace mh8_Variables;

// Basic control
//mh8_Drivetrain::
// Voltage-based
void mh8_Drivetrain::mh8_driveLeftVolt(double vltg) {
  lFront.spin(directionType::fwd, vltg, voltageUnits::mV);
  lBack.spin(directionType::fwd, vltg, voltageUnits::mV);
}
void mh8_Drivetrain::mh8_driveRightVolt(double vltg) {
  rFront.spin(directionType::fwd, vltg, voltageUnits::mV);
  rBack.spin(directionType::fwd, vltg, voltageUnits::mV);
}

// Pct-based
void mh8_Drivetrain::mh8_driveLeft(double pct) {
  lFront.spin(directionType::fwd, pct, velocityUnits::pct);
  lBack.spin(directionType::fwd, pct, velocityUnits::pct);
}
void mh8_Drivetrain::mh8_driveRight(double pct) {
  rFront.spin(directionType::fwd, pct, velocityUnits::pct);
  rBack.spin(directionType::fwd, pct, velocityUnits::pct);
}

// DriveOp
void mh8_Drivetrain::mh8_tankDrive(int lStick, int rStick) {
  mh8_driveLeftVolt(lStick);
  mh8_driveRightVolt(rStick);
}
void mh8_Drivetrain::mh8_arcadeDrive(int xInput, int yInput) {
  mh8_driveLeftVolt(yInput+xInput);
  mh8_driveRightVolt(yInput-xInput);
}

// Sensor-based driving
/*
void mh8_Drivetrain::mh8_driveToObject(float maxPower, float curveTime, double sensitivity) {
  mh8_resetDrive();
  mh8_setBrake('b'); // Set brake mode, 0 is coast, 1 is brake, 2 is hold
  int lAvgTicks = 0;
  int rAvgTicks = 0;
  int avgTicks = 0;
  float currentPower = 0;
  float lPower = 0;
  float rPower = 0;
  float alignErr = 0;
  const float alignKp = 0; // Proportional constant used to keep robot straight.

  float lastTime = Brain.Timer.time(msec);
  float deltaTime = 0;          // Time between loop runs
  float total_Dt = 0;           // Total deltaTime
  float pctComplete = 0;        // Percent completion of the time

  float distToObj = mh8Dist.objectDistance(mm);  // distance from tower
  const float deccelMM = 650.0;       // distance, in mm, from the tower where the bot should deccelerate

  auto atObj = mh8Dist.isObjectDetected() && mh8Dist.objectDistance(mm) < 125; // Should stop at around 125
  while( !atObj )
  {
    lAvgTicks = abs(mh8_getAvgDriveSideDeg('l'));
    rAvgTicks = abs(mh8_getAvgDriveSideDeg('r'));

    // Accel curve
    deltaTime = (Brain.Timer.time(msec)-lastTime);
    lastTime = Brain.Timer.time(msec);
    total_Dt += deltaTime;
    pctComplete = total_Dt/curveTime;
    currentPower = pctComplete*maxPower;

    if (currentPower <= 10) // Make sure the motor power is not 0
      currentPower = 10;

    //Decide which side is too far ahead, apply alignment and speed corretions.
    alignErr = 0;//abs((lAvgTicks - rAvgTicks)) * alignKp;
    if(lAvgTicks > rAvgTicks)
    {
      lPower = (currentPower) - alignErr;
      rPower = currentPower;
    }
    else if(rAvgTicks > lAvgTicks)
    {
      rPower = (currentPower) - alignErr;
      lPower = currentPower;
    }
    else
    {
      lPower = currentPower;
      rPower = currentPower;
    }

    // Decel
    if (mh8Dist.isObjectDetected() && mh8Dist.objectDistance(mm) < deccelMM) { // Decel if an object is detected and we are closer than the decel threshold
      lPower *= distToObj/deccelMM;
      rPower *= distToObj/deccelMM;
    }

    if (lPower < 0 || rPower < 0) {
      lPower = 0;
      rPower = 0;
    }

    //Send velocity targets to both sides of the drivetrain.
    mh8_driveLeft(lPower);
    mh8_driveRight(rPower);

    atObj = mh8Dist.isObjectDetected()  && mh8Dist.objectDistance(mm) < 125; // Update location status
    distToObj = mh8Dist.objectDistance(mm); // Update distance value

    wait(10, msec); // Save brain resources
  }
  mh8_resetDrive();
  return;
}
*/
void mh8_Drivetrain::mh8_driveToObject(float maxPower, float curveTime, double sensitivity) {}

// Normal inch-based funcs
void mh8_Drivetrain::mh8_DriveStraight(double inches, double maxRpm, char dir) {
  mh8_setBrake('b');
  mh8_resetDrive();
  const int wheelDiam = 4;//Should be set to the diameter of your drive wheels in inches.
  const int target = (inches / (wheelDiam * M_PI)) * 360 * 2.27;
  int lAvgTicks = 0;
  int rAvgTicks = 0;
  int avgTicks = 0;
  float currentPower = 0;
  float lPower = 0;
  float rPower = 0;
  float distErr = 0;
  float alignErr = 0;
  const float distKp = 0.35;//0.35;//Proportional constant used to control speed of robot.
  const float alignKp = 0.18;//Proportional constant used to keep robot straight.
  const float kDecel = 3.25;//3.45;//Constant used to determine when to decelerate.
  const float SLEW = .003;//Constant used to control acceleration in RPM/cycle.
  while(avgTicks < target)
  {
    lAvgTicks = abs(mh8_getAvgDriveSideDeg('l'));
    rAvgTicks = abs(mh8_getAvgDriveSideDeg('r'));
    avgTicks = (lAvgTicks + rAvgTicks) / 2;

    //Make sure we dont accelerate/decelerate too fast with slew.
    distErr = (target - avgTicks) * distKp;
    if(distErr > SLEW)
    {
      distErr = SLEW;
    }

    //Decide wether to accelerate or decelerate.
    if(currentPower * kDecel > (target - avgTicks))
    {
      distErr = distErr * -1;
      if(currentPower < 5)
      {
        distErr = 5 - currentPower;
      }
    }

    //Decide which side is too far ahead, apply alignment and speed corretions.
    alignErr = abs((lAvgTicks - rAvgTicks)) * alignKp;
    if(lAvgTicks > rAvgTicks)
    {
      lPower = (currentPower + distErr) - alignErr;
      rPower = currentPower + distErr;
    }
    else if(rAvgTicks > lAvgTicks)
    {
      rPower = (currentPower + distErr) - alignErr;
      lPower = currentPower + distErr;
    }
    else
    {
      lPower = currentPower + distErr;
      rPower = currentPower + distErr;
    }

    //Check what direction we should go, change motor velocities accordingly.
    if(dir == 'b')
    {
      lPower = lPower * -1;
      rPower = rPower * -1;
    }

    //Send velocity targets to both sides of the drivetrain.
    lFront.spin(directionType::fwd, lPower, velocityUnits::rpm);
    lBack.spin(directionType::fwd, lPower, velocityUnits::rpm);
    rFront.spin(directionType::fwd, rPower, velocityUnits::rpm);
    rBack.spin(directionType::fwd, rPower, velocityUnits::rpm);

    //Set current power for next cycle, make sure it doesn't get too high/low.
    /*As a side note, the distance(in ticks) at which deceleration starts is
      determined by the upper limit on currentPower's and kDecel's product.*/
    currentPower = currentPower + distErr;
    if(currentPower > maxRpm)
    {
      currentPower = maxRpm;
    }
    else if(currentPower < 35)
    {
      currentPower = 35;
    }
  }
  mh8_resetDrive();
  return;
}

void mh8_Drivetrain::mh8_Turn(double deg, int maxSp, char dir) {
  double target = deg; // Put calculation here to make the robot actually go the inputted inches

  double leftTarget = target;
  double rightTarget = target;

  if (dir == 'r') { // If going right
    leftTarget = leftTarget;
    rightTarget = -rightTarget;
  } else { // If turning left
    leftTarget = -leftTarget;
    rightTarget = rightTarget;
  }

  // Start the motor rotation
  lFront  .startRotateFor(leftTarget, rotationUnits::deg, maxSp, velocityUnits::pct);
  lBack   .startRotateFor(leftTarget, rotationUnits::deg, maxSp, velocityUnits::pct);
  rFront  .startRotateFor(rightTarget, rotationUnits::deg, maxSp, velocityUnits::pct);
  rBack   .startRotateFor(rightTarget, rotationUnits::deg, maxSp, velocityUnits::pct);

  while (driving()) {} // Do nothing until the drive stops moving
}

void mh8_Drivetrain::mh8_Arc(double lInches, double rInches, double lSpeed, double rSpeed, char dir) {
  double leftTarget = lInches; // Put calculation here to make the robot actually go the inputted inches
  double rightTarget = rInches; // Put calculation here to make the robot actually go the inputted inches

  if (dir == 'f') {
    // If going forward, do not modify the target inches
  } else {
    leftTarget = -leftTarget;
    rightTarget = -rightTarget;
  }

  // Start the motor rotation
  lFront  .startRotateFor(leftTarget, rotationUnits::deg, lSpeed, velocityUnits::rpm);
  lBack   .startRotateFor(leftTarget, rotationUnits::deg, lSpeed, velocityUnits::rpm);
  rFront  .startRotateFor(rightTarget, rotationUnits::deg, rSpeed, velocityUnits::rpm);
  rBack   .startRotateFor(rightTarget, rotationUnits::deg, rSpeed, velocityUnits::rpm);

  while (driving()) {} // Do nothing until the drive stops moving
}

// Utility
int mh8_Drivetrain::mh8_getAvgDriveSideDeg(char side) {
  if (side == 'l') { // If returning left
    return ((lFront.rotation(rotationUnits::deg) + lBack.rotation(rotationUnits::deg))/2);
  } else { // Else, return right
    return ((rFront.rotation(rotationUnits::deg) + rBack.rotation(rotationUnits::deg))/2);
  }
}
void mh8_Drivetrain::mh8_resetDrive() {
  // Reset encoders
  lFront.resetRotation();
  lBack.resetRotation();
  rFront.resetRotation();
  rBack.resetRotation();
  // Stop the motors
  lFront.stop();
  lBack.stop();
  rFront.stop();
  rBack.stop();
}
void mh8_Drivetrain::mh8_setBrake(char mode) {
  brakeType m_brakeMode;
  switch(mode){
    case 'c': case 'C':
      m_brakeMode = coast;
      break;
    case 'b': case 'B':
      m_brakeMode = brake;
      break;
    case 'h': case 'H':
      m_brakeMode = hold;
      break;
  }
  lFront.setBrake(m_brakeMode);
  lBack.setBrake(m_brakeMode);
  rFront.setBrake(m_brakeMode);
  rBack.setBrake(m_brakeMode);
}

bool  mh8_Drivetrain::driving() {
  if ((lFront.isSpinning() || lBack.isSpinning()) || (rFront.isSpinning() || rBack.isSpinning()))
    return true;
  else
    return false;
}