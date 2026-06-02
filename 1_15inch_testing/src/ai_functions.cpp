/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "ai_functions.h"
#include <string>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
using namespace vex;
using namespace std;

bool intakemotorrunning;

// Robot constants
const double WHEEL_DIAMETER = 4.0;        // inches
const double GEAR_RATIO = 0.5;            // motor:wheel ratio
const double TRACK_WIDTH = 12.0;          // inches between left/right wheels (ADJUST TO YOUR ROBOT)
const double PI = 3.14159265358979323846;

ScoringPos getScoringPos(SCORING_LOCATIONS location) {
    switch (location) {
        case RED_HIGH_LEFT:   return {-40,   47,    270};
        case RED_HIGH_RIGHT:  return {-37,  -47,  270};
        case RED_MID_LEFT:    return {-16,   13.5,  311};
        case RED_MID_RIGHT:   return {-15.5, -18,    43};
        case BLUE_HIGH_LEFT:  return { 40, 49,     90};
        case BLUE_HIGH_RIGHT: return { 40, 47.5,   90};
        case BLUE_MID_LEFT:   return { 16,  -14,    135};
        case BLUE_MID_RIGHT:  return { 17,   20.5,  221};
    }
    return {0, 0, 0};
}

// Helper: wrap angle to [-180, 180]
double wrapAngle(double angle) {
    while (angle > 180.0)  angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void turnToAbsolute(double theta) {
  // PID constants - tune these for your robot
  double kP = 0.35;
  double kI = 0.03;
  double kD = 0.2;

  double integral = 0;
  double prevError = 0;
  double error = 0;

  // Tolerance settings
  double errorThreshold = 0.75;      // degrees
  double velocityThreshold = 1.5;   // deg/s of motor output to consider stopped
  int settleCount = 0;
  int settleTarget = 10;            // loops within tolerance before exiting

  double integralLimit = 50.0;      // anti-windup cap

  while (true) {
    // Compute error as shortest angular distance to target [-180, 180]
    double current = DrivetrainInertial.heading(degrees);
    error = theta - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    // Integral with anti-windup (only accumulate near target)
    if (fabs(error) < 15) {
      integral += error;
    } else {
      integral = 0;
    }
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    // Derivative
    double derivative = error - prevError;

    // PID output
    double output = (kP * error) + (kI * integral) + (kD * derivative);

    // Clamp output to motor voltage range
    if (output > 12) output = 12;
    if (output < -12) output = -12;

    // Apply: positive output turns clockwise (right)
    leftDriveSmart.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::rev, output, vex::voltageUnits::volt);

    // Check settle condition
    if (fabs(error) < errorThreshold && fabs(derivative) < velocityThreshold) {
      settleCount++;
      if (settleCount >= settleTarget) break;
    } else {
      settleCount = 0;
    }

    prevError = error;
    wait(10, msec);
  }

  leftDriveSmart.stop(brake);
  rightDriveSmart.stop(brake);
}

void turnToRelative(double theta) {
  // PID constants - tune these for your robot
  double kP = 0.35;
  double kI = 0.03;
  double kD = 0.2;

  double integral = 0;
  double prevError = 0;

  // Tolerance settings
  double errorThreshold = 0.75;      // degrees
  double velocityThreshold = 1.5;   // deg/s of motor output to consider stopped
  int settleCount = 0;
  int settleTarget = 10;            // loops within tolerance before exiting

  double integralLimit = 50.0;      // anti-windup cap

  // Snapshot starting heading and compute absolute target.
  // Positive theta = turn right (clockwise), negative = turn left.
  double startHeading = DrivetrainInertial.heading(degrees);
  double target = startHeading + theta;

  while (true) {
    double current = DrivetrainInertial.heading(degrees);
    double error = target - current;
    // Normalize to shortest angular distance [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    // Integral with anti-windup (only accumulate near target)
    if (fabs(error) < 15) {
      integral += error;
    } else {
      integral = 0;
    }
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    // Derivative
    double derivative = error - prevError;

    // PID output
    double output = (kP * error) + (kI * integral) + (kD * derivative);

    // Clamp to motor voltage range
    if (output > 12) output = 12;
    if (output < -12) output = -12;

    // Apply: positive output turns clockwise (right)
    leftDriveSmart.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::rev, output, vex::voltageUnits::volt);

    // Settle check
    if (fabs(error) < errorThreshold && fabs(derivative) < velocityThreshold) {
      settleCount++;
      if (settleCount >= settleTarget) break;
    } else {
      settleCount = 0;
    }

    prevError = error;
    wait(10, msec);
  }

  leftDriveSmart.stop(brake);
  rightDriveSmart.stop(brake);
}

void turnTo(double targetX, double targetY) {
    // PID constants for turning (tune these for your robot)
    double kP = 1.0;
    double kI = 0.02;
    double kD = 0.2;

    // Tolerances
    double HEADING_TOLERANCE = 1.0;     // degrees
    double INTEGRAL_LIMIT = 50.0;       // anti-windup
    double MIN_POWER = 2.0;             // minimum power to overcome static friction
    double MAX_POWER = 80.0;            // cap on motor power (%)
    double SETTLE_TIME_MS = 150;           // must be within tolerance for this long
    double TIMEOUT_MS = 2500;              // abort if taking too long

    // Compute desired absolute heading toward target point.
    // Using atan2(dx, dy) because heading 0 = +Y axis, CW positive.
    double dx = targetX - currX;
    double dy = targetY - currY;

    // If we're basically on the target, no meaningful heading exists.
    if (std::sqrt(dx*dx + dy*dy) < 0.1) return;

    double targetHeading = std::atan2(dx, dy) * 180.0 / PI;  // degrees, CW from +Y

    // PID state
    double integral = 0.0;
    double prevError = wrapAngle(targetHeading - currH);
    int settleCounter = 0;
    int elapsed = 0;
    const int dt = 10;  // ms per loop

    while (elapsed < TIMEOUT_MS) {
        // Compute error as the shortest angular distance
        double error = wrapAngle(targetHeading - currH);

        // Check settling
        if (std::fabs(error) < HEADING_TOLERANCE) {
            settleCounter += dt;
            if (settleCounter >= SETTLE_TIME_MS) break;
        } else {
            settleCounter = 0;
        }

        // Integral with anti-windup (only accumulate when close)
        if (std::fabs(error) < 15.0) {
            integral += error * (dt / 1000.0);
            if (integral >  INTEGRAL_LIMIT) integral =  INTEGRAL_LIMIT;
            if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
        } else {
            integral = 0;
        }

        // Derivative
        double derivative = (error - prevError) / (dt / 1000.0);
        prevError = error;

        // PID output
        double output = kP * error + kI * integral + kD * derivative;

        // Clamp magnitude
        if (output >  MAX_POWER) output =  MAX_POWER;
        if (output < -MAX_POWER) output = -MAX_POWER;

        // Minimum power to overcome friction (only when not essentially done)
        if (std::fabs(error) > HEADING_TOLERANCE && std::fabs(output) < MIN_POWER) {
            output = (output >= 0) ? MIN_POWER : -MIN_POWER;
        }

        // Apply to drive: positive output = turn clockwise (right)
        leftDriveSmart.spin(vex::directionType::fwd,   output, vex::velocityUnits::pct);
        rightDriveSmart.spin(vex::directionType::rev,  output, vex::velocityUnits::pct);

        task::sleep(dt);
        elapsed += dt;
    }

    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
}

// void turnTo(double targetX, double targetY) {
//     // ===== Speed profile =====
//     const double MIN_SPEED = 8.0;            // % at start and end
//     const double ABSOLUTE_MAX = 40.0;         // % cap on cruise speed for any turn
//     const double ACCEL_FRACTION = 0.3;       // first 15% of the turn: ramp up
//     const double DECEL_FRACTION = 0.3;       // last  15% of the turn: ramp down

//     // ===== Turn-size-proportional max speed =====
//     // Scale linearly from MIN_SPEED (at 0°) up to ABSOLUTE_MAX (at FULL_SPEED_TURN°).
//     // Beyond FULL_SPEED_TURN, cap at ABSOLUTE_MAX.
//     const double FULL_SPEED_TURN = 90.0;      // degrees needed to earn full cruise speed

//     // ===== Tolerances and limits =====
//     const double HEADING_TOLERANCE = 1.0;
//     const int    SETTLE_TIME_MS = 150;
//     const int    TIMEOUT_MS = 2500;
//     const int    dt = 10;

//     // ===== Compute desired absolute heading =====
//     double dx = targetX - currX;
//     double dy = targetY - currY;
//     if (std::sqrt(dx*dx + dy*dy) < 0.1) return;
//     double targetHeading = std::atan2(dx, dy) * 180.0 / PI;

//     // ===== Capture initial error — total turn we plan to make =====
//     double initialError = wrapAngle(targetHeading - currH);
//     double totalTurn = std::fabs(initialError);
//     if (totalTurn < HEADING_TOLERANCE) return;

//     // ===== Compute MAX_SPEED proportional to turn size =====
//     double MAX_SPEED = MIN_SPEED + (ABSOLUTE_MAX - MIN_SPEED) * (totalTurn / FULL_SPEED_TURN);
//     if (MAX_SPEED > ABSOLUTE_MAX) MAX_SPEED = ABSOLUTE_MAX;
//     if (MAX_SPEED < MIN_SPEED)    MAX_SPEED = MIN_SPEED;

//     // ===== Loop =====
//     int settleCounter = 0;
//     int elapsed = 0;

//     while (elapsed < TIMEOUT_MS) {
//         double error = wrapAngle(targetHeading - currH);

//         if (std::fabs(error) < HEADING_TOLERANCE) {
//             settleCounter += dt;
//             if (settleCounter >= SETTLE_TIME_MS) break;
//         } else {
//             settleCounter = 0;
//         }

//         // Progress through the planned turn
//         double turned = totalTurn - std::fabs(error);
//         if (turned < 0) turned = 0;
//         double progress = turned / totalTurn;
//         if (progress > 1.0) progress = 1.0;

//         // Trapezoidal speed
//         double speed;
//         if (progress < ACCEL_FRACTION) {
//             double t = progress / ACCEL_FRACTION;
//             speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * t;
//         } else if (progress > (1.0 - DECEL_FRACTION)) {
//             double t = (progress - (1.0 - DECEL_FRACTION)) / DECEL_FRACTION;
//             speed = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * t;
//         } else {
//             speed = MAX_SPEED;
//         }

//         // Direction follows the live sign of error (allows reversal on overshoot)
//         int liveDir = (error >= 0) ? 1 : -1;
//         double output = speed * liveDir;

//         if (output >= 0) {
//             leftDriveSmart.spin(vex::directionType::fwd,  output, vex::velocityUnits::pct);
//             rightDriveSmart.spin(vex::directionType::rev, output, vex::velocityUnits::pct);
//         } else {
//             leftDriveSmart.spin(vex::directionType::rev, -output, vex::velocityUnits::pct);
//             rightDriveSmart.spin(vex::directionType::fwd, -output, vex::velocityUnits::pct);
//         }

//         task::sleep(dt);
//         elapsed += dt;
//     }

//     leftDriveSmart.stop(brake);
//     rightDriveSmart.stop(brake);
// }

/*void driveFor(double dist) {
    // PID constants for driving (tune for your robot)
    double kP_drive = 2.0;
    double kI_drive = 0.0;
    double kD_drive = 0.1;

    // Tolerances and limits
    double DIST_TOLERANCE = 0.25;       // inches
    double INTEGRAL_LIMIT_D = 20.0;     // anti-windup (inches·sec)
    double MIN_POWER_D = 3.0;           // overcome static friction (%)
    double MAX_POWER_D = 90.0;          // cap motor power (%)
    int SETTLE_TIME_MS_D = 150;
    int TIMEOUT_MS_D = 4000;

    // Convert inches -> motor degrees.
    // Wheel circumference (inches per wheel revolution):
    double wheelCircumference = PI * WHEEL_DIAMETER;     // ~12.566 in
    // Wheel revolutions needed to cover `dist`:
    double wheelRevs = dist / wheelCircumference;
    // Motor revolutions = wheel revs / gear ratio (wheel:motor).
    // With GEAR_RATIO = 0.5 meaning the wheel turns 0.5 rev per motor rev (torque gearing),
    // motor must spin MORE than the wheel. So motorRevs = wheelRevs / 0.5 = wheelRevs * 2.
    double motorRevs = wheelRevs / GEAR_RATIO;
    double targetDegrees = motorRevs * 360.0;

    // Capture starting encoder position so this is relative, not absolute.
    double startPos = frontLeft.position(degrees);

    // PID state
    double integral = 0.0;
    double prevError = targetDegrees;
    int settleCounter = 0;
    int elapsed = 0;
    const int dt = 10;  // ms

    while (elapsed < TIMEOUT_MS_D) {
        double currentPos = frontLeft.position(degrees) - startPos;
        double error = targetDegrees - currentPos;

        // Convert error to inches just for the tolerance check (more intuitive)
        double errorInches = (error / 360.0) * GEAR_RATIO * wheelCircumference;

        if (std::fabs(errorInches) < DIST_TOLERANCE) {
            settleCounter += dt;
            if (settleCounter >= SETTLE_TIME_MS_D) break;
        } else {
            settleCounter = 0;
        }

        // Integral with anti-windup (only accumulate when reasonably close)
        if (std::fabs(errorInches) < 6.0) {
            integral += errorInches * (dt / 1000.0);
            if (integral >  INTEGRAL_LIMIT_D) integral =  INTEGRAL_LIMIT_D;
            if (integral < -INTEGRAL_LIMIT_D) integral = -INTEGRAL_LIMIT_D;
        } else {
            integral = 0;
        }

        // Derivative on error (in inches/sec)
        double derivative = (errorInches - ((prevError / 360.0) * GEAR_RATIO * wheelCircumference)) / (dt / 1000.0);
        prevError = error;

        // PID output (in % power)
        double output = kP_drive * errorInches + kI_drive * integral + kD_drive * derivative;

        // Clamp
        if (output >  MAX_POWER_D) output =  MAX_POWER_D;
        if (output < -MAX_POWER_D) output = -MAX_POWER_D;

        // Minimum power to overcome friction
        if (std::fabs(errorInches) > DIST_TOLERANCE && std::fabs(output) < MIN_POWER_D) {
            output = (output >= 0) ? MIN_POWER_D : -MIN_POWER_D;
        }

        // Apply: positive output drives forward, negative drives reverse
        if (output >= 0) {
            leftDriveSmart.spin(vex::directionType::fwd, output, percent);
            rightDriveSmart.spin(vex::directionType::fwd, output, percent);
        } else {
            leftDriveSmart.spin(vex::directionType::rev, -output, percent);
            rightDriveSmart.spin(vex::directionType::rev, -output, percent);
        }

        task::sleep(dt);
        elapsed += dt;
    }

    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
}*/

void forwardStraight(double fdistance) {
    double fbcoef = 43;  // convert inches to motor degrees
    double distanceEncoder = fbcoef * fdistance;
    double currentPos = 0;
    double posDifference;
    double instantV = 0;
    double thresholdBegin = 10 * fbcoef;
    double thresholdEnd   = 15 * fbcoef;
    double minSpeed = 130.0;
    double maxSpeed = 550.0;
    double currentAngle = DrivetrainInertial.heading(degrees);
    double angle = currentAngle;

    double AngleTolerance = 0.25;   // maintain angle +- 0.25 degrees
    double percentChange  = 1.1;

    // VEX has no STOP_AND_RESET_ENCODER + RUN_TO_POSITION. Reset encoders
    // and drive by velocity, ending the loop when we reach the target count.
    frontLeft.resetPosition();
    backLeft.resetPosition();
    frontRight.resetPosition();
    backRight.resetPosition();

    // Direction of travel (sign of the requested distance).
    double dir = (distanceEncoder >= 0) ? 1.0 : -1.0;

    while (fabs(currentPos) < fabs(distanceEncoder)) {
        currentPos   = backLeft.position(degrees);
        currentAngle = DrivetrainInertial.heading(degrees);

        // Trapezoidal speed profile: ramp up over the first half, down over
        // the second half, based on encoder distance from start / target.
        if (fabs(currentPos) < fabs(distanceEncoder / 2)) {
            posDifference = fabs(currentPos);
            if (posDifference > thresholdBegin) posDifference = thresholdBegin;
            instantV = (maxSpeed - minSpeed) * posDifference / thresholdBegin + minSpeed;
        } else {
            posDifference = fabs(distanceEncoder - currentPos);
            if (posDifference > thresholdEnd) posDifference = thresholdEnd;
            instantV = (maxSpeed - minSpeed) * posDifference / thresholdEnd + minSpeed;
        }

        // Heading correction. Velocities are in RPM here (VEX velocityUnits::rpm).
        if ((currentAngle - angle) > AngleTolerance) {
            // turn right a little bit
            backLeft.spin(vex::directionType::fwd,  dir * instantV * percentChange, vex::velocityUnits::rpm);
            backRight.spin(vex::directionType::fwd, dir * instantV,                 vex::velocityUnits::rpm);
            frontLeft.spin(vex::directionType::fwd, dir * instantV * percentChange, vex::velocityUnits::rpm);
            frontRight.spin(vex::directionType::fwd,dir * instantV,                 vex::velocityUnits::rpm);
        } else if ((currentAngle - angle) < (-AngleTolerance)) {
            // turn left a little bit
            backLeft.spin(vex::directionType::fwd,  dir * instantV,                 vex::velocityUnits::rpm);
            backRight.spin(vex::directionType::fwd, dir * instantV * percentChange, vex::velocityUnits::rpm);
            frontLeft.spin(vex::directionType::fwd, dir * instantV,                 vex::velocityUnits::rpm);
            frontRight.spin(vex::directionType::fwd,dir * instantV * percentChange, vex::velocityUnits::rpm);
        } else {
            backLeft.spin(vex::directionType::fwd,  dir * instantV, vex::velocityUnits::rpm);
            backRight.spin(vex::directionType::fwd, dir * instantV, vex::velocityUnits::rpm);
            frontLeft.spin(vex::directionType::fwd, dir * instantV, vex::velocityUnits::rpm);
            frontRight.spin(vex::directionType::fwd,dir * instantV, vex::velocityUnits::rpm);
        }

        wait(10, msec);  // loop pacing; the FTC version blocked on isBusy()
    }

    // No RUN_TO_POSITION to coast us to a stop — brake explicitly.
    frontLeft.stop(brake);
    backLeft.stop(brake);
    frontRight.stop(brake);
    backRight.stop(brake);
}

void driveFor(double distance, double speedPercent) {

    // ===== Speed profile =====
    const double MIN_SPEED = 10.0;            // % at start and end
    const double ACCEL_FRACTION = 0.20;       // 0% -> 15% of distance: ramp up
    const double DECEL_FRACTION = 0.30;       // 85% -> 100% of distance: ramp down

    // ===== Distance-proportional max speed =====
    // Scale linearly from MIN_SPEED (at 0 in) up to 100% (at FULL_SPEED_DIST in).
    // Beyond FULL_SPEED_DIST, cap at 100%.
    const double FULL_SPEED_DIST = 30.0;      // inches needed to earn full 100% speed
    const double ABSOLUTE_MAX = 75.0;

    double absDist = std::fabs(distance);
    double MAX_SPEED = MIN_SPEED + (ABSOLUTE_MAX - MIN_SPEED) * (absDist / FULL_SPEED_DIST);
    if (MAX_SPEED > ABSOLUTE_MAX) MAX_SPEED = ABSOLUTE_MAX;
    if (MAX_SPEED < MIN_SPEED)    MAX_SPEED = MIN_SPEED;

    // ===== Loop timing =====
    const int dt = 10;  // ms

    // ===== Distance setup =====
    double wheelCircumference = PI * WHEEL_DIAMETER;
    double wheelRevs = absDist / wheelCircumference;
    double motorRevs = wheelRevs / GEAR_RATIO;
    double targetDegrees = motorRevs * 360.0;

    bool driveForward = (distance >= 0);
    double startPos = frontLeft.position(degrees);
    double startTime = Brain.Timer.time(msec);
    int timeout = absDist * 60;

    // ===== Drive loop =====
    while (true) {
        double traveled = std::fabs(frontLeft.position(degrees) - startPos);
        if (traveled >= targetDegrees || Brain.Timer.time(msec) - startTime >= timeout) break;

        double progress = traveled / targetDegrees;

        // ===== Compute speed from trapezoidal profile =====
        double speed;
        if (progress < ACCEL_FRACTION) {
            double t = progress / ACCEL_FRACTION;
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * t * speedPercent;
        } else if (progress > (1.0 - DECEL_FRACTION)) {
            double t = (progress - (1.0 - DECEL_FRACTION)) / DECEL_FRACTION;
            speed = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * t * speedPercent;
        } else {
            speed = MAX_SPEED * speedPercent;
        }

        vex::directionType dir = driveForward ? vex::directionType::fwd : vex::directionType::rev;
        leftDriveSmart.spin(dir, speed, percent);
        rightDriveSmart.spin(dir, speed, percent);

        task::sleep(dt);
    }

    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
}

void autoOuttakeHigh(int time) {
    outtake.spin(directionType::fwd, 100, velocityUnits::pct);
    intake.spin(vex::directionType::fwd, 100, percent);
    wait(time, timeUnits::msec);
    outtake.stop();
}

void autoOuttakeMidHigh(int time) {
    outtake.spin(directionType::fwd, 100, velocityUnits::pct);
    intake.spin(vex::directionType::fwd, 100, percent);
    wait(time, timeUnits::msec);
    outtake.stop();
}

void autoOuttakeMidLow(int time) {
    outtake.spin(directionType::rev, 100, velocityUnits::pct);
    intake.spin(vex::directionType::rev, 100, percent);
    wait(time, timeUnits::msec);
    outtake.stop();
}

void moveToPosition(double targetX, double targetY){
    // First turn to face the target point
    turnTo(targetX, targetY);

    // Then drive forward to the target point
    double dist = distanceTo(targetX, targetY);
    driveFor(dist, 1);
}

double distanceTo(double target_x, double target_y){
    double distance = sqrt(pow((target_x - currX), 2) + pow((target_y - currY), 2));
    return distance;
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type, AI_RECORD local_map){
    DETECTION_OBJECT target;
    double lowestDist = 1000000000;

    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x / 0.0254, local_map.detections[i].mapLocation.y / 0.0254);
        if (distance < lowestDist && local_map.detections[i].classID == (int) type) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }

    return target;
}

void slideUpTo(double destinationAngle) {
    // Read the current position from slideMotor1
    double height = slideMotor1.position(rotationUnits::deg);

    while (abs(height - destinationAngle) > 4.0) {
        if (height < destinationAngle) {
            // Spin both motors forward to lift the slide up
            slideMotor1.spin(directionType::fwd, 100, velocityUnits::pct);
            slideMotor2.spin(directionType::fwd, 100, velocityUnits::pct);
        } 
        else {
            // Spin both motors in reverse to lower the slide down
            slideMotor1.spin(directionType::rev, 100, velocityUnits::pct);
            slideMotor2.spin(directionType::rev, 100, velocityUnits::pct);
        }

        // Update the height tracker from the encoder for the next loop check
        height = slideMotor1.position(rotationUnits::deg);
        
        // VEX wait() defaults to seconds if units aren't specified. 
        // 20 msec is the recommended minimum time step for sensor updates.
        wait(10, timeUnits::msec);
    }

    // Stop both motors to lock the slide position once within the target threshold
    slideMotor1.stop();
    slideMotor2.stop();
}

void slideMoveToBottomPosition() {
    slideMotor1.spin(directionType::rev, 100, velocityUnits::pct);
    slideMotor2.spin(directionType::rev, 100, velocityUnits::pct);

    while (!limitSwitch.pressing()) {
        wait(10, timeUnits::msec);
    }

    slideMotor1.stop();
    slideMotor2.stop();
    slideMotor1.setPosition(0, rotationUnits::deg);
    slideMotor2.setPosition(0, rotationUnits::deg);
}

int autoIntake() {
    
    intake.setStopping(brakeType::brake);
    outtake.setStopping(brakeType::hold);

    intake.spin(directionType::fwd, 100, velocityUnits::pct);
    outtake.spin(directionType::fwd, 100, velocityUnits::pct);

    while (!OpticalSensor.isNearObject() && intakemotorrunning) {
        wait(10, timeUnits::msec);
    }

    outtake.stop();
    intake.spin(directionType::fwd, 100, velocityUnits::pct);
    
    // while (std::abs(intake.velocity(velocityUnits::rpm)) > 130 && intakemotorrunning) {
    //     wait(20, timeUnits::msec);
    // }
    
    while (intakemotorrunning) {
        // intake.spin(directionType::fwd, 0, velocityUnits::pct);
        wait(20, timeUnits::msec);
    }    

    intake.stop();
    
    return 1;
}

void scoreIn(SCORING_LOCATIONS location, int time) {
    ScoringPos pos = getScoringPos(location);
    moveToPosition(pos.x, pos.y);
    turnToAbsolute(pos.heading);
    if (location == RED_HIGH_LEFT || location == RED_HIGH_RIGHT || location == BLUE_HIGH_LEFT || location == BLUE_HIGH_RIGHT) {
        slideUpTo(350); // Raise to high goal
        driveFor(-15.0, 1); // Drive forward to score
        autoOuttakeHigh(time);
    } else if (location == RED_MID_LEFT || location == BLUE_MID_LEFT){
        slideMoveToBottomPosition();
        driveFor(-8.0, 1); // Drive forward to score
        autoOuttakeMidHigh(time);
    }
    else{
        slideMoveToBottomPosition();
        driveFor(8.0, 1); // Drive forward to score
        autoOuttakeMidLow(time);
    }
}

void intakeTarget (DETECTION_OBJECT target) {
    double targetX = target.mapLocation.x / 0.0254; // Convert from meters to inches
    double targetY = target.mapLocation.y / 0.0254; // Convert from meters to inches
    intakemotorrunning = true;
    vex::task t1(autoIntake);
    moveToPosition(targetX, targetY);
    intakemotorrunning = false;
}

struct Rect { double xmin, ymin, xmax, ymax; };

// Robot footprint half-width (18"/2 = 9") + required clearance (9") = 18"
static const double INFLATE = 18.0;

// Field bounds (12 ft = 144", centered at origin). Keep the robot's
// center far enough from the wall that its footprint stays inside.
static const double FIELD = 72.0;
static const double WALL_MARGIN = 9.0;  // half robot footprint

std::vector<Rect> buildObstacles() {
  double raw[6][4] = {
    {-23.5,  47.5, 23.75,  47.5},   // top bar (thin)
    {-23.5, -47.5, 23.75, -47.5},   // bottom bar (thin)
    { -8.0,  -8.0,  8.0,   8.0},    // center square
    { -8.0,  -8.0,  8.0,   8.0},    // center square (duplicate diag)
    {-71.0,  -9.0, -54.0,  9.0},    // left block
    { 54.0,  -9.0,  71.0,  9.0}     // right block
  };
  std::vector<Rect> obs;
  for (int i = 0; i < 6; i++) {
    double xmin = std::min(raw[i][0], raw[i][2]);
    double xmax = std::max(raw[i][0], raw[i][2]);
    double ymin = std::min(raw[i][1], raw[i][3]);
    double ymax = std::max(raw[i][1], raw[i][3]);
    obs.push_back({xmin - INFLATE, ymin - INFLATE,
                   xmax + INFLATE, ymax + INFLATE});
  }
  return obs;
}

bool pointBlocked(double x, double y, const std::vector<Rect>& obs) {
  if (x < -FIELD + WALL_MARGIN || x > FIELD - WALL_MARGIN ||
      y < -FIELD + WALL_MARGIN || y > FIELD - WALL_MARGIN)
    return true;
  for (const auto& r : obs)
    if (x >= r.xmin && x <= r.xmax && y >= r.ymin && y <= r.ymax)
      return true;
  return false;
}

// ---- A* grid search -------------------------------------------------

static const double CELL = 3.0;  // grid resolution in inches

double toWorld(int g) { return g * CELL; }
int toGrid(double w)  { return (int)std::round(w / CELL); }

std::vector<std::pair<double,double>> planPath(
    double sx, double sy, double dx, double dy,
    const std::vector<Rect>& obs) {

  int gridMin = toGrid(-FIELD);
  int gridMax = toGrid(FIELD);
  int span = gridMax - gridMin + 1;

  auto idx = [&](int gx, int gy) {
    return (gy - gridMin) * span + (gx - gridMin);
  };

  std::vector<double> gScore(span * span, 1e18);
  std::vector<int> cameFrom(span * span, -1);
  std::vector<bool> closed(span * span, false);

  int sgx = toGrid(sx), sgy = toGrid(sy);
  int dgx = toGrid(dx), dgy = toGrid(dy);

  auto h = [&](int gx, int gy) {
    double ddx = (gx - dgx) * CELL, ddy = (gy - dgy) * CELL;
    return std::sqrt(ddx*ddx + ddy*ddy);
  };

  typedef std::pair<double,int> PQItem;
  std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;

  gScore[idx(sgx,sgy)] = 0;
  open.push({h(sgx,sgy), idx(sgx,sgy)});

  auto coords = [&](int id, int& gx, int& gy) {
    gx = id % span + gridMin;
    gy = id / span + gridMin;
  };

  int dx8[8] = {1,-1,0,0,1,1,-1,-1};
  int dy8[8] = {0,0,1,-1,1,-1,1,-1};

  bool found = false;
  while (!open.empty()) {
    int cur = open.top().second; open.pop();
    if (closed[cur]) continue;
    closed[cur] = true;

    int cgx, cgy; coords(cur, cgx, cgy);
    if (cgx == dgx && cgy == dgy) { found = true; break; }

    for (int k = 0; k < 8; k++) {
      int ngx = cgx + dx8[k], ngy = cgy + dy8[k];
      if (ngx < gridMin || ngx > gridMax ||
          ngy < gridMin || ngy > gridMax) continue;
      if (pointBlocked(toWorld(ngx), toWorld(ngy), obs)) continue;

      double step = (dx8[k]!=0 && dy8[k]!=0) ? CELL*1.41421356 : CELL;
      int nid = idx(ngx, ngy);
      double tentative = gScore[cur] + step;
      if (tentative < gScore[nid]) {
        gScore[nid] = tentative;
        cameFrom[nid] = cur;
        open.push({tentative + h(ngx, ngy), nid});
      }
    }
  }

  std::vector<std::pair<double,double>> path;
  if (!found) return path;

  int cur = idx(dgx, dgy);
  std::vector<int> rev;
  while (cur != -1) { rev.push_back(cur); cur = cameFrom[cur]; }
  std::reverse(rev.begin(), rev.end());
  for (int id : rev) {
    int gx, gy; coords(id, gx, gy);
    path.push_back({toWorld(gx), toWorld(gy)});
  }
  return path;
}

Rect obstacleAhead(double dist) {
  // currH convention: 0deg=+Y, clockwise-positive (same as targetHeading).
  double rad = currH * M_PI / 180.0;
  double ox = currX + dist * sin(rad);
  double oy = currY + dist * cos(rad);
  // 18"x18" footprint -> half-extent 9", plus INFLATE.
  double half = 9.0 + INFLATE;
  return { ox - half, oy - half, ox + half, oy + half };
}

// ---- Drive loop that aborts on a front-sensor obstacle --------------

// Returns: 0 = reached destination, 1 = blocked (needs replan).
int driveRoute(const std::vector<std::pair<double,double>>& path) {
  const double STOP_DIST = 3.0;  // inches

  for (size_t i = 1; i < path.size(); i++) {
    double dxr = path[i].first  - currX;
    double dyr = path[i].second - currY;
    double dist = std::sqrt(dxr*dxr + dyr*dyr);
    if (dist < 0.5) continue;

    double targetHeading = atan2(dxr, dyr) * 180.0 / M_PI;
    if (targetHeading < 0) targetHeading += 360;
    turnToAbsolute(targetHeading);

    // Check before committing to the leg.
    if (FrontDis.objectDistance(inches) < STOP_DIST) {
      leftDriveSmart.stop(brake);
      rightDriveSmart.stop(brake);
      return 1;
    }

    // Drive the leg incrementally so we can watch the sensor the whole way.
    double driven = 0;
    const double STEP = 4.0;  // inches per increment
    while (driven < dist) {
      if (FrontDis.objectDistance(inches) < STOP_DIST) {
        leftDriveSmart.stop(brake);
        rightDriveSmart.stop(brake);
        return 1;
      }
      double seg = std::min(STEP, dist - driven);
      driveFor(seg, 1);
      driven += seg;
    }
  }
  return 0;
}

// ---- Path smoothing (line-of-sight) ---------------------------------

bool segmentClear(double x0, double y0, double x1, double y1,
                  const std::vector<Rect>& obs) {
  double dist = std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
  int steps = std::max(1, (int)(dist / 1.0));
  for (int i = 0; i <= steps; i++) {
    double t = (double)i / steps;
    if (pointBlocked(x0 + t*(x1-x0), y0 + t*(y1-y0), obs))
      return false;
  }
  return true;
}

std::vector<std::pair<double,double>> smooth(
    std::vector<std::pair<double,double>> path,
    const std::vector<Rect>& obs) {
    if (path.size() < 3) return path;
    std::vector<std::pair<double,double>> out;
    out.push_back(path[0]);
    size_t anchor = 0;
    for (size_t i = 2; i < path.size(); i++) {
        if (!segmentClear(path[anchor].first, path[anchor].second,
                        path[i].first, path[i].second, obs)) {
        out.push_back(path[i-1]);
        anchor = i-1;
        }
    }
    out.push_back(path.back());
    return out;
}

// ---- Main entry point -----------------------------------------------

void routePlan(double destinationX, double destinationY) {
  auto obs = buildObstacles();

  if (pointBlocked(destinationX, destinationY, obs)) {
    Brain.Screen.print("Destination unreachable");
    return;
  }

  const int MAX_REPLANS = 5;
  for (int attempt = 0; attempt <= MAX_REPLANS; attempt++) {
    auto raw = planPath(currX, currY, destinationX, destinationY, obs);
    if (raw.empty()) {
      Brain.Screen.print("No route found");
      return;
    }
    auto path = smooth(raw, obs);

    int result = driveRoute(path);
    if (result == 0) return;  // arrived

    // Blocked: record the foreign robot as an obstacle, then replan.
    obs.push_back(obstacleAhead(FrontDis.objectDistance(inches)));

    // Safety: if we're now boxed in, bail.
    if (pointBlocked(currX, currY, obs)) {
      // The detected obstacle overlaps our own cell (very close hit);
      // shrink its effect by backing off slightly before replanning.
      driveFor(-6.0, 1);
    }
  }
  Brain.Screen.print("Could not reach: too many obstacles");
}

void auton_isolation(){
//    int n = 0;
    GPS.calibrate();
    waitUntil(!(GPS.isCalibrating()));
    DrivetrainInertial.setHeading(90, rotationUnits::deg);
    driveFor(20.0, 1);
    turnToAbsolute(0);
    intakemotorrunning = true;
    vex::task t1(autoIntake);
    driveFor(20.0, 0.5);
    moveToPosition(-36, 30);
    intakemotorrunning = false;
    turnToAbsolute(0);
    while(FrontDis.objectDistance(inches) > 18.0) {
        leftDriveSmart.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);
        rightDriveSmart.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);
        wait(20, timeUnits::msec);
    }
    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
    turnToAbsolute(270);
    driveFor(-12.0);
}

void auton_interaction(){
    
}



void teleop(void) {

// Set slide motors to "hold"
    slideMotor1.setStopping(brakeType::brake);
    slideMotor2.setStopping(brakeType::brake);

    slideMotor1.setPosition(0, rotationUnits::deg);
    slideMotor2.setPosition(0, rotationUnits::deg);
    //link.setOffset()

    // int x = 1;
    // int y = 1;
    Controller1.Screen.clearScreen();
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Read joystick positions from the controller (-100 to 100)
    // Axis 3: Left Joystick Y-Axis (Forward/Backward)
    // Axis 4: Left Joystick X-Axis (Strafe Left/Right)
    // Axis 1: Right Joystick X-Axis (Rotate Left/Right)
	
    int forwardValue = Controller1.Axis3.position();
    int strafeValue  = Controller1.Axis4.position();
    int turnValue    = Controller1.Axis1.position();

    // Mecanum Kinematics Math
    int flSpeed = forwardValue + strafeValue + turnValue;
    int blSpeed = forwardValue - strafeValue + turnValue;
    int frSpeed = forwardValue - strafeValue - turnValue;
    int brSpeed = forwardValue + strafeValue - turnValue;

    // Apply calculated speeds to the motors
    frontLeft.spin(vex::directionType::fwd, flSpeed, percent);
    backLeft.spin(vex::directionType::fwd, blSpeed, percent);
    frontRight.spin(vex::directionType::fwd, frSpeed, percent);
    backRight.spin(vex::directionType::fwd, brSpeed, percent);

    // for(int i=0;i<3;i++) {
    //     if(i < local_map.detectionCount ) {
    //         Controller1.Screen.setCursor(x+i, y);
    //         Controller1.Screen.print("%1d,%.2f,%.2f,%.2f",
    //                         local_map.detections[i].classID,
    //                         (local_map.detections[i].mapLocation.x / 0.0254),  // mm -> inches
    //                         (local_map.detections[i].mapLocation.y / 0.0254),  // mm -> inches
    //                         (local_map.detections[i].mapLocation.z / 0.0254)); // mm -> inches
    //     }
    //     else {
    //         Controller1.Screen.setCursor(x+i, y);
    //         Controller1.Screen.clearLine();
    //         Controller1.Screen.print("---");
    //     }
    // }


    if (Controller1.ButtonY.pressing()) {
        routePlan(48.0, 48.0);
    }

    if (Controller1.ButtonUp.pressing()) {
        slideUpTo(350);
    } 
    else if(Controller1.ButtonDown.pressing()) {
        slideMoveToBottomPosition();
    } 

    if (Controller1.ButtonR1.pressing()) {
        if (intakemotorrunning) {
            intakemotorrunning = false;
            intake.stop();
            outtake.stop();
        } else {
            intakemotorrunning = true;
            intake.spin(vex::directionType::fwd, 100, percent);
            outtake.spin(vex::directionType::fwd, 100, percent);
        }
        wait(200, timeUnits::msec);
        
    } 
    else if (Controller1.ButtonR2.pressing()) {
        if (intakemotorrunning) {
            intakemotorrunning = false;
            intake.stop();
            outtake.stop();
        } else {
            intakemotorrunning = true;
            intake.spin(vex::directionType::rev, 100, percent);
            outtake.spin(vex::directionType::rev, 100, percent);
        }
        wait(200, timeUnits::msec);
    } 
    else if (Controller1.ButtonX.pressing()) {
        if (intakemotorrunning) {
            intakemotorrunning = false;
        } else {
            intakemotorrunning = true;
            vex::task t1(autoIntake);
        }
        wait(200, timeUnits::msec);
    }

    task::sleep(20);

  }
}