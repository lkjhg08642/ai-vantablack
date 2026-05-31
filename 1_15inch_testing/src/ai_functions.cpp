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

void turnToHeading(double angle) {
    // PID constants for turning (tune these for your robot)
    double kP = 1.0;
    double kI = 0.02;
    double kD = 0.2;
    // Tolerances
    double HEADING_TOLERANCE = 1.0;     // degrees
    double INTEGRAL_LIMIT = 50.0;       // anti-windup
    double MIN_POWER = 2.0;             // minimum power to overcome static friction
    double MAX_POWER = 80.0;            // cap on motor power (%)
    double SETTLE_TIME_MS = 150;        // must be within tolerance for this long
    double TIMEOUT_MS = 2500;           // abort if taking too long

    // Target heading is just the requested angle (absolute heading in degrees, CW from +Y)
    double targetHeading = angle;

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

void driveFor(double distance) {

    // ===== Speed profile =====
    const double MIN_SPEED = 10.0;            // % at start and end
    const double ACCEL_FRACTION = 0.20;       // 0% -> 15% of distance: ramp up
    const double DECEL_FRACTION = 0.20;       // 85% -> 100% of distance: ramp down

    // ===== Distance-proportional max speed =====
    // Scale linearly from MIN_SPEED (at 0 in) up to 100% (at FULL_SPEED_DIST in).
    // Beyond FULL_SPEED_DIST, cap at 100%.
    const double FULL_SPEED_DIST = 48.0;      // inches needed to earn full 100% speed
    const double ABSOLUTE_MAX = 70.0;

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
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * t;
        } else if (progress > (1.0 - DECEL_FRACTION)) {
            double t = (progress - (1.0 - DECEL_FRACTION)) / DECEL_FRACTION;
            speed = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * t;
        } else {
            speed = MAX_SPEED;
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
    driveFor(dist);
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
    turnToHeading(pos.heading);
    if (location == RED_HIGH_LEFT || location == RED_HIGH_RIGHT || location == BLUE_HIGH_LEFT || location == BLUE_HIGH_RIGHT) {
        slideUpTo(350); // Raise to high goal
        driveFor(-15.0); // Drive forward to score
        autoOuttakeHigh(time);
    } else if (location == RED_MID_LEFT || location == BLUE_MID_LEFT){
        slideMoveToBottomPosition();
        driveFor(-8.0); // Drive forward to score
        autoOuttakeMidHigh(time);
    }
    else{
        slideMoveToBottomPosition();
        driveFor(8.0); // Drive forward to score
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

void auton_isolation(){
//   int n = 0;
  GPS.calibrate();
  waitUntil(!(GPS.isCalibrating()));
  DrivetrainInertial.setHeading(GPS.heading(), rotationUnits::deg);

//   DETECTION_OBJECT target;
//   double lowestDist = 0;
  
  /*while (true){
    jetson_comms.get_data(&local_map);
    link.set_remote_location(local_map.pos.x, local_map.pos.y,
                              local_map.pos.az, local_map.pos.status);
    jetson_comms.request_map();

    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(n);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(local_map.detectionCount);
    
    Controller1.Screen.setCursor(3,1);
    if (local_map.detectionCount != 0) {
        target = findTarget(OBJECT::BallBlue, local_map);
        double targetX = target.mapLocation.x / 0.0254;
        double targetY = target.mapLocation.y / 0.0254;
        Controller1.Screen.print("x: %.2f, y: %.2f\n", targetX, targetY);
    }
    else {
        Controller1.Screen.setCursor(3,1);
        Controller1.Screen.print("No objects detected");
    }
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("x: %.2f, y: %.2f\n", local_map.detections[0].mapLocation.x, local_map.detections[0].mapLocation.y);
    // Iterate through detected objects to find the closest target of the specified type


    // Controller1.Screen.setCursor(2,1);
    // Controller1.Screen.print("%.2f, %.2f, %.2f\n", GPS.xPosition(distanceUnits::in), GPS.yPosition(distanceUnits::in), GPS.heading(rotationUnits::deg));
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("%.2f, %.2f, %.2f\n", currX, currY, currH);

    n++;
    this_thread::sleep_for(std::chrono::milliseconds(500));
    Controller1.Screen.clearScreen();
    //goToObject(OBJECT::BallBlue);
  }*/
//   moveToPosition(-48*2.54, -48*2.54, 0, 50);
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
        intakeTarget(findTarget(OBJECT::BallRed, local_map));
        wait(500, timeUnits::msec);
        scoreIn(RED_HIGH_RIGHT, 2000);
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