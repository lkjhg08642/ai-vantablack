/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>

enum OBJECT {
    BallBlue,
    BallRed
};

enum SCORING_LOCATIONS{
    RED_HIGH_LEFT,
    RED_HIGH_RIGHT,
    RED_MID_LEFT,
    RED_MID_RIGHT,
    BLUE_HIGH_LEFT,
    BLUE_HIGH_RIGHT,
    BLUE_MID_LEFT,
    BLUE_MID_RIGHT
};

struct ScoringPos {
    double x;
    double y;
    double heading;
};

using namespace vex;

// Calculates the distance to a given target (x, y)
double distanceTo(double target_x, double target_y);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(OBJECT type, AI_RECORD local_map);

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double targetX, double targetY);

// Drives the robot in a specified heading for a given distance and speed
void driveFor(double dist);

void auton_interaction();
void auton_isolation();

void teleop(void);