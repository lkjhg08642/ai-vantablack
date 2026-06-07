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

#include "vex.h"
#include <vector>
#include <queue>
#include <cmath>

bool intakemotorrunning;

// Robot constants
const double WHEEL_DIAMETER = 4.0;        // inches
const double GEAR_RATIO = 0.5;            // motor:wheel ratio
const double TRACK_WIDTH = 12.0;          // inches between left/right wheels (ADJUST TO YOUR ROBOT)
const double PI = 3.14159265358979323846;

const int N = 47;
int field[N][N] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1/**/,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    /**/{1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// ---------- Coordinate conversions ----------
const double CELL = 3.0;       // inches per grid cell
const int    CENTER = 23;      // center index

// GPS (inches, center origin, +X right, +Y up) -> matrix index
void GPStoGrid(double currX, double currY, int &row, int &col) {
    col = (int)round(currX / CELL) + CENTER;
    row = CENTER - (int)round(currY / CELL);
    if (col < 0) col = 0; if (col > N-1) col = N-1;
    if (row < 0) row = 0; if (row > N-1) row = N-1;
}

// matrix index -> GPS (inches, center origin)
void gridToGPS(int row, int col, double &X, double &Y) {
    X = (col - CENTER) * CELL;
    Y = (CENTER - row) * CELL;
}

// ---------- A* (4-connectivity) ----------
struct Cell { int r, c; };

int heuristic(int r, int c, int gr, int gc) {
    return abs(r - gr) + abs(c - gc);   // Manhattan
}

// Fills pathR/pathC with the route, returns number of cells (0 if no path)
int aStar(int sr, int sc, int gr, int gc, int pathR[], int pathC[]) {
    static int  gScore[N][N];
    static bool closed[N][N];
    static int  cameR[N][N], cameC[N][N];

    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            gScore[r][c] = 1e8; closed[r][c] = false;
            cameR[r][c] = -1; cameC[r][c] = -1;
        }

    // priority queue: pair<fScore, index(r*N+c)>
    std::priority_queue<std::pair<int,int>,
        std::vector<std::pair<int,int>>,
        std::greater<std::pair<int,int>>> open;

    gScore[sr][sc] = 0;
    open.push({heuristic(sr, sc, gr, gc), sr * N + sc});

    int dR[4] = {-1, 1, 0, 0};
    int dC[4] = {0, 0, -1, 1};

    while (!open.empty()) {
        int idx = open.top().second; open.pop();
        int r = idx / N, c = idx % N;
        if (closed[r][c]) continue;
        closed[r][c] = true;

        if (r == gr && c == gc) break;

        for (int k = 0; k < 4; k++) {
            int nr = r + dR[k], nc = c + dC[k];
            if (nr < 0 || nr >= N || nc < 0 || nc >= N) continue;
            if (field[nr][nc] == 1) continue;       // obstacle
            int ng = gScore[r][c] + 1;
            if (ng < gScore[nr][nc]) {
                gScore[nr][nc] = ng;
                cameR[nr][nc] = r; cameC[nr][nc] = c;
                open.push({ng + heuristic(nr, nc, gr, gc), nr * N + nc});
            }
        }
    }

    if (cameR[gr][gc] == -1 && !(sr == gr && sc == gc)) return 0;  // no path

    // reconstruct (reversed)
    int tmpR[N*N], tmpC[N*N], len = 0;
    int r = gr, c = gc;
    while (!(r == sr && c == sc)) {
        tmpR[len] = r; tmpC[len] = c; len++;
        int pr = cameR[r][c], pc = cameC[r][c];
        r = pr; c = pc;
    }
    tmpR[len] = sr; tmpC[len] = sc; len++;

    // reverse into output
    for (int i = 0; i < len; i++) {
        pathR[i] = tmpR[len - 1 - i];
        pathC[i] = tmpC[len - 1 - i];
    }
    return len;
}

// ---------- Keep only turning points ----------
int extractTurns(int pathR[], int pathC[], int len, int turnR[], int turnC[]) {
    if (len == 0) return 0;
    int t = 0;
    turnR[t] = pathR[0]; turnC[t] = pathC[0]; t++;   // start
    for (int i = 1; i < len - 1; i++) {
        int d1r = pathR[i]   - pathR[i-1], d1c = pathC[i]   - pathC[i-1];
        int d2r = pathR[i+1] - pathR[i],   d2c = pathC[i+1] - pathC[i];
        if (d1r != d2r || d1c != d2c) {              // direction changed
            turnR[t] = pathR[i]; turnC[t] = pathC[i]; t++;
        }
    }
    turnR[t] = pathR[len-1]; turnC[t] = pathC[len-1]; t++;   // destination
    return t;
}

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
  double kP = 0.4;    // small: less overshoot, more like a gentle spring; large: more aggressive correction but can overshoot and oscillate
  double kI = 0.05;   // small: integral builds slowly, good for correcting steady-state error without causing instability; large: integral builds quickly, can eliminate steady-state error faster but may cause overshoot and oscillation
  double kD = 0.03;   // small: derivative is now scaled to deg/s and filtered

  double integral = 0;
  double prevError = 0;
  double error = 0;
  double derivativeFiltered = 0;

  const double dt = 0.01;            // 10 ms, matches wait() below

  // Tolerance settings
  double errorThreshold = 0.75;      // degrees
  double velocityThreshold = 8.0;    // deg/s of actual rotation to consider stopped
  int settleCount = 0;
  int settleTarget = 10;             // loops within tolerance before exiting

  double integralLimit = 50.0;       // anti-windup cap
  double minVolt = 2.5;              // static-friction floor (tune 1.0-2.0)

  while (true) {
    // Compute error as shortest angular distance to target [-180, 180]
    double current = DrivetrainInertial.heading(degrees);
    error = theta - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    // Integral with anti-windup (only accumulate near target)
    if (fabs(error) < 15) {
      integral += error * dt;
    } else {
      integral = 0;
    }
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    // Derivative, scaled to deg/s and low-pass filtered to kill the end twitch
    double rawD = (error - prevError) / dt;
    derivativeFiltered = 0.7 * derivativeFiltered + 0.3 * rawD;
    double derivative = derivativeFiltered;

    // PID output
    double output = (kP * error) + (kI * integral) + (kD * derivative);

    // Static-friction floor: only while genuinely outside tolerance
    if (fabs(error) > errorThreshold && fabs(output) < minVolt) {
      output = (output > 0 ? minVolt : -minVolt);
    }

    // Clamp output to motor voltage range
    if (output > 12) output = 12;
    if (output < -12) output = -12;

    // Apply: positive output turns clockwise (right)
    leftDriveSmart.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::rev, output, vex::voltageUnits::volt);

    // Settle check now uses real rotational velocity (deg/s)
    if (fabs(error) < errorThreshold && fabs(derivative) < velocityThreshold) {
      settleCount++;
      if (settleCount >= settleTarget) break;
    } else {
      settleCount = 0;
    }

    prevError = error;
    wait(10, msec);
  }

  // Ramp to zero before braking to avoid a final snap
  leftDriveSmart.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
  rightDriveSmart.spin(vex::directionType::rev, 0, vex::voltageUnits::volt);
  wait(20, msec);

  leftDriveSmart.stop(brake);
  rightDriveSmart.stop(brake);
}

void turnToRelative(double theta) {
  double target = DrivetrainInertial.heading(degrees) + theta;
  turnToAbsolute(target);
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
    
    turnToAbsolute(targetHeading);
}

void forwardStraight(double fdistance) {
    double fbcoef = 51;  // convert inches to motor degrees
    double distanceEncoder = fbcoef * fdistance;
    double currentPos = 0;
    double posDifference;
    double instantV = 0;
    double thresholdBegin = 10 * fbcoef;
    double thresholdEnd   = 20 * fbcoef;
    double minSpeed = 130.0;
    double maxSpeed = 600.0;
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
            backLeft.spin(vex::directionType::fwd,  dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            backRight.spin(vex::directionType::fwd, dir * instantV*0.02,                 vex::voltageUnits::volt);
            frontLeft.spin(vex::directionType::fwd, dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            frontRight.spin(vex::directionType::fwd,dir * instantV*0.02,                 vex::voltageUnits::volt);
        } else if ((currentAngle - angle) < (-AngleTolerance)) {
            // turn left a little bit
            backLeft.spin(vex::directionType::fwd,  dir * instantV * 0.02,                 vex::voltageUnits::volt);
            backRight.spin(vex::directionType::fwd, dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            frontLeft.spin(vex::directionType::fwd, dir * instantV * 0.02,                 vex::voltageUnits::volt);
            frontRight.spin(vex::directionType::fwd,dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
        } else {
            backLeft.spin(vex::directionType::fwd,  dir * instantV * 0.02, vex::voltageUnits::volt);
            backRight.spin(vex::directionType::fwd, dir * instantV * 0.02, vex::voltageUnits::volt);
            frontLeft.spin(vex::directionType::fwd, dir * instantV * 0.02, vex::voltageUnits::volt);
            frontRight.spin(vex::directionType::fwd,dir * instantV * 0.02, vex::voltageUnits::volt);
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
    const double ABSOLUTE_MAX = 100.0;

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
        leftDriveSmart.spin(dir, speed*0.12, vex::voltageUnits::volt);
        rightDriveSmart.spin(dir, speed*0.12, vex::voltageUnits::volt);

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
    // forwardStraight(dist);
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
    // Read the current position from slideMotor2
    double height = slideMotor2.position(rotationUnits::deg);

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
        height = slideMotor2.position(rotationUnits::deg);
        
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

void auton_isolation(){
    slideMotor1.setPosition(0, rotationUnits::deg);
    slideMotor2.setPosition(0, rotationUnits::deg);
//    int n = 0;
    GPS.calibrate();
    waitUntil(!(GPS.isCalibrating()));
    DrivetrainInertial.setHeading(90, rotationUnits::deg);
    forwardStraight(18.0);
    // driveFor(18.0, 1);
    turnToAbsolute(0);
    intakemotorrunning = true;
    vex::task t1(autoIntake);
    forwardStraight(20.0);
    // driveFor(20.0, 0.5);
    moveToPosition(-36, 24);
    intakemotorrunning = false;
    turnToAbsolute(0);
    leftDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    while(FrontDis.objectDistance(inches) > 23.0) { 
        wait(20, timeUnits::msec);
    }
    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
    turnToAbsolute(265);
    slideUpTo(350);
    forwardStraight(-18.0);
    // driveFor(-18.0, 1);
    autoOuttakeHigh(1000);
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
        double startX = currX, startY =  currY;
        double destX  =  -48, destY  = 48;

        int sr, sc, gr, gc;
        GPStoGrid(startX, startY, sr, sc);
        GPStoGrid(destX,  destY,  gr, gc);

        int pathR[N*N], pathC[N*N];
        int len = aStar(sr, sc, gr, gc, pathR, pathC);

        if (len == 0) {
            Brain.Screen.print("No path found");
        }

        int turnR[N*N], turnC[N*N];
        int turns = extractTurns(pathR, pathC, len, turnR, turnC);

        // Print + drive the turning waypoints in GPS coordinates
        Brain.Screen.print("Turn waypoints (GPS in):");
        Brain.Screen.newLine();
        for (int i = 0; i < turns; i++) {
            double wx, wy;
            gridToGPS(turnR[i], turnC[i], wx, wy);
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("(%.0f, %.0f)", wx, wy);
            Brain.Screen.print("(%.0f, %.0f)", wx, wy);
            Brain.Screen.newLine();
            wait(500, timeUnits::msec);
            moveToPosition(wx, wy);
        }
    }
    if (Controller1.ButtonA.pressing()) {
        forwardStraight(-48.0);
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