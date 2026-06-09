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
bool outtakemotorrunning;

// Robot constants
const double WHEEL_DIAMETER = 4.0;        // inches
const double GEAR_RATIO = 0.5;            // motor:wheel ratio
const double TRACK_WIDTH = 12.0;          // inches between left/right wheels (ADJUST TO YOUR ROBOT)
const double PI = 3.14159265358979323846;

bool outtake_raised = false;
bool odom_raised = false;
bool loader_dropped = false;
bool descore_raised = false;

const int N = 47;
int field[N][N] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1/**/,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
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
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},/**/
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
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
  double kP = 0.38;
  double kI = 0.06;
  double kD = 0.03;

  double integral = 0;
  double prevError = 0;
  double error = 0;
  double derivativeFiltered = 0;

  const double dt = 0.01;            // 10 ms, matches wait() below

  // Tolerance settings
  double errorThreshold = 0.8;       // degrees
  double velocityThreshold = 8.0;    // deg/s of actual rotation to consider stopped
  int settleCount = 0;
  int settleTarget = 10;             // loops within tolerance before exiting

  double integralLimit = 50.0;       // anti-windup cap
  double minVolt = 2.5;              // static-friction floor

  // ---- Timeout: give up after this long regardless of settling ----
  double timeoutSec = 2.0;           // tune: enough for a full ~180 turn + settle
  vex::timer turnTimer;
  turnTimer.clear();
  bool timedOut = false;

  while (true) {
    // ---- timeout check ----
    if (turnTimer.time(vex::timeUnits::sec) > timeoutSec) {
      timedOut = true;
      break;
    }

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

    // Derivative, scaled to deg/s and low-pass filtered
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

    // Settle check uses real rotational velocity (deg/s)
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

  // Optional: surface a timeout so callers/you can see it on the brain
  if (timedOut) {
    Brain.Screen.clearLine();
    Brain.Screen.print("turnToAbsolute timeout");
  }
}

void turnToRelative(double theta) {
  double target = DrivetrainInertial.heading(degrees) + theta;
  turnToAbsolute(target);
}

void turnTo(double targetX, double targetY) {
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
    double fbcoef = 56;  // convert inches to motor degrees
    double distanceEncoder = fbcoef * fdistance;
    double currentPos = 0;
    double posDifference;
    double instantV = 0;
    double thresholdBegin = 10 * fbcoef;
    double thresholdEnd   = 30 * fbcoef;
    double minSpeed = 125.0;
    double maxSpeed = 500.0;
    double currentAngle = DrivetrainInertial.heading(degrees);
    double angle = currentAngle;

    double AngleTolerance = 0.25;   // maintain angle +- 0.25 degrees
    double percentChange  = 1.1;

    double timeoutSec = fabs(distanceEncoder) / (minSpeed * 6.0);

    // VEX has no STOP_AND_RESET_ENCODER + RUN_TO_POSITION. Reset encoders
    // and drive by velocity, ending the loop when we reach the target count.
    odom.resetPosition();

    // Direction of travel (sign of the requested distance).
    double dir = (distanceEncoder >= 0) ? 1.0 : -1.0;

    vex::timer moveTimer;
    moveTimer.clear();

    while (fabs(currentPos) < fabs(distanceEncoder)) {

        if (moveTimer.time(vex::timeUnits::sec) > timeoutSec) break;

        currentPos   = odom.position(degrees);
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
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV*0.02,                 vex::voltageUnits::volt);
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            
        } else if ((currentAngle - angle) < (-AngleTolerance)) {
            // turn left a little bit
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * 0.02,                 vex::voltageUnits::volt);
            
        } else {
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV * 0.02, vex::voltageUnits::volt);
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * 0.02, vex::voltageUnits::volt);
           
        }

        wait(10, msec);  // loop pacing; the FTC version blocked on isBusy()
    }

    // No RUN_TO_POSITION to coast us to a stop — brake explicitly.
    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);
}

void moveToPosition(double targetX, double targetY){
    // First turn to face the target point
    turnTo(targetX, targetY);

    // Then drive forward to the target point
    double dist = distanceTo(targetX, targetY);
    
    //driveFor(dist, 1);
    forwardStraight(dist);
}

double distanceTo(double target_x, double target_y){
    double distance = sqrt(pow((target_x - currX), 2) + pow((target_y - currY), 2));
    return distance;
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type, AI_RECORD local_map){
    DETECTION_OBJECT target;
    target.classID = -1;          // sentinel: -1 means "no target found"
    double lowestDist = 1e9;

    for (int i = 0; i < local_map.detectionCount; i++) {
        if (local_map.detections[i].classID != (int) type) continue;

        double bx = local_map.detections[i].mapLocation.x / 0.0254;  // m -> in
        double by = local_map.detections[i].mapLocation.y / 0.0254;

        // reject blocks inside a blocked area
        int row, col;
        GPStoGrid(bx, by, row, col);
        if (field[row][col] == 1) continue;

        double distance = distanceTo(bx, by);
        if (distance < lowestDist) {
            lowestDist = distance;
            target = local_map.detections[i];
        }
    }
    return target;   // caller checks target.classID == -1 for "none"
}

int autoIntake() {
    
    intakemotorrunning = true;
    
    // Configure standard motor brake modes
    intake.setStopping(brakeType::brake);
    outtake.setStopping(brakeType::hold);

    while (intakemotorrunning) {
        // Spin intake at full speed (100%) and hold the outtake still
        intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
        outtake.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
        
        // Initial 1-second delay
        wait(1000, timeUnits::msec);
        
        // Monitor intake motor speed drop-off (stalling/loading checks)
        while (std::abs(intake.velocity(velocityUnits::rpm)) > 400 && intakemotorrunning) {
            wait(20, timeUnits::msec);
        }
        
        // Feed the outtake roller at 80% power
        outtake.spin(vex::directionType::fwd, 9.6, vex::voltageUnits::volt);
        
        // Poll proximity sensor (VEX Optical scales natively from 0 to 100)
        while (!OpticalSensor.isNearObject() && intakemotorrunning) {
            wait(20, timeUnits::msec);
        }
        
        // Stop outtake and monitor clearing speeds
        outtake.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
        
        while (intake.velocity(velocityUnits::rpm) > 100 && intakemotorrunning) {
            wait(20, timeUnits::msec);
        }
        
        // Settle delay before wrapping the loop iteration
        if (intakemotorrunning) {
            intake.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
            wait(300, timeUnits::msec);
        }
    }

    // Explicit shutdown cleanup safely halting elements
    intake.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
    intakemotorrunning = false;
    return 1;
}

int autoOuttakeHigh(){
    outtake_raiser.set(true);
    outtakemotorrunning = true;

    intake.setStopping(brakeType::coast);
    outtake.setStopping(brakeType::coast);

    intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
    outtake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);

    wait(600, timeUnits::msec);

    while(outtakemotorrunning){
        while(abs(intake.velocity(velocityUnits::rpm))>5 && abs(outtake.velocity(velocityUnits::rpm))>5 && outtakemotorrunning){
            wait(20, timeUnits::msec);
        }

        if(outtakemotorrunning){
            intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            wait(200, timeUnits::msec);
            intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            wait(400, timeUnits::msec);
        }
    }

    intake.stop(brake);
    outtake.stop(brake);
    outtakemotorrunning = false;
    return 1;
}

int autoOuttakeMidHigh(){
    outtake_raiser.set(false);
    outtakemotorrunning = true;

    intake.setStopping(brakeType::coast);
    outtake.setStopping(brakeType::coast);

    intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
    outtake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);

    wait(600, timeUnits::msec);

    while(outtakemotorrunning){
        while(abs(intake.velocity(velocityUnits::rpm))>5 && abs(outtake.velocity(velocityUnits::rpm))>5 && outtakemotorrunning){
            wait(20, timeUnits::msec);
        }

        if(outtakemotorrunning){
            intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            wait(200, timeUnits::msec);
            intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            wait(400, timeUnits::msec);
        }
    }

    intake.stop(brake);
    outtake.stop(brake);
    outtakemotorrunning = false;
    return 1;
}

int autoOuttakeLow(){
    outtake_raiser.set(false);
    outtakemotorrunning = true;

    intake.setStopping(brakeType::coast);
    outtake.setStopping(brakeType::coast);

    intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
    outtake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);

    wait(600, timeUnits::msec);

    while(outtakemotorrunning){
        while(abs(intake.velocity(velocityUnits::rpm))>5 && abs(outtake.velocity(velocityUnits::rpm))>5 && outtakemotorrunning){
            wait(20, timeUnits::msec);
        }

        if(outtakemotorrunning){
            intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
            wait(200, timeUnits::msec);
            intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            outtake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
            wait(400, timeUnits::msec);
        }
    }

    intake.stop(brake);
    outtake.stop(brake);
    outtakemotorrunning = false;
    return 1;
}

bool pathFindTo(double destX, double destY) {
    double startX = currX, startY =  currY;

    int sr, sc, gr, gc;
    GPStoGrid(startX, startY, sr, sc);
    GPStoGrid(destX,  destY,  gr, gc);

    int pathR[N*N], pathC[N*N];
    int len = aStar(sr, sc, gr, gc, pathR, pathC);

    if (len == 0) {
        Brain.Screen.print("No path found");
        return false;
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
        wait(500, timeUnits::msec);
        moveToPosition(wx, wy);
    }
    return true;
}

void scoreIn(SCORING_LOCATIONS location, int time) {
    ScoringPos pos = getScoringPos(location);
    bool pathFound = pathFindTo(pos.x, pos.y);
    if (pathFound){
        if (location == RED_HIGH_LEFT || location == RED_HIGH_RIGHT || location == BLUE_HIGH_LEFT || location == BLUE_HIGH_RIGHT) {
            if (location == RED_HIGH_LEFT || location == BLUE_HIGH_RIGHT){
                turnToAbsolute(0);
            } else{
                turnToAbsolute(180);
            }
            if (FrontDis.objectDistance(mm) > 520) {
                leftDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
                rightDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
                while(FrontDis.objectDistance(mm) > 510) { 
                    wait(20, timeUnits::msec);
                }
                leftDriveSmart.stop(brake);
                rightDriveSmart.stop(brake);
            }
            else if(FrontDis.objectDistance(mm) < 480){
                leftDriveSmart.spin(vex::directionType::rev, 3, vex::voltageUnits::volt);
                rightDriveSmart.spin(vex::directionType::rev, 3, vex::voltageUnits::volt);
                while(FrontDis.objectDistance(mm) < 490) { 
                    wait(20, timeUnits::msec);
                }
                leftDriveSmart.stop(brake);
                rightDriveSmart.stop(brake);
            }
            turnToAbsolute(pos.heading);
            forwardStraight(-15.0); // Drive forward to score
            outtakemotorrunning = true;
            vex::task t1(autoOuttakeHigh);
            wait(time, timeUnits::msec);
            outtakemotorrunning = false;
        } else if (location == RED_MID_LEFT || location == BLUE_MID_LEFT){
            forwardStraight(-8.0); // Drive forward to score
            outtakemotorrunning = true;
            vex::task t1(autoOuttakeMidHigh);
            wait(time, timeUnits::msec);
            outtakemotorrunning = false;
        }
        else{
            forwardStraight(8.0); // Drive forward to score
            outtakemotorrunning = true;
            vex::task t1(autoOuttakeLow);
            wait(time, timeUnits::msec);
            outtakemotorrunning = false;
        }
    }
}

bool intakeTarget (DETECTION_OBJECT target) {
    double targetX = target.mapLocation.x / 0.0254; // Convert from meters to inches
    double targetY = target.mapLocation.y / 0.0254; // Convert from meters to inches
    intakemotorrunning = true;
    vex::task t1(autoIntake);
    bool pathFound = pathFindTo(targetX, targetY);
    wait(300, timeUnits::msec); // Wait for the robot to stabilize at the target position
    intakemotorrunning = false;
    return pathFound;
}

void auton_isolation(){

}

void auton_interaction(){
    
}



void teleop(void) {

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
    int turnValue    = Controller1.Axis1.position();

    // Standard Arcade Drive mixing math
    int leftSpeed  = forwardValue + turnValue;
    int rightSpeed = forwardValue - turnValue;

    // Apply calculated speeds directly to the motor groups.
    // The individual motor direction rules (true/false) you specified during 
    // initialization will automatically handle the reversing internally.
    leftDriveSmart.spin(vex::directionType::fwd, leftSpeed, percent);
    rightDriveSmart.spin(vex::directionType::fwd, rightSpeed, percent);


    if (Controller1.ButtonY.pressing()) { 
        pathFindTo(-24.0, 24.0);
    }
    if (Controller1.ButtonA.pressing()) { 
        forwardStraight(-71.5);
    }

    if (Controller1.ButtonX.pressing()) { 
        if (intakemotorrunning) {
            intakemotorrunning = false;
        } else {
            intakemotorrunning = true;
            vex::task t1(autoIntake); 
        }
        wait(250, msec); // Debounce delay
    }

    if (Controller1.ButtonR1.pressing()) {
        intake.spin(vex::directionType::fwd, 100, percent);
        outtake.spin(vex::directionType::fwd, 100, percent);
        intakemotorrunning = false; 
    }
    else if (Controller1.ButtonR2.pressing()) {
        intake.spin(vex::directionType::rev, 100, percent);
        outtake.spin(vex::directionType::rev, 100, percent);
        intakemotorrunning = false;
    }
    else {
        // If no manual buttons are held and the macro task is inactive, completely stall rollers
        if (!intakemotorrunning) {
            intake.spin(vex::directionType::fwd, 0, percent);
            outtake.spin(vex::directionType::fwd, 0, percent);
        }    
    }

    if (Controller1.ButtonY.pressing()) {
        if (outtake_raised) {
            outtake_raiser.set(false);
            outtake_raised = false;
        } else {
            outtake_raiser.set(true);
            outtake_raised = true;
        }
        wait(250, msec); // Debounce delay
    }

    if (Controller1.ButtonA.pressing()) {
        if (loader_dropped) {
            loader.set(false);
            loader_dropped = false;
        } else {
            loader.set(true);
            loader_dropped = true;
        }
        wait(250, msec); // Debounce delay
    }

    if (Controller1.ButtonLeft.pressing()) {
        if (descore_raised) {
            descore.set(false);
            descore_raised = false;
        } else {
            descore.set(true);
            descore_raised = true;
        }
        wait(250, msec); // Debounce delay
    }

    if (Controller1.ButtonRight.pressing()) {
        if (odom_raised) {
            odomraiser.set(false);
            odom_raised = false;
        } else {
            odomraiser.set(true);
            odom_raised = true;
        }
        wait(250, msec); // Debounce delay
    }

    task::sleep(10);

  }
}