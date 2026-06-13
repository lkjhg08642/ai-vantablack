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
#include <cstdlib>
#include <utility>
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

int currColor = 2; //0 blue, 2 red

const int N = 47;
int field[N][N] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1/**/,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},/**/
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// ---------- Clearance cost field ----------
// Higher cost near obstacles so A* prefers routes through open space.
double clearanceCost[N][N];

const double CLEAR_RADIUS = 8.0;    // cells: how far avoidance reaches
const double CLEAR_WEIGHT = 60.0;   // strength of avoidance (raise = wider berth)

void computeClearanceCost() {
    static double dist[N][N];
    std::queue<std::pair<int,int>> q;
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            if (field[r][c] == 1) { dist[r][c] = 0.0; q.push({r, c}); }
            else                    dist[r][c] = 1e9;
        }
    int dR[4] = {-1, 1, 0, 0};
    int dC[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        std::pair<int,int> cur = q.front(); q.pop();
        int r = cur.first, c = cur.second;
        for (int k = 0; k < 4; k++) {
            int nr = r + dR[k], nc = c + dC[k];
            if (nr < 0 || nr >= N || nc < 0 || nc >= N) continue;
            if (dist[nr][nc] > dist[r][c] + 1.0) {
                dist[nr][nc] = dist[r][c] + 1.0;
                q.push({nr, nc});
            }
        }
    }
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            if (field[r][c] == 1) { clearanceCost[r][c] = 0.0; continue; }
            double closeness = (CLEAR_RADIUS - dist[r][c]) / CLEAR_RADIUS;
            if (closeness < 0.0) closeness = 0.0;
            clearanceCost[r][c] = closeness * closeness * CLEAR_WEIGHT;
        }
}

// ---------- Coordinate conversions ----------
const double CELL   = 3.0;    // inches per grid cell
const int    CENTER = 23;     // center index (0,0) in GPS

void GPStoGrid(double gx, double gy, int &row, int &col) {
    col = (int)round(gx / CELL) + CENTER;
    row = CENTER - (int)round(gy / CELL);
    if (col < 0) col = 0; if (col > N-1) col = N-1;
    if (row < 0) row = 0; if (row > N-1) row = N-1;
}

void gridToGPS(int row, int col, double &X, double &Y) {
    X = (col - CENTER) * CELL;
    Y = (CENTER - row) * CELL;
}

// ============================================================================
//  A*  (8-connected, octile heuristic, clearance-aware)
// ============================================================================
double heuristic(int r, int c, int gr, int gc) {
    int dr = abs(r - gr), dc = abs(c - gc);
    int mn = (dr < dc) ? dr : dc;
    int mx = (dr > dc) ? dr : dc;
    return (mx - mn) + 1.41421356 * mn;   // octile
}

// Snap a cell to the nearest free cell via BFS (handles start/dest on a 1).
bool nearestFreeCell(int r, int c, int &outR, int &outC) {
    if (field[r][c] == 0) { outR = r; outC = c; return true; }
    static bool seen[N][N];
    for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) seen[i][j] = false;
    std::queue<std::pair<int,int>> q;
    seen[r][c] = true; q.push({r, c});
    int dR[4] = {-1, 1, 0, 0};
    int dC[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        std::pair<int,int> cur = q.front(); q.pop();
        int cr = cur.first, cc = cur.second;
        for (int k = 0; k < 4; k++) {
            int nr = cr + dR[k], nc = cc + dC[k];
            if (nr < 0 || nr >= N || nc < 0 || nc >= N) continue;
            if (seen[nr][nc]) continue;
            if (field[nr][nc] == 0) { outR = nr; outC = nc; return true; }
            seen[nr][nc] = true; q.push({nr, nc});
        }
    }
    return false;
}

int aStar(int sr, int sc, int gr, int gc, int pathR[], int pathC[]) {
    static double gScore[N][N];
    static bool   closed[N][N];
    static int    cameR[N][N], cameC[N][N];
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            gScore[r][c] = 1e18; closed[r][c] = false;
            cameR[r][c] = -1; cameC[r][c] = -1;
        }
    std::priority_queue<std::pair<double,int>,
        std::vector<std::pair<double,int>>,
        std::greater<std::pair<double,int>>> open;
    gScore[sr][sc] = 0.0;
    open.push({heuristic(sr, sc, gr, gc), sr * N + sc});

    int    dR[8]       = {-1,  1,  0,  0, -1, -1,  1,  1};
    int    dC[8]       = { 0,  0, -1,  1, -1,  1, -1,  1};
    double stepCost[8] = {1.0,1.0,1.0,1.0,1.41421356,1.41421356,1.41421356,1.41421356};

    while (!open.empty()) {
        int idx = open.top().second; open.pop();
        int r = idx / N, c = idx % N;
        if (closed[r][c]) continue;
        closed[r][c] = true;
        if (r == gr && c == gc) break;
        for (int k = 0; k < 8; k++) {
            int nr = r + dR[k], nc = c + dC[k];
            if (nr < 0 || nr >= N || nc < 0 || nc >= N) continue;
            if (field[nr][nc] == 1) continue;
            if (dR[k] != 0 && dC[k] != 0) {                 // corner-cut guard
                if (field[r][nc] == 1 || field[nr][c] == 1) continue;
            }
            double ng = gScore[r][c] + stepCost[k] + clearanceCost[nr][nc];
            if (ng < gScore[nr][nc]) {
                gScore[nr][nc] = ng;
                cameR[nr][nc] = r; cameC[nr][nc] = c;
                open.push({ng + heuristic(nr, nc, gr, gc), nr * N + nc});
            }
        }
    }
    if (cameR[gr][gc] == -1 && !(sr == gr && sc == gc)) return 0;
    static int tmpR[N*N], tmpC[N*N]; int len = 0;
    int r = gr, c = gc;
    while (!(r == sr && c == sc)) {
        tmpR[len] = r; tmpC[len] = c; len++;
        int pr = cameR[r][c], pc = cameC[r][c]; r = pr; c = pc;
    }
    tmpR[len] = sr; tmpC[len] = sc; len++;
    for (int i = 0; i < len; i++) { pathR[i] = tmpR[len-1-i]; pathC[i] = tmpC[len-1-i]; }
    return len;
}

// ============================================================================
//  PATH POST-PROCESSING  — smoothing, merging, curving
// ============================================================================
bool lineOfSight(int r0, int c0, int r1, int c1) {
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = dc - dr, r = r0, c = c0;
    while (true) {
        if (field[r][c] == 1) return false;
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
    return true;
}

int smoothPath(int pathR[], int pathC[], int len, int smR[], int smC[]) {
    if (len == 0) return 0;
    int count = 0, anchor = 0;
    smR[0] = pathR[0]; smC[0] = pathC[0]; count++;
    int i = 1;
    while (i < len - 1) {
        if (lineOfSight(pathR[anchor], pathC[anchor], pathR[i+1], pathC[i+1])) i++;
        else { smR[count] = pathR[i]; smC[count] = pathC[i]; count++; anchor = i; i++; }
    }
    smR[count] = pathR[len-1]; smC[count] = pathC[len-1]; count++;
    return count;
}

int mergeClose(int wpR[], int wpC[], int n, int minDistCells, int outR[], int outC[]) {
    if (n <= 2) { for (int i = 0; i < n; i++) { outR[i] = wpR[i]; outC[i] = wpC[i]; } return n; }
    int count = 0;
    outR[0] = wpR[0]; outC[0] = wpC[0]; count++;
    double minSq = (double)minDistCells * minDistCells;
    for (int i = 1; i < n - 1; i++) {
        int lr = outR[count-1], lc = outC[count-1];
        double dr = wpR[i] - lr, dc = wpC[i] - lc;
        if (dr*dr + dc*dc >= minSq) { outR[count] = wpR[i]; outC[count] = wpC[i]; count++; }
    }
    outR[count] = wpR[n-1]; outC[count] = wpC[n-1]; count++;
    return count;
}

int catmullRom(double wpX[], double wpY[], int n, double outX[], double outY[], int samplesPerSeg) {
    if (n < 2) { if (n == 1) { outX[0]=wpX[0]; outY[0]=wpY[0]; return 1; } return 0; }
    int count = 0;
    for (int i = 0; i < n - 1; i++) {
        double p0x = wpX[(i==0)?0:i-1], p0y = wpY[(i==0)?0:i-1];
        double p1x = wpX[i],   p1y = wpY[i];
        double p2x = wpX[i+1], p2y = wpY[i+1];
        double p3x = wpX[(i+2>=n)?n-1:i+2], p3y = wpY[(i+2>=n)?n-1:i+2];
        for (int s = 0; s < samplesPerSeg; s++) {
            double t = (double)s/samplesPerSeg, t2 = t*t, t3 = t2*t;
            outX[count] = 0.5*((2*p1x)+(-p0x+p2x)*t+(2*p0x-5*p1x+4*p2x-p3x)*t2+(-p0x+3*p1x-3*p2x+p3x)*t3);
            outY[count] = 0.5*((2*p1y)+(-p0y+p2y)*t+(2*p0y-5*p1y+4*p2y-p3y)*t2+(-p0y+3*p1y-3*p2y+p3y)*t3);
            count++;
        }
    }
    outX[count] = wpX[n-1]; outY[count] = wpY[n-1]; count++;
    return count;
}

bool gpsPointIsFree(double x, double y) {
    int col = (int)round(x / CELL) + CENTER;
    int row = CENTER - (int)round(y / CELL);
    if (row < 0 || row >= N || col < 0 || col >= N) return false;
    return (field[row][col] == 0);
}
bool curveIsFree(double cx[], double cy[], int cn) {
    for (int i = 0; i < cn; i++) if (!gpsPointIsFree(cx[i], cy[i])) return false;
    return true;
}

// ============================================================================
//  DYNAMIC OBSTACLES  (other moving robots) — temporary blocks
// ============================================================================
const int MAX_DYN = 400;
int dynR[MAX_DYN], dynC[MAX_DYN];
int dynCount = 0;
const int ROBOT_HALF_CELLS = 3;     // 18" robot -> 9" half -> 3 cells

void markDynamicObstacle(int r, int c) {
    if (r < 0 || r >= N || c < 0 || c >= N) return;
    if (field[r][c] == 1) return;            // already blocked (real wall)
    if (dynCount >= MAX_DYN) return;
    field[r][c] = 1;
    dynR[dynCount] = r; dynC[dynCount] = c; dynCount++;
}
void markDynamicPatch(int r0, int c0, int halfCells) {
    for (int dr = -halfCells; dr <= halfCells; dr++)
        for (int dc = -halfCells; dc <= halfCells; dc++)
            markDynamicObstacle(r0 + dr, c0 + dc);
}
void clearDynamicObstacles() {
    for (int i = 0; i < dynCount; i++) field[dynR[i]][dynC[i]] = 0;
    dynCount = 0;
}

// ============================================================================
//  COLLISION HANDLING
// ============================================================================
bool reachedWaypoint(double wx, double wy) {
    int wr, wc, rr, rc;
    GPStoGrid(wx, wy, wr, wc);
    GPStoGrid(currX, currY, rr, rc);
    return (wr == rr && wc == rc);
}

void blockCellAhead() {
    double h = currH * M_PI / 180.0;
    double aheadX = currX + 2.0 * CELL * sin(h);   // ~6" ahead -> obstacle body
    double aheadY = currY + 2.0 * CELL * cos(h);
    int r, c;
    GPStoGrid(aheadX, aheadY, r, c);
    markDynamicPatch(r, c, ROBOT_HALF_CELLS);      // 18"-footprint patch
    computeClearanceCost();                         // field changed -> rebuild
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
    while (angle > 360.0)  angle -= 360.0;
    while (angle < 0) angle += 360.0;
    return angle;
}

void turnToAbsolute(double theta) {
  // PID constants - tune these for your robot
  double kP = 0.35;
  double kI = 0.03;
  double kD = 0.06;

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

void turnToReverse(double targetX, double targetY) {
    // Compute desired absolute heading toward target point.
    // Using atan2(dx, dy) because heading 0 = +Y axis, CW positive.
    double dx = targetX - currX;
    double dy = targetY - currY;

    // If we're basically on the target, no meaningful heading exists.
    if (std::sqrt(dx*dx + dy*dy) < 0.1) return;

    double targetHeading = std::atan2(dx, dy) * 180.0 / PI;  // degrees, CW from +Y
    
    turnToAbsolute(wrapAngle(wrapAngle(targetHeading) + 180.0));
}

void forwardStraight(double fdistance, double maxRPM = 600.0) {
    double fbcoef = 56;  // convert inches to motor degrees
    double distanceEncoder = fbcoef * fdistance;
    double currentPos = 0;
    double posDifference;
    double instantV = 0;
    double thresholdBegin = 10 * fbcoef;
    double thresholdEnd   = 30 * fbcoef;
    double minSpeed = 125.0;
    double maxSpeed = maxRPM;
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
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV*0.02,                 vex::voltageUnits::volt);
            
        } else if ((currentAngle - angle) < (-AngleTolerance)) {
            // turn left a little bit
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * 0.02,                 vex::voltageUnits::volt);
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV * percentChange*0.02, vex::voltageUnits::volt);
            
        } else {
            leftDriveSmart.spin(vex::directionType::fwd,  dir * instantV * 0.02, vex::voltageUnits::volt);
            rightDriveSmart.spin(vex::directionType::fwd, dir * instantV * 0.02, vex::voltageUnits::volt);
           
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
DETECTION_OBJECT findTarget(AI_RECORD local_map){
    DETECTION_OBJECT target;
    target.classID = -1;          // sentinel: -1 means "no target found"
    double lowestDist = 1e9;

    for (int i = 0; i < local_map.detectionCount; i++) {
        if (local_map.detections[i].classID != currColor) continue;

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

int blockColor(){ // 0 blue, 1 red, 2 none
    float hue; 
    hue = OpticalSensor.hue();

    if(hue < 20 && hue >0){return 1;}
    if(hue < 220 && hue >200){return 0;}
    return 2;
}

int autoIntakeColor() {
    
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
        while (!(blockColor() == currColor) && intakemotorrunning) {
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

    intake.setStopping(brakeType::hold);
    outtake.setStopping(brakeType::hold);
    return 1;
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

    intake.setStopping(brakeType::hold);
    outtake.setStopping(brakeType::hold);
    return 1;
}
void autoOuttakeHigh(int time, int voltage = 12) {

    outtake.spin(directionType::fwd, voltage, voltageUnits::volt);
    intake.spin(vex::directionType::fwd, voltage, voltageUnits::volt);

    vex::timer timer;
    timer.clear();

    while(timer.time(timeUnits::msec) < time) {
        if (currColor == 0 && blockColor() == 1) {
            outtake.stop();
            intake.stop();
            forwardStraight(5.0);
            outtake.spin(directionType::fwd, voltage, voltageUnits::volt);
            intake.spin(vex::directionType::fwd, voltage, voltageUnits::volt);
            while(!(blockColor() == currColor)){
                wait(10, timeUnits::msec);
            }
            outtake.stop();
            intake.stop();
            forwardStraight(-7.0);
            outtake.spin(directionType::fwd, voltage, voltageUnits::volt);
            intake.spin(vex::directionType::fwd, voltage, voltageUnits::volt);
        }
         else if (currColor == 1 && blockColor() == 0) { 
            outtake.stop();
            intake.stop();
            forwardStraight(5.0);
            outtake.spin(directionType::fwd, voltage, voltageUnits::volt);
            intake.spin(vex::directionType::fwd, voltage, voltageUnits::volt);
            while(!(blockColor() == currColor)){
                wait(10, timeUnits::msec);
            }
            outtake.stop();
            intake.stop();
            forwardStraight(-7.0);
            outtake.spin(directionType::fwd, voltage, voltageUnits::volt);
            intake.spin(vex::directionType::fwd, voltage, voltageUnits::volt);
        }
        wait(5, timeUnits::msec);
    }
    intake.stop();
    outtake.stop();
}

int autoOuttakeLow(){
    outtake_raiser.set(false);
    outtakemotorrunning = true;

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

// ============================================================================
//  PATH FIND TO  — plan, drive, detect collisions, replan, refresh on arrival
// ============================================================================
bool pathFindTo(double destX, double destY) {
    static bool clearanceReady = false;
    if (!clearanceReady) { computeClearanceCost(); clearanceReady = true; }

    const int    MAX_REPLANS = 6;
    const double BACKUP_IN    = 10.0;
    const int    MERGE_CELLS  = 3;

    bool success = false;

    for (int attempt = 0; attempt < MAX_REPLANS; attempt++) {
        int sr, sc, gr, gc;
        GPStoGrid(currX, currY, sr, sc);
        GPStoGrid(destX,  destY,  gr, gc);

        int sr2, sc2, gr2, gc2;
        if (!nearestFreeCell(sr, sc, sr2, sc2)) { Brain.Screen.print("No free start"); break; }
        if (!nearestFreeCell(gr, gc, gr2, gc2)) { Brain.Screen.print("No free dest");  break; }
        sr = sr2; sc = sc2; gr = gr2; gc = gc2;

        static int pathR[N*N], pathC[N*N];
        int len = aStar(sr, sc, gr, gc, pathR, pathC);
        if (len == 0) { Brain.Screen.print("No path found"); break; }

        static int smR[N*N], smC[N*N];
        int pts = smoothPath(pathR, pathC, len, smR, smC);

        static int mgR[N*N], mgC[N*N];
        pts = mergeClose(smR, smC, pts, MERGE_CELLS, mgR, mgC);

        static double wpX[N*N], wpY[N*N];
        for (int i = 0; i < pts; i++) gridToGPS(mgR[i], mgC[i], wpX[i], wpY[i]);

        static double curveX[N*N], curveY[N*N];
        int cn = catmullRom(wpX, wpY, pts, curveX, curveY, 8);
        bool useCurve = curveIsFree(curveX, curveY, cn);

        // Drive either the safe curve or the straight waypoints.
        int driveN      = useCurve ? cn  : pts;
        double *dx      = useCurve ? curveX : wpX;
        double *dy      = useCurve ? curveY : wpY;

        bool blocked = false;
        for (int i = 1; i < driveN; i++) {
            Controller1.Screen.setCursor(1,1);
            Controller1.Screen.print("(%.1f, %.1f)   ", dx[i], dy[i]);
            moveToPosition(dx[i], dy[i]);

            if (!reachedWaypoint(dx[i], dy[i])) {
                Controller1.Screen.setCursor(2,1);
                Controller1.Screen.print("Blocked! replanning");
                forwardStraight(-BACKUP_IN);     // reverse 10"
                blockCellAhead();      // mark moving-robot footprint + refresh
                blocked = true;
                break;
            }
        }
        if (!blocked) { success = true; break; }
    }

    // Refresh the map: moving-robot blocks are temporary, wipe them all.
    clearDynamicObstacles();
    computeClearanceCost();

    if (!success) Brain.Screen.print("Gave up after max replans");
    return success;
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
            autoOuttakeHigh(time);
        } else if (location == RED_MID_LEFT || location == BLUE_MID_LEFT){
            forwardStraight(-8.0); // Drive forward to score
            autoOuttakeHigh(time);
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

void intakeLoader(){

    leftDriveSmart.setStopping(brakeType::hold);
    rightDriveSmart.setStopping(brakeType::hold);

    vex::timer loaderTime; 
    loaderTime.clear();

    leftDriveSmart.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);

    while (loaderTime.time(vex::timeUnits::msec) < 3000){
        wait(10, timeUnits::msec);
    }

    leftDriveSmart.stop();
    rightDriveSmart.stop();
    
}

void auton_isolation(){

    DrivetrainInertial.setHeading(180, rotationUnits::deg);
    leftDriveSmart.setStopping(brakeType::hold);
    rightDriveSmart.setStopping(brakeType::hold);
    intake.setStopping(brakeType::hold);
    outtake.setStopping(brakeType::hold);

    vex::timer autonTimer;
    autonTimer.clear();

    intakemotorrunning = true;
    vex::task t1(autoIntake);
    forwardStraight(23.5);
    turnToAbsolute(270);
    loader.set(true);
    forwardStraight(10.0);

    intakeLoader();

    outtake_raiser.set(true);
    forwardStraight(-27);

    intakemotorrunning = false;

    autoOuttakeHigh( (16 - autonTimer.time(vex::timeUnits::sec))*1000, 9);

}


void auton_interaction(){

    int timeToRest = 500;
    double distance, angle, target_x, target_y;
    DETECTION_OBJECT target;

    // DrivetrainInertial.setHeading(270, rotationUnits::deg);

    forwardStraight(12);
    // outtake_raiser.set(false);
    
    // start looking 

    angle = 175;
    
    vex::timer location1timer;
    location1timer.clear();

    while (location1timer.time(vex::timeUnits::sec) < 30){

        turnToAbsolute(angle);
        
        wait(timeToRest, timeUnits::msec);
        target = findTarget(local_map);

        target_x = target.mapLocation.x / 0.0254;
        target_y = target.mapLocation.y / 0.0254;
        distance = distanceTo(target_x, target_y) - 2;

        if(target.classID != -1){

            if( (target_x < -68 && target_y < - 68) || (target_x < -70) || (target_x > -24) || (target_y > -15)){

            }else{ // ok to intake
                intakemotorrunning = true;
                vex::task t5(autoIntakeColor);
                
                turnTo(target_x, target_y);
                turnToRelative(2);
                
                forwardStraight(distance, 355.0);
                //intake one more ball already

                wait(timeToRest, timeUnits::msec);
                turnToReverse(-36, -46);
                distance = distanceTo(-36, -46);
                forwardStraight(-distance);

                wait(timeToRest, timeUnits::msec);
                turnToReverse(-23.75, -47);
                distance = distanceTo(-23.75, -47);
                intakemotorrunning = false;
                forwardStraight(-distance);

                autoOuttakeHigh(1500, 8);

                forwardStraight(12);
            }

        } 
        
        angle = angle - 30;
        
    }

    pathFindTo(23,-20);

    angle = 55;

    vex::timer location2timer;
    location2timer.clear();

    while (location2timer.time(vex::timeUnits::sec) < 30){

        turnToAbsolute(angle);
        
        wait(timeToRest, timeUnits::msec);
        target = findTarget(local_map);

        target_x = target.mapLocation.x / 0.0254;
        target_y = target.mapLocation.y / 0.0254;
        distance = distanceTo(target_x, target_y) - 2;

        if(target.classID != -1){

            if((target_x > 50 && target_y > -15) || (distance > 35)|| (target_x < 24) || (target_y > -15)){

            }else{ // ok to intake
                intakemotorrunning = true;
                vex::task t5(autoIntakeColor);
                
                turnTo(target_x, target_y);
                
                forwardStraight(distance, 355.0);
                //intake one more ball already

                wait(timeToRest, timeUnits::msec);
                turnToReverse(23,-20);
                distance = distanceTo(23,-20);
                forwardStraight(-distance);

                wait(timeToRest, timeUnits::msec);
                turnToReverse(0,0);
                intakemotorrunning = false;
                outtake_raiser.set(false);
                forwardStraight(-distanceTo(0,0) + 14);

                autoOuttakeHigh(2000, 9);

                forwardStraight(10);
            }

        }
        angle = angle + 25;
    }

    pathFindTo(-34, -27);
    

    turnToAbsolute(90);
    

    leftDriveSmart.spin(vex::directionType::rev, 10, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::rev, 10, vex::voltageUnits::volt);

    wait(2000, timeUnits::msec);

    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);

    forwardStraight(30);

    turnToAbsolute(0);

    leftDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    while(currY < -5){
        wait(5, timeUnits::msec);
    }
    
    leftDriveSmart.stop(brake);
    rightDriveSmart.stop(brake);

    turnToAbsolute(90);

    odomraiser.set(true);
    wait(300, timeUnits::msec);

    leftDriveSmart.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
    rightDriveSmart.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
    wait(5000, timeUnits::msec);
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


    if (Controller1.ButtonB.pressing()) { 
        auton_isolation();
        // forwardStraight(24);
    }
    // if (Controller1.ButtonB.pressing()) { 
    //     DETECTION_OBJECT target = findTarget(local_map);
    //     if (target.classID != -1){
    //         turnTo(target.mapLocation.x / 0.0254, target.mapLocation.y / 0.0254);
    //         forwardStraight(distanceTo(target.mapLocation.x / 0.0254, target.mapLocation.y / 0.0254), 300.0);
    //     }
    // }

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