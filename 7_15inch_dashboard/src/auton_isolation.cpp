#include "vex.h"
#include "ai_functions.h"
#include "auton_isolation.h"
#include <string>
#include <iostream>
using namespace vex;
using namespace std;

void auton_isolation(){
  GPS.calibrate();
  waitUntil(!(GPS.isCalibrating()));

  
}

