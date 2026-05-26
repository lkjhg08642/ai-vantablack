#include "vex.h"
#include "ai_functions.h"
#include "auton_isolation.h"
#include <string>
#include <iostream>
using namespace vex;
using namespace std;

static AI_RECORD local_map;

int n = 0;
void auton_isolation(){
  GPS.calibrate();
  waitUntil(!(GPS.isCalibrating()));
  DETECTION_OBJECT target;
  double lowestDist = 0;
  
  while (true){
    jetson_comms.get_data(&local_map);
    link.set_remote_location(local_map.pos.x, local_map.pos.y,
                              local_map.pos.az, local_map.pos.status);
    jetson_comms.request_map();

    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(n);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(local_map.detectionCount);

    if (local_map.detectionCount != 0) {
      lowestDist = distanceTo(local_map.detections[0].mapLocation.x, local_map.detections[0].mapLocation.y);
    }
    else {}
    
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("x: %.2f, y: %.2f\n", local_map.detections[0].mapLocation.x, local_map.detections[0].mapLocation.y);
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == (int) OBJECT::BallBlue) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print(lowestDist);

    double x = target.mapLocation.x;
    double y = target.mapLocation.y;
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("x: %.2f, y: %.2f\n", x, y);
    n++;
    this_thread::sleep_for(std::chrono::milliseconds(500));
    Controller1.Screen.clearScreen();
    //goToObject(OBJECT::BallBlue);
  }

}  

