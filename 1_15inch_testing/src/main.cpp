/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"

using namespace vex;

competition Competition;

brain Brain;
controller Controller1 = controller(primary);

motor frontLeft = motor(PORT12, ratio6_1, true);
motor backLeft = motor(PORT11, ratio6_1, true);
motor frontRight = motor(PORT18, ratio6_1, false); 
motor backRight = motor(PORT20, ratio6_1, false);
inertial DrivetrainInertial = inertial(PORT2);
gps GPS = gps(PORT8, 121.6, -40, distanceUnits::mm, 90);
motor intake = motor(PORT3, ratio6_1, true);
motor outtake = motor(PORT4, ratio6_1, false);
motor slideMotor1 = motor(PORT17, ratio18_1, false);
motor slideMotor2 = motor(PORT16, ratio18_1, true);
limit limitSwitch = limit(Brain.ThreeWirePort.H);
optical OpticalSensor = optical(PORT5);
distance FrontDis = distance(PORT15);

motor_group leftDriveSmart = motor_group(frontLeft, backLeft);
motor_group rightDriveSmart = motor_group(frontRight, backRight);
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, GPS, 219.44, 320, 40, mm, 1.3333333333333333);

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
// #define MANAGER_ROBOT 1

// #if defined(MANAGER_ROBOT)
// #pragma message("building for the manager")
// ai::robot_link       link(PORT7, "6599A_AI_robot", linkType::manager );
// #else
// #pragma message("building for the worker")
ai::robot_link       link(PORT7, "6599A_AI_robot", linkType::worker );
// #endif

bool firstAutoFlag = true;

void autonomousMain(void) {
  if(firstAutoFlag)
    auton_isolation();
  else 
    auton_interaction();
  
  firstAutoFlag = false;
}

int main() {

  static AI_RECORD local_map; // local storage for latest data from the Jetson Nano
  
  int32_t loop_time = 33; // Run at about 15Hz

  GPS.calibrate();
  waitUntil(!(GPS.isCalibrating()));

  DrivetrainInertial.calibrate();
  waitUntil(!(DrivetrainInertial.isCalibrating()));
  
  DrivetrainInertial.setHeading(GPS.heading(), rotationUnits::deg);
  


  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);
  Competition.drivercontrol(teleop);
    // autonomousMain();

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  // this_thread::sleep_for(loop_time);

  // while(1) {
  //     // get last map data
  //     jetson_comms.get_data( &local_map );

  //     // set our location to be sent to partner robot
  //     link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

  //     // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

  //     // request new data    
  //     // NOTE: This request should only happen in a single task.    
  //     jetson_comms.request_map();

  //     // Allow other tasks to run
  //     this_thread::sleep_for(loop_time);
  // }
}