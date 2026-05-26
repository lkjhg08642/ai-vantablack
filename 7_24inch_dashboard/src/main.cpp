/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "auton_isolation.h"
#include "auton_interaction.h"

using namespace vex;

competition Competition;

brain Brain;
controller Controller1 = controller(primary);

motor lf  = motor(PORT15, ratio6_1, false);
motor lm = motor(PORT4, ratio6_1, true);
motor lr  = motor(PORT5, ratio6_1, true);

motor rf = motor(PORT8, ratio6_1, true);
motor rm = motor(PORT7, ratio6_1, false);
motor rr = motor(PORT10, ratio6_1, false);

motor_group leftDriveSmart = motor_group(lf, lm, lr);
motor_group rightDriveSmart = motor_group(rf, rm, rr);

inertial DrivetrainInertial = inertial(PORT3);

gps GPS = gps(PORT12, -152.40, 0.00, mm, 270);
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, GPS, 219.44, 320, 40, mm, 1.3333333333333333);

rotation odom = rotation(PORT6, false);

motor intake = motor(PORT9, ratio6_1, false);
motor outtake = motor(PORT2, ratio6_1, true);
digital_out outtake_raiser = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.B);
digital_out loader = digital_out(Brain.ThreeWirePort.H);
digital_out odomraiser = digital_out(Brain.ThreeWirePort.G);
optical OpticalSensor = optical(PORT19);
distance FrontDis = distance(PORT18);

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
ai::robot_link       link(PORT1, "6599A_AI_robot", linkType::manager );
// #else
// #pragma message("building for the worker")
// ai::robot_link       link(PORT1, "6599A_AI_robot", linkType::worker );
// #endif
 
void auto_Isolation(void) {
  auton_isolation();
}

void auto_Interaction(void) {
  auton_interaction();
}

bool firstAutoFlag = true;

void autonomousMain(void) {
  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();
    // auto_Isolation();
  
  firstAutoFlag = false;
}

bool intakerunning = false;
bool outtake_raised = false;
bool descore_raised = false;
bool loader_dropped = false;
bool odom_raised = false;

int autoIntakeFunc() {
    intakerunning = true;
    
    // Configure standard motor brake modes
    intake.setStopping(brakeType::brake);
    outtake.setStopping(brakeType::hold);

    while (intakerunning) {
        // Spin intake at full speed (100%) and hold the outtake still
        intake.spin(directionType::fwd, 100, velocityUnits::pct);
        outtake.spin(directionType::fwd, 0, velocityUnits::pct);
        
        // Initial 1-second delay
        wait(1000, timeUnits::msec);
        
        // Monitor intake motor speed drop-off (stalling/loading checks)
        while (std::abs(intake.velocity(velocityUnits::rpm)) > 400 && intakerunning) {
            wait(20, timeUnits::msec);
        }
        
        // Feed the outtake roller at 80% power
        outtake.spin(directionType::fwd, 80, velocityUnits::pct);
        
        // Poll proximity sensor (VEX Optical scales natively from 0 to 100)
        while (!OpticalSensor.isNearObject() && intakerunning) {
            wait(20, timeUnits::msec);
        }
        
        // Stop outtake and monitor clearing speeds
        outtake.spin(directionType::fwd, 0, velocityUnits::pct);
        
        while (intake.velocity(velocityUnits::rpm) > 100 && intakerunning) {
            wait(20, timeUnits::msec);
        }
        
        // Settle delay before wrapping the loop iteration
        if (intakerunning) {
            intake.spin(directionType::fwd, 0, velocityUnits::pct);
            wait(300, timeUnits::msec);
        }
    }

    // Explicit shutdown cleanup safely halting elements
    intake.spin(directionType::fwd, 0, velocityUnits::pct);
    intakerunning = false;
    return 1;
}

void teleop(void) {

  OpticalSensor.setLightPower(100);
  
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
    leftDriveSmart.spin(forward, leftSpeed, percent);
    rightDriveSmart.spin(forward, rightSpeed, percent);
   

    if (Controller1.ButtonX.pressing()) { 
        if (intakerunning) {
            intakerunning = false;
        } else {
            intakerunning = true;
            vex::task t1(autoIntakeFunc); 
        }
        wait(250, msec); // Debounce delay
    }

    if (Controller1.ButtonR1.pressing()) {
        intake.spin(forward, 100, percent);
        outtake.spin(forward, 100, percent);
        intakerunning = false; 
    }
    else if (Controller1.ButtonR2.pressing()) {
        intake.spin(reverse, 100, percent);
        outtake.spin(reverse, 100, percent);
        intakerunning = false;
    }
    else {
        // If no manual buttons are held and the macro task is inactive, completely stall rollers
        if (!intakerunning) {
            intake.spin(forward, 0, percent);
            outtake.spin(forward, 0, percent);
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

int main() {

  static AI_RECORD local_map; // local storage for latest data from the Jetson Nano

  int32_t loop_time = 33; // Run at about 15Hz

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
    //   Competition.autonomous(autonomousMain);
    //   Competition.drivercontrol(teleop);
    autonomousMain();

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  while(1) {
      // get last map data
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}