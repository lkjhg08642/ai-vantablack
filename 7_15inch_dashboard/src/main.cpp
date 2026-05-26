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

motor frontLeft = motor(PORT12, ratio6_1, true);
motor backLeft = motor(PORT11, ratio6_1, true);
motor frontRight = motor(PORT18, ratio6_1, false); 
motor backRight = motor(PORT20, ratio6_1, false);
inertial DrivetrainInertial = inertial(PORT2);
gps GPS = gps(PORT8, 152.40, -25.4, distanceUnits::mm, M_PI/2);
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

bool intakemotorrunning;

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
  
  firstAutoFlag = false;
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

void teleop(void) {

// Set slide motors to "hold"
    slideMotor1.setStopping(brakeType::brake);
    slideMotor2.setStopping(brakeType::brake);

    slideMotor1.setPosition(0, rotationUnits::deg);
    slideMotor2.setPosition(0, rotationUnits::deg);

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
    frontLeft.spin(forward, flSpeed, percent);
    backLeft.spin(forward, blSpeed, percent);
    frontRight.spin(forward, frSpeed, percent);
    backRight.spin(forward, brSpeed, percent);

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
            intake.spin(forward, 100, percent);
            outtake.spin(forward, 100, percent);
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
            intake.spin(reverse, 100, percent);
            outtake.spin(reverse, 100, percent);
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

    task::sleep(10);

  }
}

int main() {

  static AI_RECORD local_map; // local storage for latest data from the Jetson Nano

  int32_t loop_time = 33; // Run at about 15Hz

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);
  Competition.drivercontrol(teleop);

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