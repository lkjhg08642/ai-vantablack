#include "autons.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127

float Vratio = 12000.0/static_cast<float>(pros::battery::get_voltage());
const int DRIVE_SPEED = 110*Vratio;
const int TURN_SPEED = 80*Vratio;
const int SWING_SPEED = 80*Vratio;
bool intakemotorrunning = false;
bool outtakemotorrunning = false;
double startTime;
float tempdist;
float x_in, y_in;


///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

int blockColor(){ // 1 is red, 2 is blue, 0 is none
  float hue; 
  hue = OpticalSensor.get_hue();

  if(hue < 20 && hue >0){return 1;}
  if(hue < 220 && hue >200){return 2;}
  return 0;
}

float actualIMU(){
  
  float chassisIMU;

  chassisIMU = chassis.drive_imu_get();

  if(chassisIMU < 0){
    //keep adding 360 until it is >0
    while(chassisIMU < 0){
      chassisIMU += 360;
    }
  } else if(chassisIMU >= 360){
    //keep minus 360 until it is <360
    while(chassisIMU >=360){
      chassisIMU-=360;
    }
  } else {
  }

  return chassisIMU;

}

int autoIntake(){
  intakemotorrunning = true;
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while(intakemotorrunning){
    intake.move(127);
    outtake.move(0);
    pros::delay(1000);
    while(abs(intake.get_actual_velocity())>130 && intakemotorrunning){
      pros::delay(20);
    }
    outtake.move(80);
    while(OpticalSensor.get_proximity()< 200 && intakemotorrunning) {
      pros::delay(20);
    }
    outtake.move(0);
    while(intake.get_actual_velocity()>100 && intakemotorrunning){
      pros::delay(20);
    }
    if (intakemotorrunning){
      intake.move(0);
      pros::delay(100);
    }
  }

  intake.move(0);
  intakemotorrunning = false;
  return 1;
}

int autoOuttake(){
  outtakemotorrunning = true;
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  intake.move(127);
  outtake.move(127);
  pros::delay(600);

  while(outtakemotorrunning){
    while(abs(intake.get_actual_velocity())>5 && abs(outtake.get_actual_velocity())>5 && outtakemotorrunning){
      pros::delay(20);
    }

    if(outtakemotorrunning){
      intake.move(-127);
      outtake.move(-127);
      pros::delay(200);
      intake.move(127);
      outtake.move(127);
      pros::delay(400);
    }
  }

  intake.move(0);
  outtake.move(0);
  outtakemotorrunning = false;
  return 1;
}

void intakeLoader(){
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  startTime = pros::millis();  
  while(pros::millis()-startTime < 1500){
    pros::delay(250);
    chassis.pid_odom_set(-0.05_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(0.08_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
  }
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
}


void skills_24in() {

  //initializations
  loader.set_value(false);
  chassis.odom_enable(true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);
  odomraiser.set_value(false);
  chassis.odom_x_set(15);
  chassis.odom_y_set(22);
  chassis.odom_theta_set(90);




  //end of parking test 


  pros::delay(200);

  //driving to first loader
  chassis.pid_odom_set(29.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(181, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //take from 1st loader
  loader.set_value(true);
  pros::delay(300);

  pros::Task t1(autoIntake);

  chassis.pid_odom_set(8_in, 0.5*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  //1st loader
  intakeLoader();

  //scoring in first goal
  outtake_raiser.set_value(true);

  chassis.pid_odom_set(-35_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t0(autoOuttake);
  pros::delay(2500);
  outtakemotorrunning = false;
  loader.set_value(false);

  //leaving first goal
  chassis.pid_odom_set(15_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(23_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(5, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //going to second loader
  chassis.pid_odom_set(85_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-6_in, 0.5*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(true);

  //taking from second loader
  chassis.pid_odom_set(8_in, 0.5*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  
  pros::Task t1_5(autoIntake);

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  chassis.pid_odom_set(tempdist, DRIVE_SPEED);
  chassis.pid_wait();

  //2nd loader
  intakeLoader();
  pros::delay(600);
  outtake_raiser.set_value(true);
  
  //leaving second loader
  chassis.pid_odom_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_turn_set(90, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //going to second goal
  chassis.pid_odom_set(-92.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //outtaking second loader blocks in second goal
  chassis.pid_odom_set(-16_in, 0.6*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  
  pros::Task t2(autoOuttake);
  pros::delay(3000);
  outtakemotorrunning = false;

  //going to third loader
  chassis.pid_turn_set(2, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(8_in, 0.5*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::delay(100);
  pros::Task t3(autoIntake);
  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  chassis.pid_odom_set(tempdist, 0.75*DRIVE_SPEED);
  chassis.pid_wait();

  //3rd loader
  intakeLoader();
  pros::delay(1000);

  chassis.pid_turn_set(0, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //outtaking into second goal from third loader
  chassis.pid_odom_set(-35_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t5(autoOuttake);
  pros::delay(3000);
  outtakemotorrunning = false;

    chassis.pid_odom_set(8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  //aligns
  chassis.pid_odom_set(42_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  odomraiser.set_value(true);

  chassis.pid_turn_set(0, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_odom_set(50_in, 1.5*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(5000);

}


void auton_start(){

  chassis.odom_enable(true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);

  chassis.odom_x_set(15);
  chassis.odom_y_set(22);
  chassis.odom_theta_set(90);

  pros::delay(200);

  chassis.pid_odom_set(27_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(true);
  pros::delay(300);

  pros::Task t1(autoIntake);

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  // while(true){
  //   master.print(0,0,"%f", tempdist);
  //   pros::delay(1000);
  // }
  // around 6.5 inches
  chassis.pid_odom_set(tempdist, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_odom_set(6.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //1st loader
  intakeLoader();

  outtake_raiser.set_value(true);

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set({35_in,0_in}, fwd, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-23_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180.5, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(false);
  intakemotorrunning = false;
  
  chassis.pid_odom_set(-57_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  while (FrontDis.get_distance()<300){
    chassis.pid_odom_set(-0.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    pros::delay(20);
  }

  chassis.pid_turn_set(1, 0.7* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t2(autoOuttake);

  pros::delay(3500);

  loader.set_value(true);
  outtakemotorrunning = false;

  pros::Task t3(autoIntake);

  chassis.pid_odom_set(23.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(5_in, 0.7*DRIVE_SPEED);
  chassis.pid_wait();
  

  //2nd loader
  intakeLoader();
  
  chassis.pid_odom_set(-29_in, DRIVE_SPEED);
  chassis.pid_wait();

  intakemotorrunning = false;
  pros::delay(500);

  pros::Task t4(autoOuttake);

  pros::delay(3400);

  outtakemotorrunning = false;

  chassis.pid_odom_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(97.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_odom_set(57-x_in, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();

  // while(true){
  //   master.print(0,0,"%f   %f", x_in, y_in);
  //   pros::delay(1000);
  // }

  chassis.pid_turn_set(1, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t5(autoIntake);

  chassis.pid_odom_set(4_in, DRIVE_SPEED);
  chassis.pid_wait();

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  chassis.pid_odom_set(tempdist, 0.8*DRIVE_SPEED);
  chassis.pid_wait();

  //3rd loader
  intakeLoader();

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(40, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(2, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(false);
  intakemotorrunning = false;
  
  chassis.pid_odom_set(-60_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  while (FrontDis.get_distance()<300){
    chassis.pid_odom_set(-0.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    pros::delay(20);
  }

  chassis.pid_turn_set(180, 0.7* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t6(autoOuttake);

  pros::delay(3400);

  loader.set_value(true);
  outtakemotorrunning = false;

  pros::Task t7(autoIntake);

  chassis.pid_odom_set(23.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(4.5_in, 0.7*DRIVE_SPEED);
  chassis.pid_wait();

  //4th loader
  intakeLoader();

  chassis.pid_turn_set(181, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-29_in, DRIVE_SPEED);
  chassis.pid_wait();

  intakemotorrunning = false;
  pros::delay(500);

  pros::Task t8(autoOuttake);

  pros::delay(3400);

  outtakemotorrunning = false;

}

void auton1() {
  
  auton_start();

  //park from the front

  chassis.pid_odom_set(5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(false);

  chassis.pid_turn_set(90, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(45_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  odomraiser.set_value(true);

  intake.move(127);
  outtake.move(127);

  chassis.pid_drive_constants_set(10000, 0,0);  
  chassis.pid_odom_set(100_in, 127);
  chassis.pid_wait();

}

void auton2() {
  
  auton_start();

  // park from the side

  chassis.pid_odom_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(false);
  pros::delay(200);
  
  chassis.pid_turn_set(145, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(17_in, 0.8*DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(false);
  descore.set_value(true);

  chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 90, 30);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(1_in, DRIVE_SPEED);
  chassis.pid_wait();

  loader.set_value(true);
  intake.move(-127);
  odomraiser.set_value(true); //raise the odom wheel
  
  chassis.pid_drive_constants_set(10000, 0,0);  
  pros::delay(250);

  chassis.pid_drive_set(100_in, 127);
  pros::delay(500);

  float ypos = GPSsensor.get_position_y();
  while (ypos < 1){
    ypos = GPSsensor.get_position_y();
    pros::delay(25);
  }

}



void auton4(){ //red blocks only

  OpticalSensor.set_led_pwm(100);
  chassis.odom_enable(true);

  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);

  chassis.odom_x_set(0);
  chassis.odom_y_set(0);
  chassis.odom_theta_set(0);

  pros::delay(200);

  loader.set_value(true);

  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  intake.move(127);
  outtake.move(127);

  int color;

  while(true){
    color = OpticalSensor.get_hue();
    if(color > 200 && color <220){
      intake.move(0);
      outtake.move(0);
      chassis.pid_odom_set(4_in, DRIVE_SPEED);
      chassis.pid_wait_quick_chain();
      intake.move(127);
      outtake.move(127);
      pros::delay(300);
      intake.move(0);
      outtake.move(0);
      chassis.pid_odom_set(-4_in, DRIVE_SPEED);
      chassis.pid_wait_quick_chain();
      intake.move(127);
      outtake.move(127);
    }
    pros::delay(20);
  }

}

void auton5(){ //blue blocks only

  OpticalSensor.set_led_pwm(100);
  chassis.odom_enable(true);

  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);

  chassis.odom_x_set(0);
  chassis.odom_y_set(0);
  chassis.odom_theta_set(0);

  pros::delay(200);

  loader.set_value(true);

  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  intake.move(127);
  outtake.move(127);

  int color;

  while(true){
    color = OpticalSensor.get_hue();
    if(color > 0 && color <20){
      intake.move(0);
      outtake.move(0);
      chassis.pid_odom_set(4_in, DRIVE_SPEED);
      chassis.pid_wait_quick_chain();
      intake.move(127);
      outtake.move(127);
      pros::delay(300);
      intake.move(0);
      outtake.move(0);
      chassis.pid_odom_set(-4_in, DRIVE_SPEED);
      chassis.pid_wait_quick_chain();
      intake.move(127);
      outtake.move(127);
    }
    pros::delay(20);
  }

}


///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void auton_20(){

  chassis.odom_enable(true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);

  chassis.odom_x_set(15);
  chassis.odom_y_set(22);
  chassis.odom_theta_set(90);

  pros::delay(200);

  chassis.pid_odom_set(27_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(true);
  pros::delay(300);

  pros::Task t1(autoIntake);

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  // while(true){
  //   master.print(0,0,"%f", tempdist);
  //   pros::delay(1000);
  // }
  // around 6.5 inches
  chassis.pid_odom_set(tempdist, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_odom_set(6.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //1st loader
  intakeLoader();

  outtake_raiser.set_value(true);

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set({35_in,0_in}, fwd, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-22_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180.5, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(false);
  intakemotorrunning = false;
  
  chassis.pid_odom_set(-57_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  while (FrontDis.get_distance()<300){
    chassis.pid_odom_set(-0.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    pros::delay(20);
  }

  chassis.pid_turn_set(1, 0.7* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t2(autoOuttake);

  pros::delay(3500);

  loader.set_value(true);
  outtakemotorrunning = false;

  pros::Task t3(autoIntake);

  chassis.pid_odom_set(23.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(5_in, 0.7*DRIVE_SPEED);
  chassis.pid_wait();
  

  //2nd loader
  intakeLoader();
  
  chassis.pid_odom_set(-29_in, DRIVE_SPEED);
  chassis.pid_wait();

  intakemotorrunning = false;
  pros::delay(500);

  pros::Task t4(autoOuttake);

  pros::delay(3400);

  outtakemotorrunning = false;

  chassis.pid_odom_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(97.5_in, 30);
  chassis.pid_wait();


}

void auton_30(){

  chassis.odom_enable(true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_turn_behavior_set(ez::shortest);

  chassis.odom_x_set(15);
  chassis.odom_y_set(22);
  chassis.odom_theta_set(90);

  pros::delay(200);

  chassis.pid_odom_set(27_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  loader.set_value(true);
  pros::delay(300);

  pros::Task t1(autoIntake);

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  // while(true){
  //   master.print(0,0,"%f", tempdist);
  //   pros::delay(1000);
  // }
  // around 6.5 inches
  chassis.pid_odom_set(tempdist, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_odom_set(6.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //1st loader
  intakeLoader();

  outtake_raiser.set_value(true);

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set({35_in,0_in}, fwd, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-22_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180.5, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(false);
  intakemotorrunning = false;
  
  chassis.pid_odom_set(-57_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  while (FrontDis.get_distance()<300){
    chassis.pid_odom_set(-0.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    pros::delay(20);
  }

  chassis.pid_turn_set(1, 0.7* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t2(autoOuttake);

  pros::delay(3500);

  loader.set_value(true);
  outtakemotorrunning = false;

  pros::Task t3(autoIntake);

  chassis.pid_odom_set(23.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(5_in, 0.7*DRIVE_SPEED);
  chassis.pid_wait();
  

  //2nd loader
  intakeLoader();
  
  chassis.pid_odom_set(-29_in, DRIVE_SPEED);
  chassis.pid_wait();

  intakemotorrunning = false;
  pros::delay(500);

  pros::Task t4(autoOuttake);

  pros::delay(3400);

  outtakemotorrunning = false;

  chassis.pid_odom_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(97.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_odom_set(57-x_in, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();

  // while(true){
  //   master.print(0,0,"%f   %f", x_in, y_in);
  //   pros::delay(1000);
  // }

  chassis.pid_turn_set(1, 0.75* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t5(autoIntake);

  chassis.pid_odom_set(4_in, DRIVE_SPEED);
  chassis.pid_wait();

  tempdist = 0;
  for (int i=0;i<5;i++){tempdist = FrontDis.get_distance();}
  for (int i=0;i<5;i++){tempdist += FrontDis.get_distance();}
  tempdist/=5;
  tempdist = tempdist / 25.4 - 9;

  chassis.pid_odom_set(tempdist, 0.8*DRIVE_SPEED);
  chassis.pid_wait();

  //3rd loader
  intakeLoader();

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(40, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(2, 0.8* TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  loader.set_value(false);
  intakemotorrunning = false;
  
  chassis.pid_odom_set(-60_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  while (FrontDis.get_distance()<300){
    chassis.pid_odom_set(-0.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();
    pros::delay(20);
  }

  chassis.pid_turn_set(180, 0.7* TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set(-8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  pros::Task t6(autoOuttake);

  pros::delay(3400);

  loader.set_value(true);
  outtakemotorrunning = false;

  pros::Task t7(autoIntake);

  chassis.pid_odom_set(23.5_in, 30);
  chassis.pid_wait_quick_chain();


}

void auton_40(){
  auton_start();
}