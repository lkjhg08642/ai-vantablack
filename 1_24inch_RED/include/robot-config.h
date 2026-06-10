using namespace vex;

extern brain Brain;
extern controller Controller1;
extern motor lf;
extern motor lm;
extern motor lr;
extern motor rf;
extern motor rm;
extern motor rr;
extern motor_group leftDriveSmart;
extern motor_group rightDriveSmart;
extern inertial DrivetrainInertial;
extern gps GPS;
extern smartdrive Drivetrain;
extern rotation odom;
extern motor intake;
extern motor outtake;
extern digital_out outtake_raiser;
extern digital_out descore;
extern digital_out loader;
extern digital_out odomraiser;
extern optical OpticalSensor;
extern distance FrontDis;

extern float currX, currY, currH;
extern AI_RECORD local_map;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
