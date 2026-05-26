using namespace vex;

extern brain Brain;
extern controller Controller1;

extern motor frontLeft;
extern motor backLeft;
extern motor frontRight; 
extern motor backRight;
extern inertial DrivetrainInertial;
extern gps GPS;
extern motor intake;
extern motor outtake;
extern motor slideMotor1;
extern motor slideMotor2;
extern optical OpticalSensor;
extern distance FrontDis;
extern limit limitSwitch;

extern motor_group leftDriveSmart;
extern motor_group rightDriveSmart;;
extern smartdrive Drivetrain;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
