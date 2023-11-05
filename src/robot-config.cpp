#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

#ifdef ROBOT1 //54198k
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT15, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT13, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonRight = led(Brain.ThreeWirePort.C); //Extension 1
  led PistonLeft = led(Brain.ThreeWirePort.B); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.H); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT2//DCH
   motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT18, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT15, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonRight = led(Brain.ThreeWirePort.C); //Extension 1
  led PistonLeft = led(Brain.ThreeWirePort.B); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.C); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT3//WU
  motor Motor_BaseLF = motor(PORT2, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT4, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT9, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT11, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT13, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT14, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT21, ratio36_1, true);

  inertial IMU = inertial(PORT20);
  led PistonHook = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonRight = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonLeft = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
