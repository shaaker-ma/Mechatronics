// File:          ubot.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

enum JointPosition {
  ARM_FRONT_FLOOR,
  ARM_FRONT_PLATE,
  ARM_HANOI_PREPARE,
  ARM_FRONT_CARDBOARD_BOX,
  ARM_RESET,
  ARM_BACK_PLATE_HIGH,
  ARM_BACK_PLATE_LOW,
  ARM_MAX_HEIGHT
};

enum Orientation {
  ARM_BACK_LEFT,
  ARM_LEFT,
  ARM_FRONT_LEFT,
  ARM_FRONT,
  ARM_FRONT_RIGHT,
  ARM_RIGHT,
  ARM_BACK_RIGHT,
  ARM_MAX_SIDE
};


class MyController: public Robot{
public:
  MyController(){
    // Arm init
    arm1 = getMotor("arm1");
    arm2 = getMotor("arm2");
    arm3 = getMotor("arm3");
    arm4 = getMotor("arm4");
    arm5 = getMotor("arm5");
    
    arm2->setVelocity(0.5);
    arm_reset();
  }

  // Arm functionallity
  void arm_reset(){
    arm1->setPosition(0.0);
    arm2->setPosition(1.57);
    arm3->setPosition(-2.635);
    arm4->setPosition(1.78);
    arm4->setPosition(0.0);
  }
  void arm_set_position(JointPosition height) {
    switch (height) {
      case ARM_FRONT_FLOOR:
        arm2->setPosition(-0.97);
        arm3->setPosition(-1.55);
        arm4->setPosition(-0.61);
        arm5->setPosition(0.0);
        break;
      case ARM_FRONT_PLATE:
        arm2->setPosition(-0.62);
        arm3->setPosition(-0.98);
        arm4->setPosition(-1.53);
        arm5->setPosition(0.0);
        break;
      case ARM_FRONT_CARDBOARD_BOX:
        arm2->setPosition(0.0);
        arm3->setPosition(-0.77);
        arm4->setPosition(-1.21);
        arm5->setPosition(0.0);
        break;
      case ARM_RESET:
        arm2->setPosition(1.57);
        arm3->setPosition(-2.635);
        arm4->setPosition(1.78);
        arm5->setPosition(0.0);
        break;
      case ARM_BACK_PLATE_HIGH:
        arm2->setPosition(0.678);
        arm3->setPosition(0.682);
        arm4->setPosition(1.74);
        arm5->setPosition(0.0);
        break;
      case ARM_BACK_PLATE_LOW:
        arm2->setPosition(0.92);
        arm3->setPosition(0.42);
        arm4->setPosition(1.78);
        arm5->setPosition(0.0);
        break;
      case ARM_HANOI_PREPARE:
        arm2->setPosition(-0.4);
        arm3->setPosition(-1.2);
        arm4->setPosition(-M_PI_2);
        arm5->setPosition(M_PI_2);
        break;
      default:
        fprintf(stderr, "arm_height() called with a wrong argument\n");
        return;
    }
    currentJointState = height;
  }
  void arm_set_orientation(Orientation orientation) {
    switch (orientation) {
      case ARM_BACK_LEFT:
        arm1->setPosition(-2.949);
        break;
      case ARM_LEFT:
        arm1->setPosition(-M_PI_2);
        break;
      case ARM_FRONT_LEFT:
        arm1->setPosition(-0.2);
        break;
      case ARM_FRONT:
        arm1->setPosition(0.0);
        break;
      case ARM_FRONT_RIGHT:
        arm1->setPosition(0.2);
        break;
      case ARM_RIGHT:
        arm1->setPosition(M_PI_2);
        break;
      case ARM_BACK_RIGHT:
        arm1->setPosition(2.949);
        break;
      default:
        fprintf(stderr, "arm_set_side() called with a wrong argument\n");
        return;
    }
    currentOrientation = orientation;
  }
  void _arm_set_orientation(double orientation){
    arm1->setPosition(orientation);
  }
  void _set_arm_joint_state(int arm, double radian){
    switch (arm){
      case 1:
        arm1->setPosition(radian);
        break;
      case 2:
        arm2->setPosition(radian);
        break;
      case 3:
        arm3->setPosition(radian);
        break;
      case 4:
        arm4->setPosition(radian);
        break;
      case 5:
        arm5->setPosition(radian);
        break;
      default:
        fprintf(stderr, "wrong arm idx");
        return;
    }
  }
  
  double arm_get_sub_arm_length(int arm) {
    switch (arm) {
      case 1:
        return 0.253;
      case 2:
        return 0.155;
      case 3:
        return 0.135;
      case 4:
        return 0.081;
      case 5:
        return 0.105;
    }
    return 0.0;
  }
  void arm_ik(double x, double y, double z) {
    double x1 = sqrt(x * x + z * z);
    double y1 = y + arm_get_sub_arm_length(4) + arm_get_sub_arm_length(5) - arm_get_sub_arm_length(1);

    double a = arm_get_sub_arm_length(2);
    double b = arm_get_sub_arm_length(3);
    double c = sqrt(x1 * x1 + y1 * y1);

    double alpha = -asin(z / x1);
    double beta = -(M_PI_2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(y1 / x1));
    double gamma = -(M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b)));
    double delta = -(M_PI + (beta + gamma));
    double epsilon = M_PI_2 + alpha;

    _set_arm_joint_state(1, alpha);
    _set_arm_joint_state(2, beta);
    _set_arm_joint_state(3, gamma);
    _set_arm_joint_state(4, delta);
    _set_arm_joint_state(5, epsilon);
  }

private:
  int time_step = 10;
  
  Motor* arm1;
  Motor* arm2;
  Motor* arm3;
  Motor* arm4;
  Motor* arm5;
  JointPosition currentJointState;
  Orientation currentOrientation;

};

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  MyController* controller = new MyController();

  // get the time step of the current world.
  int timeStep = (int)controller->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (controller->step(timeStep) != -1) {
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete controller;
  return 0;
}
