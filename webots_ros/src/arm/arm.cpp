#include "arm.hpp"


ArmMotor::ArmMotor(ros::NodeHandle* n, std::string controllerName, std::string motor_name){
    arm_motor = n->serviceClient<webots_ros::set_float>(controllerName + '/' + motor_name + "/set_position");
    arm_setPosition.request.value = 0;
}

void ArmMotor::setPosition(float radian){
    arm_setPosition.request.value = radian;
    arm_motor.call(arm_setPosition);
}



Gripper::Gripper(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices){
    gripper1_position = n->serviceClient<webots_ros::set_float>(controllerName + '/' + devices[14] + "/set_position");
    gripper1_velocity = n->serviceClient<webots_ros::set_float>(controllerName + '/' + devices[14] + "/set_velocity");
    gripper1_velocity_srv.request.value = 0.03;
    gripper1_velocity.call(gripper1_velocity_srv);

    gripper2_position = n->serviceClient<webots_ros::set_float>(controllerName + '/' + devices[16] + "/set_position");
    gripper2_velocity = n->serviceClient<webots_ros::set_float>(controllerName + '/' + devices[16] + "/set_velocity");
    gripper2_velocity_srv.request.value = 0.03;
    gripper2_velocity.call(gripper2_velocity_srv);

    // webots_ros::set_int setSensorSrv1;
    // webots_ros::set_int setSensorSrv2;
    // ros::ServiceClient SetClient1;
    // ros::ServiceClient SetClient2;
    // setSensorSrv1.request.value = 1;
    // setSensorSrv2.request.value = 1;
    // SetClient1 = n->serviceClient<webots_ros::set_int>(controllerName + "/" + devices[15] + "/position_sensor/enable");
    // SetClient2 = n->serviceClient<webots_ros::set_int>(controllerName + "/" + devices[17] + "/position_sensor/enable");

    // SetClient1.call(setSensorSrv1);
    // SetClient2.call(setSensorSrv2);
    
}

void Gripper::release(){
    gripper1_position_srv.request.value = max_pos;
    gripper1_position.call(gripper1_position_srv);
    gripper2_position_srv.request.value = max_pos;
    gripper2_position.call(gripper2_position_srv);
}

void Gripper::grip(){
    gripper1_position_srv.request.value = min_pos;
    gripper1_position.call(gripper1_position_srv);
    gripper2_position_srv.request.value = min_pos;
    gripper2_position.call(gripper2_position_srv);
}
void Gripper::gripper_set_gap(double gap) {
    double v = bound(0.5 * (gap - offset_when_locked), min_pos, max_pos);
    gripper1_position_srv.request.value = v;
    gripper1_position.call(gripper1_position_srv);
    gripper2_position_srv.request.value = v;
    gripper2_position.call(gripper2_position_srv);
}

void Gripper::whereAreYou(){
    gripper1_getPos.call(gripper1_getPos_srv);
    gripper2_getPos.call(gripper2_getPos_srv);
    std::cout << gripper1_getPos_srv.response.value << " " << gripper2_getPos_srv.response.value << std::endl;
}


Arm::Arm(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices){
    arm1 = new ArmMotor(n, controllerName, devices[4]);
    arm2 = new ArmMotor(n, controllerName, devices[6]);
    arm3 = new ArmMotor(n, controllerName, devices[8]);
    arm4 = new ArmMotor(n, controllerName, devices[10]);
    arm5 = new ArmMotor(n, controllerName, devices[12]);
    gotoHome();
    ee = new Gripper(n, controllerName, devices);
}
void Arm::gotoHome(){
    arm1->setPosition(0.0);
    arm2->setPosition(1.57);
    arm3->setPosition(-2.635);
    arm4->setPosition(1.78);
    arm5->setPosition(0.0);
}
void Arm::setMotorPosition(ARM arm, double radian){
    switch (arm){
    case ARM1:
        arm1->setPosition(radian);
        break;
    case ARM2:
        arm2->setPosition(radian);
        break;
    case ARM3:
        arm3->setPosition(radian);
        break;
    case ARM4:
        arm4->setPosition(radian);
        break;
    case ARM5:
        arm5->setPosition(radian);
        break;
    default:
        break;
    }
}

void Arm::arm_set_height(enum Height height) {
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
  current_height = height;
}

double Arm::arm_get_sub_arm_length(enum ARM arm) {
  switch (arm) {
    case ARM1:
      return 0.253;
    case ARM2:
      return 0.155;
    case ARM3:
      return 0.135;
    case ARM4:
      return 0.081;
    case ARM5:
      return 0.105;
  }
  return 0.0;
}


void Arm::arm_ik(double x, double y, double z) {
  double x1 = sqrt(x * x + z * z);
  double y1 = y + arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5) - arm_get_sub_arm_length(ARM1);

  double a = arm_get_sub_arm_length(ARM2);
  double b = arm_get_sub_arm_length(ARM3);
  double c = sqrt(x1 * x1 + y1 * y1);

  double alpha = -asin(z / x1);
  double beta = -(M_PI_2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(y1 / x1));
  double gamma = -(M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b)));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;

  arm1->setPosition(alpha);
  arm2->setPosition(beta);
  arm3->setPosition(gamma);
  arm4->setPosition(delta);
  arm5->setPosition(epsilon);
}


void Arm::arm_increase_height() {
  current_height++;
  if (current_height >= ARM_MAX_HEIGHT)
    current_height = ARM_MAX_HEIGHT - 1;
  arm_set_height((Height)current_height);
}

void Arm::arm_set_orientation(enum Orientation orientation) {
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
}