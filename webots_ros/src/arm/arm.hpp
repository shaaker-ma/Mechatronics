#include <cmath>
#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>


#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/save_image.h>
#include <eigen3/Eigen/Dense>


#define TIME_STEP 32

#define A1 0.033
#define A2 0.155
#define A3 0.135
#define A4 0.0
#define A5 0.0

#define B1 0.147 
#define B2 0.0
#define B3 0.0
#define B4 0.0
#define B5 0.2175

#define AA1 (M_PI / 2.0)
#define AA2 0.0
#define AA3 0.0
#define AA4 (M_PI/ 2.0)
#define AA5 0.0

enum Height {
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


class ArmMotor{
public:
    ArmMotor(ros::NodeHandle* n, std::string controllerName, std::string motor_name);
    void setPosition(float radian);
private:
    ros::ServiceClient arm_motor;
    webots_ros::set_float arm_setPosition;

};


enum ARM{
    ARM1,
    ARM2,
    ARM3,
    ARM4,
    ARM5
};



class Gripper{
public:
    Gripper(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices);
    void release();
    void grip();
    void gripper_set_gap(double gap);
    void whereAreYou();
private:
    ros::ServiceClient gripper1_position;
    webots_ros::set_float gripper1_position_srv;

    ros::ServiceClient gripper2_position;
    webots_ros::set_float gripper2_position_srv;

    ros::ServiceClient gripper1_velocity;
    webots_ros::set_float gripper1_velocity_srv;

    ros::ServiceClient gripper2_velocity;
    webots_ros::set_float gripper2_velocity_srv;

    double bound(double v, double a, double b) {
        return (v > b) ? b : (v < a) ? a : v;
    }

    ros::ServiceClient gripper1_getPos;
    webots_ros::get_float gripper1_getPos_srv;

    ros::ServiceClient gripper2_getPos;
    webots_ros::get_float gripper2_getPos_srv;

    double min_pos = 0.0;
    double max_pos = 0.025;
    double offset_when_locked = 0.021;

};


class Arm{
public:
    Arm(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices);
    void gotoHome();
    void setMotorPosition(ARM arm, double radian);
    void arm_set_height(enum Height height);
    void arm_set_orientation(enum Orientation orientation);
    void arm_increase_height();
    Gripper* ee;
    int current_height = ARM_RESET;
    void arm_ik(double x, double y, double z);
    double arm_get_sub_arm_length(enum ARM arm);
private:
    ArmMotor* arm1;
    ArmMotor* arm2;
    ArmMotor* arm3;
    ArmMotor* arm4;
    ArmMotor* arm5;

};