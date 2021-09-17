#include <signal.h>
#include <cmath>
#include <thread>
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <signal.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <webots_ros/BoolStamped.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Int8Stamped.h>
#include <webots_ros/RadarTarget.h>
#include <webots_ros/RecognitionObject.h>
#include <webots_ros/StringStamped.h>

#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_float_array.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

#include <webots_ros/camera_get_focus_info.h>
#include <webots_ros/camera_get_info.h>
#include <webots_ros/camera_get_zoom_info.h>
#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_draw_oval.h>
#include <webots_ros/display_draw_pixel.h>
#include <webots_ros/display_draw_polygon.h>
#include <webots_ros/display_draw_rectangle.h>
#include <webots_ros/display_draw_text.h>
#include <webots_ros/display_get_info.h>
#include <webots_ros/display_image_copy.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>
#include <webots_ros/display_image_save.h>
#include <webots_ros/display_set_font.h>
#include <webots_ros/field_get_bool.h>
#include <webots_ros/field_get_color.h>
#include <webots_ros/field_get_count.h>
#include <webots_ros/field_get_float.h>
#include <webots_ros/field_get_int32.h>
#include <webots_ros/field_get_node.h>
#include <webots_ros/field_get_rotation.h>
#include <webots_ros/field_get_string.h>
#include <webots_ros/field_get_type.h>
#include <webots_ros/field_get_type_name.h>
#include <webots_ros/field_get_vec2f.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_import_node.h>
#include <webots_ros/field_remove.h>
#include <webots_ros/field_set_bool.h>
#include <webots_ros/field_set_color.h>
#include <webots_ros/field_set_float.h>
#include <webots_ros/field_set_int32.h>
#include <webots_ros/field_set_rotation.h>
#include <webots_ros/field_set_string.h>
#include <webots_ros/field_set_vec2f.h>
#include <webots_ros/field_set_vec3f.h>
#include <webots_ros/lidar_get_frequency_info.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/motor_set_control_pid.h>
#include <webots_ros/node_add_force_or_torque.h>
#include <webots_ros/node_add_force_with_offset.h>
#include <webots_ros/node_get_center_of_mass.h>
#include <webots_ros/node_get_contact_point.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/node_get_id.h>
#include <webots_ros/node_get_name.h>
#include <webots_ros/node_get_number_of_contact_points.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/node_get_parent_node.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_static_balance.h>
#include <webots_ros/node_get_status.h>
#include <webots_ros/node_get_type.h>
#include <webots_ros/node_get_velocity.h>
#include <webots_ros/node_remove.h>
#include <webots_ros/node_reset_functions.h>
#include <webots_ros/node_set_velocity.h>
#include <webots_ros/node_set_visibility.h>
#include <webots_ros/pen_set_ink_color.h>
#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/receiver_get_emitter_direction.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/robot_set_mode.h>
#include <webots_ros/robot_wait_for_user_input_event.h>
#include <webots_ros/save_image.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/supervisor_get_from_id.h>
#include <webots_ros/supervisor_movie_start_recording.h>
#include <webots_ros/supervisor_set_label.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_orientation.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_position.h>

#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/save_image.h>
#include "../math/tiny_math.h"

struct goto_struct{
  Vector2 v_target;
  double alpha;
  bool reached;
};

#define SPEED 4.0
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0

class Motor{
public:
    Motor(ros::NodeHandle* n, std::string controllerName, std::string motor_name);
    void setVel(double vel);
private:
    ros::ServiceClient motorP;
    ros::ServiceClient motorV;
    webots_ros::set_float setPosition;
    webots_ros::set_float setVelocity;
};

class Base{
public:
    Base(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices);
    void init(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices);
    void setCompassData(double x, double y, double z);
    void setGPSData(double lat, double alt, double lng);

    void reset();
    void goForward();
    void goBackward();
    void goLeft();
    void goRight();
    void turnLeft();
    void turnRight();
    
    void gotoInit();
    void gotoSetTarget(double x, double z, double alpha);
    void gotoRun();
    bool goalReached();
private:
    Motor* m1;
    Motor* m2;
    Motor* m3;
    Motor* m4;
    goto_struct goto_data;

    double compass_x;
    double compass_y;
    double compass_z;

    double gps_lat;
    double gps_alt;
    double gps_long;
};