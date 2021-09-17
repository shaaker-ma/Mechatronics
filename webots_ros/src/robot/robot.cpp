#include "robot.hpp"


Robot::Robot(ros::NodeHandle& n, std::string name, std::vector<std::string> devices){
    this->controller_name = name;

    timeStepClient = n.serviceClient<webots_ros::set_int>(name + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;
    base = new Base(&n, name, devices);
    initCompass(&n);
    initGPS(&n);
    // initCamera(&n);
    
}

void Robot::initGPS(ros::NodeHandle* n){
    ros::ServiceClient set_GPS_client;
    webots_ros::set_int GPS_srv;
    
    
    set_GPS_client = n->serviceClient<webots_ros::set_int>(controller_name + "/gps/enable");

    ros::ServiceClient sampling_period_GPS_client;
    webots_ros::get_int sampling_period_GPS_srv;
    sampling_period_GPS_client = n->serviceClient<webots_ros::get_int>(controller_name + "/gps/get_sampling_period");

    GPS_srv.request.value = 32;
    if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
        ROS_INFO("GPS enabled.");
        sub_GPS_32 = n->subscribe(controller_name + "/gps/values", 1, &Robot::GPSCallback, this);
        while (sub_GPS_32.getNumPublishers() == 0) {
            ros::spinOnce();
            timeStepClient.call(timeStepSrv);
        }
        timeStepClient.call(timeStepSrv);

        sub_GPS_speed = n->subscribe(controller_name + "/gps/speed", 1, &Robot::GPSSpeedCallback, this);
        while (sub_GPS_speed.getNumPublishers() == 0) {
            ros::spinOnce();
            timeStepClient.call(timeStepSrv);
        }
    } else {
        if (!GPS_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
            ROS_ERROR("Failed to enable GPS.");
    }

    timeStepClient.call(timeStepSrv);
    ros::spinOnce();
}

void Robot::initCompass(ros::NodeHandle* n){
    ros::ServiceClient set_compass_client;
    webots_ros::set_int compass_srv;
    
    set_compass_client = n->serviceClient<webots_ros::set_int>(controller_name + "/compass/enable");

    ros::ServiceClient sampling_period_compass_client;
    
    compass_srv.request.value = 32;
    if (set_compass_client.call(compass_srv) && compass_srv.response.success == 1) {
        ROS_INFO("Compass enabled.");
        sub_compass_32 = n->subscribe(controller_name + "/compass/values", 1, &Robot::compassCallback, this);
        while (sub_compass_32.getNumPublishers() == 0) {
            ros::spinOnce();
            timeStepClient.call(timeStepSrv);
        }
    } 
    else {
        if (compass_srv.response.success == -1)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable compass.");
    }
    // sub_compass_32.shutdown();
    // while(1){
    //     timeStepClient.call(timeStepSrv);
    //     ros::spinOnce();
    // }
    webots_ros::get_int sampling_period_compass_srv;
    sampling_period_compass_client = n->serviceClient<webots_ros::get_int>(controller_name + "/compass/get_sampling_period");
    sampling_period_compass_client.call(sampling_period_compass_srv);
    ROS_INFO("Compass is enabled with a sampling period of %d.", sampling_period_compass_srv.response.value);
    // for (int i = 0 ; i < 50 ; i++){
    //     ros::spinOnce();
    // }
}
void Robot::compassCallback(const sensor_msgs::MagneticField::ConstPtr &values){
    float compassValues[3];
    compassValues[0] = values->magnetic_field.x;
    compassValues[1] = values->magnetic_field.y;
    compassValues[2] = values->magnetic_field.z;
    this->base->setCompassData(values->magnetic_field.x, values->magnetic_field.y, values->magnetic_field.z);

    // ROS_INFO("Compass values are x=%f y=%f z=%f (time: %d:%d).", compassValues[0], compassValues[1], compassValues[2],
    //         values->header.stamp.sec, values->header.stamp.nsec);
}

void Robot::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
    float GPSValues[3];
    GPSValues[0] = values->latitude;
    GPSValues[1] = values->altitude;
    GPSValues[2] = values->longitude;
    base->setGPSData(values->latitude, values->altitude, values->longitude);
    // ROS_INFO("GPS values are x=%f y=%f z=%f (time: %d:%d).", GPSValues[0], GPSValues[1], GPSValues[2], values->header.stamp.sec,
    //         values->header.stamp.nsec);
}
void Robot::GPSSpeedCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
    // ROS_INFO("GPS speed is: %fkm/h (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
}
void Robot::initArm(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices){
    arm = new Arm(n, controllerName, devices);
    
}
bool Robot::checkTimeStep(){
    return (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success);
}
void Robot::resetTimeStep(){
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
}

void Robot::passive_wait(double sec) {
    double start_time = ros::Time::now().toSec();
    do {
        checkTimeStep();
        ros::spinOnce();
    } while (start_time + sec > ros::Time::now().toSec());
}

void Robot::initCamera(ros::NodeHandle* n){
    // camera_set_focal_distance
    double focal_distance = 0.33;
    ros::ServiceClient camera_set_client = n->serviceClient<webots_ros::set_float>(controller_name + "/kinect_color/set_focal_distance");
    webots_ros::set_float camera_set_focal_distance_srv;
    camera_set_focal_distance_srv.request.value = focal_distance;
    if (camera_set_client.call(camera_set_focal_distance_srv) && camera_set_focal_distance_srv.response.success)
        ROS_INFO("Camera focal distance set to %f.", focal_distance);
    else
        ROS_ERROR("Failed to call service camera_set_focal_distance.");

    camera_set_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera_set_fov
    double fov = 1.33;
    camera_set_client = n->serviceClient<webots_ros::set_float>(controller_name + "/kinect_color/set_fov");
    webots_ros::set_float camera_set_fov_srv;
    camera_set_fov_srv.request.value = fov;
    if (camera_set_client.call(camera_set_fov_srv) && camera_set_fov_srv.response.success)
        ROS_INFO("Camera fov set to %f.", fov);
    else
        ROS_ERROR("Failed to call service camera_set_fov.");

    camera_set_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera enable
    ros::ServiceClient enable_camera_client;
    webots_ros::set_int camera_srv;
    ros::Subscriber sub_camera_color;

    enable_camera_client = n->serviceClient<webots_ros::set_int>(controller_name + "/kinect_color/enable");
    camera_srv.request.value = TIME_STEP;
    if (enable_camera_client.call(camera_srv) && camera_srv.response.success) {
        ROS_INFO("Camera enabled.");
        sub_camera_color = n->subscribe(controller_name + "/kinect_color/image", 1, &Robot::cameraCallback, this);
        ROS_INFO("Topic for camera color initialized.");
        while (sub_camera_color.getNumPublishers() == 0) {
        ros::spinOnce();
        timeStepClient.call(timeStepSrv);
        }
        ROS_INFO("Topic for camera color connected.");
    } else {
        if (camera_srv.response.success == -1)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable camera.");
    }

    sub_camera_color.shutdown();
    enable_camera_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera_get_info
    ros::ServiceClient get_info_client = n->serviceClient<webots_ros::camera_get_info>(controller_name + "/kinect_color/get_info");
    webots_ros::camera_get_info get_info_srv;
    if (get_info_client.call(get_info_srv))
        ROS_INFO("Camera of %s has a width of %d, a height of %d, a field of view of %f, a near range of %f.", controller_name.c_str(),
                get_info_srv.response.width, get_info_srv.response.height, get_info_srv.response.Fov,
                get_info_srv.response.nearRange);
    else
        ROS_ERROR("Failed to call service camera_get_info.");
    if (get_info_srv.response.Fov != fov)
        ROS_ERROR("Failed to set camera fov.");

    get_info_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera_get_focus_info
    get_info_client = n->serviceClient<webots_ros::camera_get_focus_info>(controller_name + "/kinect_color/get_focus_info");
    webots_ros::camera_get_focus_info camera_get_focus_info_srv;
    if (get_info_client.call(camera_get_focus_info_srv))
        ROS_INFO("Camera of %s has focalLength %f, focalDistance %f, maxFocalDistance %f, and minFocalDistance %f.",
                controller_name.c_str(), camera_get_focus_info_srv.response.focalLength,
                camera_get_focus_info_srv.response.focalDistance, camera_get_focus_info_srv.response.maxFocalDistance,
                camera_get_focus_info_srv.response.minFocalDistance);
    else
        ROS_ERROR("Failed to call service camera_get_focus_info.");
    if (camera_get_focus_info_srv.response.focalDistance != focal_distance)
        ROS_ERROR("Failed to set camera focal distance.");

    get_info_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera_get_zoom_info
    get_info_client = n->serviceClient<webots_ros::camera_get_zoom_info>(controller_name + "/kinect_color/get_zoom_info");
    webots_ros::camera_get_zoom_info camera_get_zoom_info_srv;
    if (get_info_client.call(camera_get_zoom_info_srv))
        ROS_INFO("Camera of %s has min fov %f, anf max fov %f.", controller_name.c_str(), camera_get_zoom_info_srv.response.minFov,
                camera_get_zoom_info_srv.response.maxFov);
    else
        ROS_ERROR("Failed to call service camera_get_zoom_info.");

    get_info_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // check presence of recognition capability
    get_info_client = n->serviceClient<webots_ros::get_bool>(controller_name + "/kinect_color/has_recognition");
    webots_ros::get_bool camera_has_recognition_srv;
    if (get_info_client.call(camera_has_recognition_srv))
        if (camera_has_recognition_srv.response.value)
        ROS_INFO("Recognition capability of camera of %s found.", controller_name.c_str());
        else
        ROS_ERROR("Recognition capability of camera of %s not found.", controller_name.c_str());
    else
        ROS_ERROR("Failed to call service camera_get_zoom_info.");

    get_info_client.shutdown();
    timeStepClient.call(timeStepSrv);

    // camera_save_image
    ros::ServiceClient save_image_client = n->serviceClient<webots_ros::save_image>(controller_name + "/kinect_color/save_image");
    webots_ros::save_image save_image_srv;
    save_image_srv.request.filename = std::string(getenv("HOME")) + std::string("/test_image_camera.png");
    save_image_srv.request.quality = 100;

    if (save_image_client.call(save_image_srv) && save_image_srv.response.success == 1)
        ROS_INFO("Image saved.");
    else
        ROS_ERROR("Failed to call service save_image.");

    save_image_client.shutdown();
    timeStepClient.call(timeStepSrv);

    ROS_INFO("Camera disabled.");
}

void Robot::cameraCallback(const sensor_msgs::Image::ConstPtr &values) {
    std::vector<unsigned char> imageColor;
    int i = 0;
    imageColor.resize(values->step * values->height);
    for (std::vector<unsigned char>::const_iterator it = values->data.begin(); it != values->data.end(); ++it) {
        imageColor[i] = *it;
        i++;
    }
}




void Robot::gotoPos(double x, double z, double a){
    base->gotoSetTarget(x, z, a);
    while(!base->goalReached()){
        base->gotoRun();
        checkTimeStep();
        ros::spinOnce();
    }
    base->reset();

}

void Robot::stock(Orientation o, bool stock) {
  arm->arm_set_height(ARM_BACK_PLATE_HIGH);
  arm->arm_set_orientation(o);
  passive_wait(4.5);
  if (stock){
      arm->ee->release();
  }
  else{
      arm->ee->gripper_set_gap(0.04);
  }
  passive_wait(1.0);
  arm->arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(3.0);
}


void Robot::gripBox(double x, int level, int column, bool grip) {
    static double h_per_step = 0.002;
    static double box_length = 0.05;
    static double box_gap = 0.01;
    static double platform_height = 0.01;
    static double offset = 0.005;  // security margin

    double y = offset + platform_height + (level + 1) * box_length;
    double z = 0.5 * column * (box_gap + box_length);
    z *= 0.9;  // This fix a small offset that I cannot explain

    if (!grip)
        y += offset;

    // prepare
    arm->setMotorPosition(ARM5, M_PI_2);
    arm->arm_ik(x, 0.20, z);
    if (grip)
        arm->ee->release();
    passive_wait(1.0);

    // move the arm down
    double h;
    for (h = 0.2; h > y; h -= h_per_step) {
        arm->arm_ik(x, h, z);
        checkTimeStep();
        ros::spinOnce();
    }

    passive_wait(0.2);

    // grip or ungrip
    if (grip)
        arm->ee->gripper_set_gap(0.04);
    else
        arm->ee->release();
    passive_wait(1.0);

    // move the arm up
    for (h = y; h < 0.2; h += h_per_step) {
        arm->arm_ik(x, h, z);
        checkTimeStep();
        ros::spinOnce();
    }
    arm->arm_set_orientation(ARM_FRONT);
}



void Robot::run(){
    arm->gotoHome();
    base->reset();
    // arm->ee->grip();
    passive_wait(5);
    // arm->ee->release();
    // ROS_INFO("Initiating has been done.");
    double distance_arm0_platform = 0.2;
    double distance_arm0_robot_center = 0.189;
    double distance_origin_platform = 1.0;
    double angles[3] = {0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
    int GOTO_SRC = 0, GOTO_TMP = 1, GOTO_DST = 2;

    double delta = distance_origin_platform - distance_arm0_platform - distance_arm0_robot_center;
    double goto_info[3][3] = {{delta * cos(angles[0]), delta * sin(angles[0]), -angles[0]},
                                {delta * cos(angles[1]), delta * sin(angles[1]), -angles[1]},
                                {delta * cos(angles[2]), delta * sin(angles[2]), -angles[2]}};

    arm->arm_set_height(ARM_HANOI_PREPARE);
    // passive_wait(4);
    // for (double i = distance_arm0_platform; i > 0.09 ; i -= 0.01){
    //     std::cout << i << std::endl;
    //     arm->arm_ik(i, 0.20, 0.0);
    //     passive_wait(1);
    //     checkTimeStep();
    //     ros::spinOnce();
    // }
    
    // SRC A1 => DST
    gotoPos(goto_info[GOTO_SRC][0], goto_info[GOTO_SRC][1], goto_info[GOTO_SRC][2]);
    gripBox(distance_arm0_platform, 2, 0, true);
    gotoPos(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
    gripBox(distance_arm0_platform, 0, 0, false);
    // SRC B1 & B2 => TMP
    gotoPos(goto_info[GOTO_SRC][0], goto_info[GOTO_SRC][1], goto_info[GOTO_SRC][2]);
    gripBox(distance_arm0_platform, 1, 1, true);
    stock(ARM_FRONT, true);
    gripBox(distance_arm0_platform, 1, -1, true);
    gotoPos(goto_info[GOTO_TMP][0], goto_info[GOTO_TMP][1], goto_info[GOTO_TMP][2]);
    gripBox(distance_arm0_platform, 0, -1, false);
    stock(ARM_FRONT, false);
    gripBox(distance_arm0_platform, 0, 1, false);
    // // DST A1 => TMP
    // gotoPos(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
    // gripBox(distance_arm0_platform, 0, 0, true);
    // gotoPos(goto_info[GOTO_TMP][0], goto_info[GOTO_TMP][1], goto_info[GOTO_TMP][2]);
    // gripBox(distance_arm0_platform, 1, 0, false);
    // // SRC C1-C2-C3 => DST
    // gotoPos(goto_info[GOTO_SRC][0], goto_info[GOTO_SRC][1], goto_info[GOTO_SRC][2]);
    // gripBox(distance_arm0_platform, 0, -2, true);
    // stock(ARM_FRONT_LEFT, true);
    // gripBox(distance_arm0_platform, 0, 0, true);
    // stock(ARM_FRONT_RIGHT, true);
    // gripBox(distance_arm0_platform, 0, 2, true);
    // gotoPos(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
    // gripBox(distance_arm0_platform, 0, 2, false);
    // stock(ARM_FRONT_RIGHT, false);
    // gripBox(distance_arm0_platform, 0, 0, false);
    // stock(ARM_FRONT_LEFT, false);
    // gripBox(distance_arm0_platform, 0, -2, false);
    // // TMP A1 => SRC
    // gotoPos(goto_info[GOTO_TMP][0], goto_info[GOTO_TMP][1], goto_info[GOTO_TMP][2]);
    // gripBox(distance_arm0_platform, 1, 0, true);
    // gotoPos(goto_info[GOTO_SRC][0], goto_info[GOTO_SRC][1], goto_info[GOTO_SRC][2]);
    // gripBox(distance_arm0_platform, 0, 0, false);
    // // TMP B1 & B2 => DST
    // gotoPos(goto_info[GOTO_TMP][0], goto_info[GOTO_TMP][1], goto_info[GOTO_TMP][2]);
    // gripBox(distance_arm0_platform, 0, 1, true);
    // stock(ARM_FRONT, true);
    // gripBox(distance_arm0_platform, 0, -1, true);
    // gotoPos(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
    // gripBox(distance_arm0_platform, 1, -1, false);
    // stock(ARM_FRONT, false);
    // gripBox(distance_arm0_platform, 1, 1, false);
    // // SRC A1 => DST
    // gotoPos(goto_info[GOTO_SRC][0], goto_info[GOTO_SRC][1], goto_info[GOTO_SRC][2]);
    // gripBox(distance_arm0_platform, 0, 0, true);
    // gotoPos(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
    // gripBox(distance_arm0_platform, 2, 0, false);
    // // end behavior
    // arm->gotoHome();
    // gotoPos(0.0, 0.0, 0.0);

    ROS_INFO("Done");





    while (1){
        if (checkTimeStep()) {
            ROS_ERROR("Failed to call next step with time_step service.");
            exit(1);
        }
        ros::spinOnce();
    }
    
}
