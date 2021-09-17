#include "base.hpp"




Base::Base(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices){
    init(n, controllerName, devices);
}


void Base::init(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices){
    m1 = new Motor(n, controllerName, devices[18]);
    m2 = new Motor(n, controllerName, devices[20]);
    m3 = new Motor(n, controllerName, devices[22]);
    m4 = new Motor(n, controllerName, devices[24]);
    gotoInit();
}

void Base::reset(){
    m1->setVel(0.0);
    m2->setVel(0.0);
    m3->setVel(0.0);
    m4->setVel(0.0);
}
void Base::goForward(){
    m1->setVel(SPEED);
    m2->setVel(SPEED);
    m3->setVel(SPEED);
    m4->setVel(SPEED);
}
void Base::goBackward(){
    m1->setVel(-SPEED);
    m2->setVel(-SPEED);
    m3->setVel(-SPEED);
    m4->setVel(-SPEED);
}
void Base::goLeft(){
    m1->setVel(SPEED);
    m2->setVel(-SPEED);
    m3->setVel(-SPEED);
    m4->setVel(SPEED);
}
void Base::goRight(){
    m1->setVel(-SPEED);
    m2->setVel(SPEED);
    m3->setVel(SPEED);
    m4->setVel(-SPEED);
}
void Base::turnRight(){
    m1->setVel(-SPEED);
    m2->setVel(SPEED);
    m3->setVel(-SPEED);
    m4->setVel(SPEED);
}
void Base::turnLeft(){
    m1->setVel(SPEED);
    m2->setVel(-SPEED);
    m3->setVel(SPEED);
    m4->setVel(-SPEED);
}


void Base::setCompassData(double x, double y, double z){
    this->compass_x = x;
    this->compass_y = y;
    this->compass_z = z;
}
void Base::setGPSData(double lat, double alt, double lng){
    this->gps_lat = lat;
    this->gps_alt = alt;
    this->gps_long = lng;
}
void Base::gotoInit(){
    goto_data.v_target.u = 0.0;
    goto_data.v_target.v = 0.0;
    goto_data.alpha = 0.0;
    goto_data.reached = false;
}
void Base::gotoSetTarget(double x, double z, double alpha){
    goto_data.v_target.u = x;
    goto_data.v_target.v = z;
    goto_data.alpha = alpha;
    goto_data.reached = false;
}
void Base::gotoRun(){

    // compute 2d vectors
    Vector2 v_gps = {gps_lat, gps_long};
    Vector2 v_front = {compass_x, compass_y};
    Vector2 v_right = {-v_front.v, v_front.u};
    Vector2 v_north = {1.0, 0.0};

    // compute distance
    Vector2 v_dir;
    vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
    double distance = vector2_norm(&v_dir);

    // compute absolute angle & delta with the delta with the target angle
    double theta = vector2_angle(&v_front, &v_north);
    double delta_angle = theta - goto_data.alpha;

    // compute the direction vector relatively to the robot coordinates
    // using an a matrix of homogenous coordinates
    Matrix33 transform;
    matrix33_set_identity(&transform);
    transform.a.u = v_front.u;
    transform.a.v = v_right.u;
    transform.b.u = v_front.v;
    transform.b.v = v_right.v;
    transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
    transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
    Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
    Vector3 v_target_rel;
    matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

    // compute the speeds
    double speeds[4] = {0.0, 0.0, 0.0, 0.0};
    // -> first stimulus: delta_angle
    speeds[0] = -delta_angle / M_PI * K1;
    speeds[1] = delta_angle / M_PI * K1;
    speeds[2] = -delta_angle / M_PI * K1;
    speeds[3] = delta_angle / M_PI * K1;

    // -> second stimulus: u coord of the relative target vector
    speeds[0] += v_target_rel.u * K2;
    speeds[1] += v_target_rel.u * K2;
    speeds[2] += v_target_rel.u * K2;
    speeds[3] += v_target_rel.u * K2;

    // -> third stimulus: v coord of the relative target vector
    speeds[0] += -v_target_rel.v * K3;
    speeds[1] += v_target_rel.v * K3;
    speeds[2] += v_target_rel.v * K3;
    speeds[3] += -v_target_rel.v * K3;

    // apply the speeds
    int i;
    for (i = 0; i < 4; i++) {
        speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
        speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)

        // added an arbitrary factor increasing the convergence speed
        speeds[i] *= 30.0;
        speeds[i] = bound(speeds[i], -SPEED, SPEED);
    }
    m1->setVel(speeds[0]);
    m2->setVel(speeds[1]);
    m3->setVel(speeds[2]);
    m4->setVel(speeds[3]);

    // check if the taget is reached
    if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
        goto_data.reached = true;
}

bool Base::goalReached(){
    return goto_data.reached;
}

Motor::Motor(ros::NodeHandle* n, std::string controllerName, std::string motor_name){
    motorP = n->serviceClient<webots_ros::set_float>(controllerName + '/' + motor_name + "/set_position");
    motorV = n->serviceClient<webots_ros::set_float>(controllerName + '/' + motor_name + "/set_velocity");
    setPosition.request.value = 0;
    setVelocity.request.value = 0;
}

void Motor::setVel(double vel){
    setPosition.request.value = INFINITY;
    setVelocity.request.value = vel;
    motorP.call(setPosition);
    motorV.call(setVelocity);
}



