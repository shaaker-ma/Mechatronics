

#include "../arm/arm.hpp"
#include "../base/base.hpp"


class Robot{
public:
    Robot(ros::NodeHandle& n, std::string name, std::vector<std::string> devices);
    void initGPS(ros::NodeHandle* n);
    void initCompass(ros::NodeHandle* n);
    void initCamera(ros::NodeHandle* n);
    void initArm(ros::NodeHandle* n, std::string controllerName, std::vector<std::string> devices);
    
    void compassCallback(const sensor_msgs::MagneticField::ConstPtr &values);
    void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values);
    void GPSSpeedCallback(const webots_ros::Float64Stamped::ConstPtr &value);
    void cameraCallback(const sensor_msgs::Image::ConstPtr &values);
    
    bool checkTimeStep();
    void resetTimeStep();
    void passive_wait(double sec);
    
    void gotoPos(double x, double z, double a);
    void gripBox(double x, int level, int column, bool grip);
    void stock(Orientation o, bool stock);

    void run();

    
private:
    Arm* arm;
    Base* base;
    ros::ServiceClient timeStepClient;
    webots_ros::set_int timeStepSrv;
    std::string controller_name;

    ros::Subscriber sub_GPS_speed;
    ros::Subscriber sub_GPS_32;
    ros::Subscriber sub_compass_32;
};

