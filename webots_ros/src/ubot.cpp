
#include "robot/robot.hpp"


static int controllerCount;
static std::vector<std::string> controllerList;
static std::vector<float> imageRangeFinder;

// ARM1    -2.9496   ->  2.9496
// ARM2    -1.13446  ->  1.5708
// ARM3    -2.54     ->  2.54
// ARM4    -1.78024  ->  1.78024
// ARM5    -2.92343  ->  2.92343



Robot* robot;

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}


void quit(int sig) {
    robot->resetTimeStep();
    ROS_INFO("User stopped the 'ubot' node.");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    std::string controllerName;
    std::vector<std::string> deviceList;


    ros::init(argc, argv, "ubot", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();

    // if there is more than one controller available, let the user choose
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for  controller choice.");
        return 1;
        }
    }
    // leave topic once it's not necessary anymore
    nameSub.shutdown();

    // call device_list service to get the list of the devices available on the controller and print it the device_list_srv object
    // contains 2 members request and response. Their fields are described in the corresponding .srv file
    ros::ServiceClient deviceListClient = n.serviceClient<webots_ros::robot_get_device_list>(controllerName + "/robot/get_device_list");
    webots_ros::robot_get_device_list deviceListSrv;

    if (deviceListClient.call(deviceListSrv)) deviceList = deviceListSrv.response.list;
    else ROS_ERROR("Failed to call service device_list.");

    std::cout << controllerName << std::endl;
    robot = new Robot(n, controllerName, deviceList);
    robot->initArm(&n, controllerName, deviceList);
    for (auto i: deviceList){
        std::cout << i << std::endl;
    }

    
    // Test Application
	std::thread t1([=](){
		robot->run();
	});


	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	t1.join();

	ROS_INFO("Done");
    sleep(1);

	ros::shutdown();
    return 0;


}
