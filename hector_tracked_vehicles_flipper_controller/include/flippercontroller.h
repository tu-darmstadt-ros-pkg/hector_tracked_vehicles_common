#ifndef FLIPPERCONTROLLER_H
#define FLIPPERCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

namespace hector_tracked_vehicles_flipper_controller {

class FlipperController
{
public:
    FlipperController();
    void setup();
    void process();
private:

    void handleJointStates(sensor_msgs::JointStateConstPtr msg);

    void handleCommandAbsolute(std_msgs::Float64ConstPtr msg);
    void handleCommandRelative(std_msgs::Float64ConstPtr msg);

    //parameter

    double flipper_tolerance;

    //variables

    double current_flipper_state;
    double lastFlipperCommand;

    bool is_last_recieved_relative;
    ros::Time time_last_relative;
    double relative_movement;

    bool is_last_recieved_absolute;
    double absolute_target;

    // ROS node handle
    ros::NodeHandle node_handle_;

    //Subscriber

    ros::Subscriber jointStateSub_;
    ros::Subscriber commandAbsSub_;
    ros::Subscriber commandRelSub_;

    // Publisher
    ros::Publisher flipperCommandPub_;
    ros::Publisher flipperStatePub_;


};

}
#endif // FLIPPERCONTROLLER_H
