#include "flippercontroller.h"
#include <sensor_msgs/JointState.h>

namespace hector_tracked_vehicles_flipper_controller {

FlipperController::FlipperController()
{


}

void FlipperController::setup(){
    jointStateSub_ = node_handle_.subscribe("/joint_states", 1, &FlipperController::handleJointStates, this);
    commandAbsSub_ = node_handle_.subscribe("/flipper_control/command/absolute", 1, &FlipperController::handleCommandAbsolute, this);
    commandRelSub_ = node_handle_.subscribe("/flipper_control/command/relative", 1, &FlipperController::handleCommandRelative, this);
    flipperCommandPub_ = node_handle_.advertise<sensor_msgs::JointState>("/jointstate_cmd", 100);
    flipperStatePub_ = node_handle_.advertise<std_msgs::Float64>("/flipper_control/state", 100);

    ros::param::param("~flipper_tolerance", flipper_tolerance, 0.1);
}

void FlipperController::process(){

    std_msgs::Float64 current_msg;
    current_msg.data = current_flipper_state;
    flipperStatePub_.publish(current_msg);

    if(is_last_recieved_absolute){
        if(fabs(current_flipper_state - absolute_target) <= flipper_tolerance){
            is_last_recieved_absolute = false;
            lastFlipperCommand = current_flipper_state;
        }
    }else if(is_last_recieved_relative){
        if(ros::Time::now() - time_last_relative < ros::Duration(0.5) ){
        double target = current_flipper_state + relative_movement;
        if (fabs(target - lastFlipperCommand) >=  flipper_tolerance){

            sensor_msgs::JointState msg;
            msg.name.push_back("flipper_front");
            msg.position.push_back(target);
            flipperCommandPub_.publish(msg);
            lastFlipperCommand = target;

        }
        }else{

        }
    }else{

    }
}

void FlipperController::handleJointStates(sensor_msgs::JointStateConstPtr msg){
    for ( int i = 0; i < msg->name.size() ; i++ ) {
        if ( msg->name[i] == "flipper_front" ) {
            current_flipper_state = msg->position[i];
            return;
        }
    }
}


void FlipperController::handleCommandAbsolute(std_msgs::Float64ConstPtr msg){
    is_last_recieved_absolute = true;
    is_last_recieved_relative = false;
    absolute_target = msg->data;
    if (fabs(absolute_target - lastFlipperCommand) >=  flipper_tolerance){
        sensor_msgs::JointState msg;
        msg.name.push_back("flipper_front");
        msg.position.push_back(absolute_target);
        flipperCommandPub_.publish(msg);
        lastFlipperCommand = absolute_target;
    }
}

void FlipperController::handleCommandRelative(std_msgs::Float64ConstPtr msg){
    is_last_recieved_absolute = false;
    is_last_recieved_relative = true;
    relative_movement = msg->data;
    time_last_relative = ros::Time::now();
}

} //Namespace
