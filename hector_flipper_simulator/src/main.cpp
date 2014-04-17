//=================================================================================================
// Copyright (c) 2014, Christian Rose, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>

#include <std_msgs/Float64.h>

double front_goal = 0;
double rear_goal = 0;

double front_pose = 0;
double rear_pose = 0;

void frontFlipperCallback(const std_msgs::Float64::ConstPtr& msg){
    front_goal = msg->data;
    ROS_DEBUG("front msg: %f", front_goal);
}

void rearFlipperCallback(const std_msgs::Float64::ConstPtr& msg){
    rear_goal = msg->data;
}

const double STEP = 0.5;

double calcNewPose(double goal, double pose){
    double diff = goal - pose;

    if(abs(diff) < STEP){
        return goal;
    }

    if(diff < 0){
        return pose - STEP;
    }else{
        return pose + STEP;
    }


    return goal;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ros_flipper_simulator");

    ros::NodeHandle n;

    ros::Subscriber flipperFront = n.subscribe("/flipper/control/front", 1000, &frontFlipperCallback);
    ros::Subscriber flipperRear = n.subscribe("/flipper/control/rear", 1000, &rearFlipperCallback);

    ros::Publisher frontStatePublisher = n.advertise<std_msgs::Float64>("/flipper/state/front", 1000);
    ros::Publisher rearStatePublisher = n.advertise<std_msgs::Float64>("/flipper/state/rear", 1000);

    ros::Rate rate(30);

    while(ros::ok()){

        front_pose = calcNewPose(front_goal, front_pose);

        rear_pose = calcNewPose(rear_goal, rear_pose);

        std_msgs::Float64 msg;
        msg.data = front_pose;

        frontStatePublisher.publish(msg);

        msg.data = rear_pose;

        rearStatePublisher.publish(msg);

        ros::spinOnce();
        rate.sleep();

    }

  return(0);
}


