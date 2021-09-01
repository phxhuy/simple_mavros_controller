/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

mavros_msgs::State current_state;
ros::Subscriber state_sub, trajectory_subscriber;
ros::Publisher local_pos_pub;

Eigen::Vector4d position_d;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void desiredPoseCallback(geometry_msgs::PoseStamped position_trajectory_msg){
    position_d  <<  position_trajectory_msg.pose.position.x,
                    position_trajectory_msg.pose.position.y, 
                    position_trajectory_msg.pose.position.z, 
                    0.28;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // test trajectory

    trajectory_subscriber = nh.subscribe< geometry_msgs::PoseStamped>
    ("/desired_mavros_position_command", 10, desiredPoseCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // std::cout << "start flying now to point" << std::endl;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position_d(0);
        pose.pose.position.y = position_d(1);
        pose.pose.position.z = position_d(2);
        pose.pose.orientation.z = position_d(3);

        local_pos_pub.publish(pose);

        std::cout << "start flying now to point ("
        << position_d(0) << ", "
        << position_d(1) << ", "
        << position_d(2) << ", "
        << position_d(3) << ") "
        <<  std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}