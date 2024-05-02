#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "robotnik_msgs/set_digital_output.h"
#include <actionlib/client/simple_action_client.h>
#include <rising_manipulation_msgs/MoveToAction.h>
#include <rosmon_msgs/StartStop.h>

class Node {
public:
    Node(ros::NodeHandle& nh) : nh_(nh) {
        // Subscribe to the bool topic
        toggle_kinova_sub_ = nh.subscribe("toggle_kinova", 1, &Node::toggleKinovaCallback, this);

        // Wait for the trigger service to become available
        ros::service::waitForService("/robot/robotnik_base_hw/set_digital_output");
        ros::service::waitForService("/rosmon_arm_bringup/start_stop");
        ros::service::waitForService("/rosmon_moveit/start_stop");
        ros::service::waitForService("/rosmon_manipulation_app/start_stop");

        // Create a client for the trigger service
        trigger_client_ = nh.serviceClient<robotnik_msgs::set_digital_output>("/robot/robotnik_base_hw/set_digital_output");
        rosmon_client_ = nh.serviceClient<rosmon_msgs::StartStop>("/rosmon_arm_bringup/start_stop");
        moveit_rosmon_client_ = nh.serviceClient<rosmon_msgs::StartStop>("/rosmon_moveit/start_stop");
        manipulation_rosmon_client_ = nh.serviceClient<rosmon_msgs::StartStop>("/rosmon_manipulation_app/start_stop");

        robotnik_msgs::set_digital_output srv;
        srv.request.output = 10;
        srv.request.value = false;
        
        // Call the service
        if (trigger_client_.call(srv)) {
            ROS_INFO("Trigger sent");
        } else {
            ROS_ERROR("Failed to call trigger service");
        }

        // Reset moveit move to action client
        move_to_action_client_.reset(
            new actionlib::SimpleActionClient<rising_manipulation_msgs::MoveToAction>("/my_gen3/rising_manipulation_app/move_to"));


        move_to_goal_ = rising_manipulation_msgs::MoveToGoal();

        move_to_action_client_->cancelGoal ();
    }

    void toggleKinovaCallback(const std_msgs::Bool::ConstPtr& msg) {
        // When receiving a message on the bool topic, store the value and publish the trigger
        if(msg->data)
        {
            publishTrigger();
        }

        if(!msg->data)
        {
            move_to_goal_.to = "folded";
            move_to_action_client_->sendGoal(move_to_goal_);
            move_to_action_client_->waitForResult(ros::Duration(30.0));

            if(move_to_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                publishTrigger();
            }

        }
        
    }

    void publishTrigger() {
        // Create a request message for the trigger service
        robotnik_msgs::set_digital_output srv;
        srv.request.output = 10;
        srv.request.value = true;
        
        // Call the service
        if (trigger_client_.call(srv)) {
            ROS_INFO("Trigger sent");
        } else {
            ROS_ERROR("Failed to call trigger service");
        }

        // Sleep for 3 seconds
        ros::Duration(7.0).sleep();

        // Check if ros::shutdown has been requested
        if (ros::isShuttingDown()) {
            ROS_INFO("Shutting down...");
            return;
        }

        srv.request.output = 10;
        srv.request.value = false;
        
        // Call the service
        if (trigger_client_.call(srv)) {
            ROS_INFO("Trigger sent");
        } else {
            ROS_ERROR("Failed to call trigger service");
        }

        rosmon_msgs::StartStop srv_rosmon;
        srv_rosmon.request.node = "my_gen3_driver";
        srv_rosmon.request.ns = "/my_gen3";
        srv_rosmon.request.action = 3;
        if (rosmon_client_.call(srv_rosmon)) {
            ROS_INFO("Arm driver restarted");
        } else {
            ROS_ERROR("Failed to restart arm driver");
        }


        rosmon_msgs::StartStop moveit_srv_rosmon;
        moveit_srv_rosmon.request.node = "move_group";
        moveit_srv_rosmon.request.ns = "/my_gen3";
        moveit_srv_rosmon.request.action = 3;
        if (moveit_rosmon_client_.call(moveit_srv_rosmon)) {
            ROS_INFO("Moveit restarted");
        } else {
            ROS_ERROR("Failed to restart Moveit");
        }

        rosmon_msgs::StartStop manipulation_srv_rosmon;
        manipulation_srv_rosmon.request.node = "rising_manipulation_app";
        manipulation_srv_rosmon.request.ns = "/my_gen3";
        manipulation_srv_rosmon.request.action = 3;
        if (manipulation_rosmon_client_.call(manipulation_srv_rosmon)) {
            ROS_INFO("Manipulation app restarted");
        } else {
            ROS_ERROR("Failed to restart Manipulation app");
        }

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber toggle_kinova_sub_;
    ros::ServiceClient trigger_client_;
    ros::ServiceClient rosmon_client_;
    ros::ServiceClient moveit_rosmon_client_;
    ros::ServiceClient manipulation_rosmon_client_;
    bool bool_value_ = false;
    // MoveIt Action client
    std::shared_ptr<actionlib::SimpleActionClient<rising_manipulation_msgs::MoveToAction>> move_to_action_client_;
    rising_manipulation_msgs::MoveToGoal move_to_goal_;
    bool active_moveit_goal_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "toggle_kinova_node");
    ros::NodeHandle nh;

    Node node(nh);

    ros::spin();

    // If ros::spin() returns, we reach here
    ROS_INFO("Node shutting down");
    ros::shutdown();

    return 0;
}