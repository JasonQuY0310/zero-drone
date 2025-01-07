#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

ros::ServiceClient arming_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient set_mode_client;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

bool set_mode(const std::string& mode)
{
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;

    if (set_mode_client.call(srv) && srv.response.mode_sent)
    {
        ROS_INFO("Mode set to %s", mode.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to set mode %s", mode.c_str());
        return false;
    }
}

bool takeoff(double altitude)
{
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = altitude;

    if (takeoff_client.call(srv) && srv.response.success)
    {
        ROS_INFO("Takeoff successful to %.2f meters.", altitude);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call takeoff service.");
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_takeoff_cpp_node");
    ros::NodeHandle nh;

    // Initialize clients
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("Waiting for FCU connection...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // Arm the vehicle
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed.");
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle.");
        return -1;
    }

    // Set mode to GUIDED
    if (!set_mode("GUIDED"))
    {
        ROS_ERROR("Failed to set mode to GUIDED.");
        return -1;
    }

    // Wait for mode change
    ros::Duration(2.0).sleep();

    // Takeoff to 2 meters
    if (!takeoff(2.0))
    {
        return -1;
    }

    // Wait for stability
    ros::Duration(10.0).sleep();

    // Switch to HOLD mode
    if (!set_mode("HOLD"))
    {
        ROS_ERROR("Failed to set mode to HOLD.");
        return -1;
    }

    ros::spin();
    return 0;
}
