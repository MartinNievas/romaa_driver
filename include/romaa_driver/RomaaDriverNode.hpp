#ifndef ROMAA_DRIVER__ROMAA_DRIVER_NODE_HPP_
#define ROMAA_DRIVER__ROMAA_DRIVER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

// Messages
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Services
#include "std_srvs/srv/set_bool.hpp"

// TF2 library
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// RoMAA communication class header
#include "romaa_comm/romaa_comm.h"

namespace romaa_driver 
{

class RomaaDriverNode : public rclcpp::Node
{
    public:
        RomaaDriverNode();
        ~RomaaDriverNode();
        bool is_connected() { return comm->is_connected(); }

    private:
        // Node parameters
        double frequency;
        std::string port;                   // Serial port name
        int baudrate;                       // Serial port baudrate
        std::string odom_frame, base_frame; // TF frames

        // Node variables
        romaa_comm *comm;
        float x, y, yaw, v, w;
        tf2::Quaternion quat_tf;

        // TF2 variables
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::TransformStamped odom_tf;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        // Subcribers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        void cmdVelCb(geometry_msgs::msg::Twist::UniquePtr );

        // Service servers
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr en_motor_srv;
        void enableMotorCb(std_srvs::srv::SetBool::Request::SharedPtr ,
            std_srvs::srv::SetBool::Response::SharedPtr );

        // Messages
        nav_msgs::msg::Odometry odom_msg;

        // Timer
        rclcpp::TimerBase::SharedPtr pub_timer;
        void pubOdometryCb();
};

} // namespace romaa_driver

#endif // ROMAA_DRIVER__ROMAA_DRIVER_NODE_HPP_
