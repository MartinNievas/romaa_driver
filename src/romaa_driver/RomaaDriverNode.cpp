#include "rclcpp/rclcpp.hpp"
#include "romaa_driver/RomaaDriverNode.hpp"

namespace romaa_driver
{

    using std::placeholders::_1;
    using std::placeholders::_2;
    using namespace std::chrono_literals;

    RomaaDriverNode::RomaaDriverNode() : Node("romaa_driver")
    {
        // Declare node parameters
        declare_parameter<double>("frequency", 10.0);
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_link");

        // Reading parameters
        frequency = get_parameter("frequency").as_double();
        port = get_parameter("port").as_string();
        baudrate = get_parameter("baudrate").as_int();
        odom_frame = get_parameter("odom_frame").as_string();
        base_frame = get_parameter("base_frame").as_string();

        // Show node parameter
        RCLCPP_INFO(get_logger(), "Node parameters:");
        RCLCPP_INFO(get_logger(), " - frequency: %.1f", frequency);
        RCLCPP_INFO(get_logger(), " - port: %s", port.c_str());
        RCLCPP_INFO(get_logger(), " - baudrate: %d", baudrate);
        RCLCPP_INFO(get_logger(), " - odom_frame: %s", odom_frame.c_str());
        RCLCPP_INFO(get_logger(), " - base_frame: %s", base_frame.c_str());

        // Create communication object
        RCLCPP_INFO(get_logger(), "Opening RoMAA communication port in %s at %d...",
                    port.c_str(), baudrate);
        comm = new romaa_comm(port.c_str(), baudrate);

        // Create publisher
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Create subcriber
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SensorDataQoS(),
            std::bind(&RomaaDriverNode::cmdVelCb, this, _1));

        // Create service servers
        en_motor_srv = create_service<std_srvs::srv::SetBool>("enable_motor",
                                                              std::bind(&RomaaDriverNode::enableMotorCb, this, _1, _2));

        // Set odometr message constant fields
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;
        odom_msg.header.stamp = now(); // Initial time for dt

        // TF2
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        odom_tf.header.frame_id = odom_frame;
        odom_tf.child_frame_id = base_frame;

        // Timer
        pub_timer = create_wall_timer(std::chrono::duration<double>(1.0 / frequency),
                                      std::bind(&RomaaDriverNode::pubOdometryCb, this)); // ToDo: use frequency
    }

    RomaaDriverNode::~RomaaDriverNode()
    {
        RCLCPP_INFO(get_logger(), "Closing RoMAA communication...");
        comm->set_speed(0.0, 0.0);

        // auto t1 = std::chrono::high_resolution_clock::now();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        // RCLCPP_INFO(get_logger(), "dt: %ld", ms_int.count());

        comm->disable_motor();
        delete comm;
    }

    void
    RomaaDriverNode::cmdVelCb(geometry_msgs::msg::Twist::UniquePtr msg)
    {
        RCLCPP_INFO(get_logger(), "v: %.2f, w: %.2f", msg->linear.x, msg->angular.z);
        comm->set_speed(msg->linear.x, msg->angular.x);
    }

    void
    RomaaDriverNode::pubOdometryCb()
    {
        // RCLCPP_INFO(get_logger(), "pubOdometryCb");

        // Time
        auto current_time = now();

        // Read odometry
        if (comm->get_odometry(x, y, yaw) == -1)
            RCLCPP_DEBUG(get_logger(), "Unable to read odometry!");
        if (comm->get_speed(v, w) == -1)
            RCLCPP_DEBUG(get_logger(), "Unable to read speed!");

        // Odometry message
        odom_msg.header.stamp = current_time;
        // Pose for circular path
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        // Yaw angle to quaternion
        quat_tf.setRPY(0, 0, yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(quat_tf);

        // Odometry twist (dummy speed)
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;

        // TF
        odom_tf.header.stamp = current_time;
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

        // publish odometry message
        odom_pub->publish(odom_msg);

        // Broadcast TF
        tf_broadcaster->sendTransform(odom_tf);
    }

    void
    RomaaDriverNode::enableMotorCb(std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr res)
    {
        if (req->data == true)
        {
            RCLCPP_INFO(get_logger(), "Enable motor");
            comm->enable_motor();
            res->success = true;
            res->message = "Motor enabled";
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Disable  motor");
            comm->disable_motor();
            res->success = true;
            res->message = "Motor disabled";
        }
    }

} // namespace romaa_driver
