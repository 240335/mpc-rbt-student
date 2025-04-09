#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"


LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Print incoming data for verification
    //RCLCPP_INFO(this->get_logger(), "Received JointState message:");
    //RCLCPP_INFO(this->get_logger(), "Position: [%f, %f]", msg.position[0], msg.position[1]);
    //RCLCPP_INFO(this->get_logger(), "Velocity: [%f, %f]", msg.velocity[0], msg.velocity[1]);

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    // ********
    // * Help *
    // ********
    
    double left_wheel_vel_lin = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double right_wheel_vel_lin = right_wheel_vel * robot_config::WHEEL_RADIUS;

    double linear = (left_wheel_vel_lin + right_wheel_vel_lin) / 2.0;
    double angular = (right_wheel_vel_lin - left_wheel_vel_lin) / (2.0*robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    odometry_.child_frame_id = "base_link";
    odometry_.header.frame_id = "map";

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    yaw += angular * dt; 
    // Update position
    odometry_.pose.pose.position.x += linear * dt * std::cos(yaw);
    odometry_.pose.pose.position.y += linear * dt * std::sin(yaw);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    //q.normalize();
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    //odometry_.twist.twist.linear.x = linear;
    //odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = odometry_.pose.pose.position.z;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
