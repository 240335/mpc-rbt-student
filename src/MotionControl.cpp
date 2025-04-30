#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {   
        // Subscriber for odometry
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 1, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
        // Subscriber for laser scans
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/tiago_base/Hokuyo_URG_04LX_UG01", 1, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));
        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist", 1);
        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");
        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );
    
        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while(!plan_client_->wait_for_service(std::chrono::seconds(5)))
        {
            if(rclcpp::ok() == false)
                break;
        }
    }

void MotionControlNode::checkCollision() {
/*
    for (const auto & range : laser_scan_.ranges) {

        if (range < thresh) {
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0;
            stop.angular.z = 0.0;
            twist_publisher_->publish(stop);
        }
    }*/
}

void MotionControlNode::updateTwist() {
    // add code here
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);*/
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    goal_pose_ = goal->pose;
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with pose: (%.2f, %.2f)", goal->pose.pose.position.x, goal->pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "nav Handle accepted");
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    //std::shared_ptr<nav_msgs::srv::GetPlan::Request> request;
    request->start = current_pose_;
    request->goal = goal_pose_;
    auto future = plan_client_->async_send_request(request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle_->get_goal();
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    //auto sequence = feedback.data;

    rclcpp::Rate loop_rate(1.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {          
            //result->error_msg = "Goal canceled";
            goal_handle_->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        feedback->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback);

        
        if (rclcpp::ok()) {
            //result->result = "Goal reached";
            goal_handle_->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return;
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    RCLCPP_INFO(this->get_logger(), "pathCallback");
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    RCLCPP_INFO(this->get_logger(), "odomCallback");
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
    /*
    checkCollision();
    updateTwist();
    */
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    RCLCPP_INFO(this->get_logger(), "lidarCallback");
    laser_scan_ = msg;
}
