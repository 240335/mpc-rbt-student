#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("/plan_path", std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));
    
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for map service...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received.");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // add code here
    aStar(request->start, request->goal);
    smoothPath();
    response->plan = path_;
    path_pub_->publish(path_);
    RCLCPP_INFO(get_logger(), "Pose: x: %f | y: %f | z: %f |", request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z);
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here
    /*auto index = this {
        return y * map_.info.width + x;
    };
    
    auto heuristic = {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };*/
    
    Cell cStart(start.pose.position.x, start.pose.position.y);
    Cell cGoal(goal.pose.position.x, goal.pose.position.y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));
    while (!openList.empty() && rclcpp::ok()) {
        auto current = openList.front();
        
        if (current->x == cGoal.x && current->y == cGoal.y) {
            path_.poses.clear();
            while (current) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = current->x;
                    pose.pose.position.y = current->y;
                    path_.poses.push_back(pose);
                    current = current->parent;
                }
            std::reverse(path_.poses.begin(), path_.poses.end());
            RCLCPP_INFO(get_logger(), "Path found!");
            return;
        }
            
        //closedList.insert(index(current->x, current->y));
            
        std::vector<std::pair<int, int>> neighbors = {
            {current->x + 1, current->y}, {current->x - 1, current->y},
            {current->x, current->y + 1}, {current->x, current->y - 1}
        };
            
        for (const auto &neighbor : neighbors) {
            
            int nx = neighbor.first;
            int ny = neighbor.second;

            if (nx < 0 || ny < 0 || nx >= map_.info.width || ny >= map_.info.height) {
                continue;
            }

            /*if (map_.data[index(nx, ny)] != 0) {
                continue;
            }

            if (closedList.find(index(nx, ny)) != closedList.end()) {
                continue;
            }*/

            auto neighborCell = std::make_shared<Cell>(nx, ny);
            neighborCell->g = current->g + 1;
            //neighborCell->h = heuristic(*neighborCell, cGoal);
            neighborCell->f = neighborCell->g + neighborCell->h;
            neighborCell->parent = current;

            openList.push_back(neighborCell);
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
}
