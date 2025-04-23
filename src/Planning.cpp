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
        dilateMap();
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
    RCLCPP_INFO(get_logger(), "Pose: x: %f | y: %f | z: %f |", request->start.pose.position.x, request->start.pose.position.y, request->start.pose.position.z);
    RCLCPP_INFO(get_logger(), "Goal: x: %f | y: %f | z: %f |", request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z);
}

void PlanningNode::dilateMap() {
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    int radius = 10; 
    for (int y = 0; y < map_.info.height; y++) {
        for (int x = 0; x < map_.info.width; x++) {
            if (map_.data[y * map_.info.width + x] > 80) {  
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && ny >= 0 && nx < map_.info.width && ny < map_.info.height) {
                            dilatedMap.data[ny * map_.info.width + nx] = 100; 
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map dilated.");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here 
    RCLCPP_INFO(get_logger(), "A* Started");

    auto resolution = map_.info.resolution;
    auto origin = map_.info.origin;

    int width = map_.info.width;
    int height = map_.info.height;

    int sx = static_cast<int>((start.pose.position.x - origin.position.x) / resolution);
    int sy = static_cast<int>((start.pose.position.y - origin.position.y) / resolution);
    int gx = static_cast<int>((goal.pose.position.x - origin.position.x) / resolution);
    int gy = static_cast<int>((goal.pose.position.y - origin.position.y) / resolution);

    Cell cStart(sx, sy);
    Cell cGoal(gx, gy);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(height * width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    auto getIndex = [width](int x, int y) {
    	return y*width+x;
    };

    auto isObstacle = [this](int x, int y) {
    	return map_.data[(y*map_.info.width) + x] > 50;
    };

    std::vector<std::pair<int, int>> directions = {
    	{1, 0}, {-1, 0}, {0, 1}, {0, -1},
    	{1, 1}, {-1,-1}, {1,-1}, {-1, 1}
    };

    while (!openList.empty() && rclcpp::ok()) {
        auto current_it = std::min_element(openList.begin(), openList.end(), [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b){
        	return a->f < b->f;
        	});
        std::shared_ptr<Cell> current = *current_it;
        openList.erase(current_it);

        if (current->x == cGoal.x && current->y == cGoal.y) {
            path_.poses.clear();
            while (current) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = current->x*resolution+origin.position.x+resolution/2;
                    pose.pose.position.y = current->y*resolution+origin.position.y+resolution/2;
                    pose.pose.position.z = 0.0;
        		    pose.pose.orientation.w = 1.0;
                    path_.poses.push_back(pose);
                    current = current->parent;
                }
            std::reverse(path_.poses.begin(), path_.poses.end());
            path_.header.frame_id = map_.header.frame_id;
            RCLCPP_INFO(get_logger(), "Path found!");
            return;
        }
            
        closedList[getIndex(current->x, current->y)] = true;
            
        for (const auto &[dx, dy] : directions) {
            int nx = current->x + dx;
        	int ny = current->y + dy;

            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                continue;
            }

            if (closedList[getIndex(nx, ny)]) {
                continue;
            }

            if (isObstacle(nx, ny)) {
                continue;
            }

            float g_new = current->g + std::hypot(dx, dy);
        	float h_new = std::hypot(gx - nx, gy - ny);
        	float f_new = g_new + h_new;

            auto it = std::find_if(openList.begin(), openList.end(), [nx, ny](const std::shared_ptr<Cell> &cell) {
        		return cell->x == nx && cell->y == ny;
        	});

            if (it == openList.end() || f_new < (*it)->f) {
        		auto neighbor = std::make_shared<Cell>(nx, ny);
        		neighbor->g = g_new;
        		neighbor->h = h_new;
        		neighbor->f = f_new;
        		neighbor->parent = current;
        		
        		if(it == openList.end()) {
        			openList.push_back(neighbor);
        		} else {
        			*it = neighbor;
        		}
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    // add code here
    if (path_.poses.size() < 3) {
        RCLCPP_INFO(get_logger(), "No need for smoothing!");
        return; // No need to smooth if the path is too short
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    
    int window = 3;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;

        for (int j = -window; j <= window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < path_.poses.size()) {
                sum_x += path_.poses[idx].pose.position.x;
                sum_y += path_.poses[idx].pose.position.y;
                count++;
            }
        }

        newPath[i].pose.position.x = sum_x / count;
        newPath[i].pose.position.y = sum_y / count;
    }

    path_.poses = newPath;
    RCLCPP_INFO(get_logger(), "Path smoothed!");
}

Cell::Cell(int c, int r) : x(c), y(r), f(0), g(0), h(0), parent(nullptr) {
    // add code here
}