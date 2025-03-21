#include "frontier_exploration.hpp"

geometry_msgs::msg::PoseStamped transformToPose(geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose.header.frame_id = transform.header.frame_id;
    return pose;
}

FrontierExploration::FrontierExploration() : Node("frontierExploration") {
    RCLCPP_INFO(this->get_logger(), "Frontier Exploration Node Started");

    //Subscriptions
    this->occGridSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1, std::bind(&FrontierExploration::occGridCallback, this, std::placeholders::_1) );
    // this->dogStateSub = this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>("high_state", 10, std::bind(&FrontierExploration::dogStateCallback, this, std::placeholders::_1));
    
    //Publishers
    this->goalPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    this->markerPub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    // this->dogCmdPub = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);

    //tf2
    this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    //Timer
    this->timer = this->create_wall_timer(std::chrono::seconds(1), [this]()-> void {
        try{
            this->mapToBaseLink = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tfBuffer->lookupTransform("map", "oak-d-base-frame", tf2::TimePointZero));
            this->robotPose = std::make_shared<geometry_msgs::msg::PoseStamped>(transformToPose(*mapToBaseLink));
        } catch (tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "Transform Exception: %s", ex.what());
        }
    });
}

FrontierExploration::~FrontierExploration() {
    RCLCPP_INFO(this->get_logger(), "Frontier Exploration Node Stopped");
}

std::vector<std::pair<int,int>> FrontierExploration::findLargestFrontier(const nav_msgs::msg::OccupancyGrid::SharedPtr occGridMsg, int robotRow, int robotCol, int radiusInCells) {
    
    std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    //Initializes a 2D array of size height x width
    // The syntax for initialisation of vector is used
    std::vector<std::vector<bool>> visited(occGridMsg->info.height, std::vector<bool>(occGridMsg->info.width, false));

    std::vector<std::vector<std::pair<int,int>>> allFrontiers;

    for (int dx = -radiusInCells; dx <= radiusInCells; dx++) {
        for(int dy = -radiusInCells; dy <= radiusInCells; dy++) {
            int row = robotRow + dx;
            int col = robotCol + dy;

            if (row < 0 || row >= static_cast<int>(occGridMsg->info.height)) continue;
            if (col < 0 || col >= static_cast<int>(occGridMsg->info.width)) continue;

            int index = row * occGridMsg->info.width + col;
            if(occGridMsg->data[index] == -1 and !visited[row][col]){
                std::vector<std::pair<int,int>> frontier;
                std::queue<std::pair<int,int>> queue;
                queue.push({row, col});
                visited[row][col] = true;

                bool isValidFrontier = false;

                while(!queue.empty()){
                    auto[currentRow, currentCol] = queue.front();
                    queue.pop();
                    frontier.push_back({currentRow, currentCol});

                    for (auto[cx, cy]: directions){
                        int nx = currentRow + cx;
                        int ny = currentCol + cy;

                        if(nx < 0 || nx >= static_cast<int>(occGridMsg->info.height)) continue;
                        if(ny < 0 || ny >= static_cast<int> (occGridMsg->info.width)) continue;

                        int nIdx = nx * occGridMsg->info.width + ny;
                        if(occGridMsg->data[nIdx] == -1 and !visited[nx][ny]){
                            queue.push({nx, ny});
                            visited[nx][ny] = true;
                        }

                        if(occGridMsg->data[nIdx] < 50 and occGridMsg->data[nIdx] > -1){
                            isValidFrontier = true;
                        }
                    }
                }
                if (isValidFrontier) {
                    allFrontiers.push_back(frontier);
                }
            }

        }
    }
    std::vector<std::pair<int,int>> largestFrontier;
    for(const auto& currentFrontier: allFrontiers){
        if(currentFrontier.size() > largestFrontier.size()) {
            largestFrontier = currentFrontier;
        }
    }

    return largestFrontier;
}

void FrontierExploration::occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occGridMsg) {
    static rclcpp::Time last_stamp;
    if (occGridMsg->header.stamp == last_stamp) {
        RCLCPP_INFO(this->get_logger(), "Received old map data, skipping processing");
        return;
    }
    last_stamp = occGridMsg->header.stamp;

    RCLCPP_INFO(this->get_logger(), "Occupancy Grid Received");
    RCLCPP_INFO(this->get_logger(), "Map size: %d x %d", occGridMsg->info.width, occGridMsg->info.height);

    if(!this->robotPose){
        RCLCPP_WARN(this->get_logger(), "Robot Pose is not available, skipping frontier search!");
        return;
    }
    double robotX = this->robotPose->pose.position.x;
    double robotY = this->robotPose->pose.position.y;
    int robotCol = static_cast<int>((robotX - occGridMsg->info.origin.position.x) / occGridMsg->info.resolution);
    int robotRow = static_cast<int>((robotY - occGridMsg->info.origin.position.y) / occGridMsg->info.resolution);  

    double searchRadius = 5.0;
    int radiusInCells = static_cast<int>(searchRadius / occGridMsg->info.resolution); 

    std::vector<std::pair<int,int>> largestFrontier = findLargestFrontier(occGridMsg, robotRow, robotCol, radiusInCells);
    RCLCPP_INFO(this->get_logger(), "Found largest frontier of size %zu", largestFrontier.size());

    if (!largestFrontier.empty()) {
        double sumX = 0.0, sumY = 0.0;
        for (const auto& [row, col] : largestFrontier) {
            sumX += col * occGridMsg->info.resolution + occGridMsg->info.origin.position.x;
            sumY += row * occGridMsg->info.resolution + occGridMsg->info.origin.position.y;
        }
        double centroidX = sumX / largestFrontier.size();
        double centroidY = sumY / largestFrontier.size();

        geometry_msgs::msg::PoseStamped goal;
        goal.pose.position.x = centroidX;
        goal.pose.position.y = centroidY;
        goal.pose.position.z = 0;
        goal.header.frame_id = occGridMsg->header.frame_id;
        this->goalPub->publish(goal);
        this->publishGoalMarker(centroidX, centroidY);
        RCLCPP_INFO(this->get_logger(), "Published goal at (%f, %f)", goal.pose.position.x, goal.pose.position.y);
    }

}

void FrontierExploration::publishGoalMarker(double x, double y){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0);
    this->markerPub->publish(marker);
}

// void FrontierExploration::dogStateCallback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr dogStateMsg){
//     std::cout<< "Dog is currently in mode: " << dogStateMsg->mode << std::endl;
//     std::cout<< "Forward speed of the dog is: " << dogStateMsg->velocity[0] << std::endl;
//     std::cout<< "Yaw speed of the dog is: " << dogStateMsg->yaw_speed << std::endl;
// }

float quaternionToEuler(const geometry_msgs::msg::Quaternion dogOrientation){
    double x = dogOrientation.x;
    double y = dogOrientation.y;
    double z = dogOrientation.z;
    double w = dogOrientation.w;

    // Roll (x-axis rotation)
    // double sinr_cosp = 2 * (w * x + y * z);
    // double cosr_cosp = 1 - 2 * (x * x + y * y);
    // float roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));

    // Pitch (y-axis rotation)
    // double sinp = 2 * (w * y - z * x);
    // if (std::abs(sinp) >= 1) {
    //     // Use 90 degrees if out of range
    //     float pitch = static_cast<float>(std::copysign(M_PI / 2, sinp));
    // } else {
    //     float pitch = static_cast<float>(std::asin(sinp));
    // }

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    return yaw;
}

float FrontierExploration::distToGoal(std::pair<float,float> currentPosition, std::pair<float,float> goalPosition){
    float xDist = pow((currentPosition.first - goalPosition.first), 2);
    float yDist = pow((currentPosition.second - goalPosition.second), 2);
    return sqrt(xDist + yDist);
}

// void FrontierExploration::publishVelocityCmd(float forwardVelocity, float yawSpeed){
//     ros2_unitree_legged_msgs::msg::HighCmd dogCmdExplore;
//     dogCmdExplore.head[0] = 0xFE;
//     dogCmdExplore.head[1] = 0xEF;
//     dogCmdExplore.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;
//     dogCmdExplore.mode = 0;
//     dogCmdExplore.gait_type = 0;
//     dogCmdExplore.speed_level = 0;
//     dogCmdExplore.foot_raise_height = 0;
//     dogCmdExplore.body_height = 0;
//     dogCmdExplore.euler[0] = 0;
//     dogCmdExplore.euler[1] = 0;
//     dogCmdExplore.euler[2] = 0;
//     dogCmdExplore.velocity[0] = 0.05f;
//     dogCmdExplore.velocity[1] = 0.0f;
//     dogCmdExplore.yaw_speed = 0.0f;
//     dogCmdExplore.reserve = 0;
//     this->dogCmdPub->publish(dogCmdExplore);
// }

// std::pair<float,float> FrontierExploration::positionToVelocity(std::pair<float,float> currentPosition, std::pair<float,float> goalPosition){
//     float Kp = 0.1;
//     float angularDifference = atan2(goalPosition.second - currentPosition.second, goalPosition.first - currentPosition.first);
//     float dogYaw = quaternionToEuler(this->robotPose->pose.orientation);
//     float yawVelocity = 0.01 * (angularDifference - dogYaw);
//     float forwardVelocity = Kp * distToGoal(currentPosition, goalPosition);
//     return std::make_pair(forwardVelocity, yawVelocity);
// }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExploration>());
    rclcpp::shutdown();
    return 0;
}
