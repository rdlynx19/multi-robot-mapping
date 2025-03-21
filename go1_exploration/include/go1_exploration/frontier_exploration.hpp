#ifndef FRONTIER_EXPLORATION_HPP_
#define FRONTIER_EXPLORATION_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
// #include "ros2_unitree_legged_msgs/msg/high_state.hpp"
// #include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
// #include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <chrono>

class FrontierExploration: public rclcpp::Node {
    public:
        FrontierExploration();
        ~FrontierExploration();

    private:
        //Timer
        rclcpp::TimerBase::SharedPtr timer;

        //Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub;  
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;  
        // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr dogCmdPub;

        //Subscriptions
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occGridSub;
        // rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr dogStateSub;
        
        //Callbacks
        void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occGridMsg);
        // void dogStateCallback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr dogStateMsg);

        //tf2
        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};

        //Helper Variables and Functions
        std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBaseLink;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> robotPose;
        std::vector<std::pair<int,int>> findLargestFrontier(const nav_msgs::msg::OccupancyGrid::SharedPtr occGridMsg, int robotRow, int robotCol, int radiusInCells);
        void publishGoalMarker(double x, double y);
        std::pair<float,float> positionToVelocity(std::pair<float,float> currentPosition, std::pair<float,float> goalPosition);
        void publishVelocityCmd(float forwardVelocity, float yawSpeed);
        float distToGoal(std::pair<float,float> currentPosition, std::pair<float,float> goalPosition);
        float quaternionToEuler(const geometry_msgs::msg::Quaternion dogOrientation);
};


#endif