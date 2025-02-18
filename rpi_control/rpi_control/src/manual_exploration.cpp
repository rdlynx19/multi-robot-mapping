#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include "rpi_interfaces/srv/arming_control.hpp"
#include "rpi_interfaces/srv/takeoff.hpp"
#include "rpi_interfaces/srv/land.hpp"
#include "rpi_interfaces/srv/trajectory.hpp"

#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum class DroneState {
	DISARMED,
	ARMED,
	TRAJECTORY,
	TAKEOFF,
	LAND
};

class ManualExploration : public rclcpp::Node
{
	public:
		ManualExploration(): Node("manual_setpoints")
	{
		mocap_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    	px4_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		home_pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
		home_pose_flag = true;
		//Publishers
		offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		visual_odometry_publisher = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

		//setting compatible qos profile for subscribers
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		//Subscribers
		optitrack_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/quad_pose", qos, std::bind(&ManualExploration::quad_pose_callback, this, std::placeholders::_1));
		vehicle_status_subscriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&ManualExploration::vehicle_status_callback, this, std::placeholders::_1));
		vehicle_pose_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
		std::bind(&ManualExploration::vehicle_pose_callback, this, std::placeholders::_1));

		//Services
		arming_control_service = this->create_service<rpi_interfaces::srv::ArmingControl>("arming_control", std::bind(&ManualExploration::arming_service_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS());

		takeoff_control_service = this->create_service<rpi_interfaces::srv::Takeoff>("takeoff", std::bind(&ManualExploration::takeoff_service_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS());

		landing_control_service = this->create_service<rpi_interfaces::srv::Land>("landing", std::bind(&ManualExploration::landing_service_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS());

		trajectory_control_service = this->create_service<rpi_interfaces::srv::Trajectory>("trajectory", std::bind(&ManualExploration::trajectory_service_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS());

		offboard_setpoint_counter = 0;

		//Generating Square Trajectory
		//std::pair<double,double> polygonCenter = {0.0, 0.0};
		//int n_sides = 4;
		//double side_length = 0.8;
		//double circum_radius = side_length / std::sqrt(2);
		//int points_per_line = 30;
		//double polygon_angle = 45.0;

		//square_trajectory = polygonPath(polygonCenter, n_sides, circum_radius, points_per_line, polygon_angle);
		//current_trajectory_index = 0;
		//position_tolerance = 0.3;
		//reached_setpoint = false;

		auto timer_callback = [this]() -> void {
			if(offboard_setpoint_counter == 20){
				//Changing to offboard mode after 20 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				
				//Arm the quadrotor
//				 this->arm(); //added service to do this
			}
			publish_offboard_control_mode();
			publish_trajectory_setpoint(current_state);
			

			//stop the count after reaching 11
			if (offboard_setpoint_counter < 21) {
				offboard_setpoint_counter++;
			}	
		};
		timer = this->create_wall_timer(100ms, timer_callback);

	}
		void arm();
		void disarm();
	
	private:
		rclcpp::TimerBase::SharedPtr timer;

		//Publishers
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
		rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_publisher;

		//Subscribers
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr optitrack_pose_subscriber;
		rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber;
		rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_pose_subscriber;

		//Services
		rclcpp::Service<rpi_interfaces::srv::ArmingControl>::SharedPtr arming_control_service;
		rclcpp::Service<rpi_interfaces::srv::Takeoff>::SharedPtr takeoff_control_service;
		rclcpp::Service<rpi_interfaces::srv::Land>::SharedPtr landing_control_service;
		rclcpp::Service<rpi_interfaces::srv::Trajectory>::SharedPtr trajectory_control_service;

		//Transform Broadcasters
		std::unique_ptr<tf2_ros::TransformBroadcaster> mocap_broadcaster;
		std::unique_ptr<tf2_ros::TransformBroadcaster> px4_broadcaster;

		//Helper Variables
		geometry_msgs::msg::PoseStamped::UniquePtr home_pose_msg;
		bool home_pose_flag = true;
		DroneState current_state = DroneState::DISARMED;
		std::vector<std::tuple<double, double, double>> square_trajectory;
		uint64_t current_trajectory_index;
		float position_tolerance;
		bool reached_setpoint;
		px4_msgs::msg::VehicleOdometry::SharedPtr current_odometry_pose;

		//common synced timestamped
		std::atomic<uint64_t> timestamp;

		//counter for the number of setpoints
		uint64_t offboard_setpoint_counter;

		void quad_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
		void vehicle_status_callback(const VehicleStatus::SharedPtr status_msg) const;
		void vehicle_pose_callback(const VehicleOdometry::SharedPtr pose_msg);
		void arming_service_callback(const std::shared_ptr<rpi_interfaces::srv::ArmingControl::Request> arm_disarm_request, std::shared_ptr<rpi_interfaces::srv::ArmingControl::Response> arm_disarm_response);
		void takeoff_service_callback(const std::shared_ptr<rpi_interfaces::srv::Takeoff::Request> takeoff_request, std::shared_ptr<rpi_interfaces::srv::Takeoff::Response> takeoff_response);
		void landing_service_callback(const std::shared_ptr<rpi_interfaces::srv::Land::Request> landing_request, std::shared_ptr<rpi_interfaces::srv::Land::Response> landing_response);
		void trajectory_service_callback(const std::shared_ptr<rpi_interfaces::srv::Trajectory::Request> trajectory_request, std::shared_ptr<rpi_interfaces::srv::Trajectory::Response> trajectory_response);


		void publish_offboard_control_mode();
		void publish_trajectory_setpoint(DroneState state = DroneState::DISARMED, float x_position = 0.0, float y_position = 0.0, float z_position = -0.4, float yaw = 0.0);
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
		void transitionTo(DroneState new_state);
		std::vector<std::tuple< double, double, double>> polygonPath(const std::pair<double, double>& polygon_center, int n_sides, double circum_radius, int points_per_line, double polygon_angle);	
};

void ManualExploration::transitionTo(DroneState new_state)
{
    RCLCPP_INFO(this->get_logger(), "Transitioning to new state: %d", static_cast<int>(new_state));
    current_state = new_state;
}

std::vector<std::tuple<double, double, double>> ManualExploration::polygonPath(
    const std::pair<double, double>& polygon_center, 
    int n_sides, 
    double circum_radius, 
    int points_per_line, 
    double polygon_angle)
{
    double angle_rad = polygon_angle * M_PI / 180.0;

    double theta_increment = 2 * M_PI / n_sides;

    // Generate vertices of the polygon
    std::vector<std::pair<double, double>> vertices;
    for (int i = 0; i <= n_sides; ++i) {
        double theta = i * theta_increment + angle_rad;
        double x = circum_radius * std::cos(theta) + polygon_center.first;
        double y = circum_radius * std::sin(theta) + polygon_center.second;
        vertices.emplace_back(x, y);
    }

    // Generate path coordinates with yaw
    std::vector<std::tuple<double, double, double>> path_coordinates;
    for (int i = 0; i < n_sides; ++i) {
        double x_start = vertices[i].first;
        double y_start = vertices[i].second;
        double x_end = vertices[i + 1].first;
        double y_end = vertices[i + 1].second;

        for (int j = 0; j < points_per_line; ++j) {
            double t = static_cast<double>(j) / (points_per_line + 1);
            double x_interp = x_start + t * (x_end - x_start);
            double y_interp = y_start + t * (y_end - y_start);

            // Calculate yaw (starts at 0 and completes a full rotation)
            double yaw = (2 * M_PI * (i * points_per_line + j)) / (n_sides * points_per_line);

            // Normalize yaw to the range [-π, π]
            if (yaw > M_PI) {
                yaw -= 2 * M_PI;
            }

            path_coordinates.emplace_back(x_interp, y_interp, yaw);
        }
    }

    return path_coordinates;
}

void ManualExploration::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm Command Sent");
	transitionTo(DroneState::ARMED);
}

void ManualExploration::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	
	RCLCPP_INFO(this->get_logger(), "Disarm Command Sent");
	transitionTo(DroneState::DISARMED);	
}

void ManualExploration::quad_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) 
{
	RCLCPP_INFO_ONCE(this->get_logger(), "Received first message from OptiTrack!");
	RCLCPP_INFO_ONCE(this->get_logger(), "Pos: %f, %f, %f", pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
	RCLCPP_INFO_ONCE(this->get_logger(), "Quat: %f, %f, %f, %f", pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
	if(ManualExploration::home_pose_flag){
		ManualExploration::home_pose_msg->pose.position.x = pose_msg->pose.position.x;
		ManualExploration::home_pose_msg->pose.position.y = pose_msg->pose.position.y;
		ManualExploration::home_pose_msg->pose.position.z = pose_msg->pose.position.z;
		ManualExploration::home_pose_msg->pose.orientation.x = pose_msg->pose.orientation.x;
		ManualExploration::home_pose_msg->pose.orientation.y = pose_msg->pose.orientation.y;
		ManualExploration::home_pose_msg->pose.orientation.z = pose_msg->pose.orientation.z;
		ManualExploration::home_pose_msg->pose.orientation.w = pose_msg->pose.orientation.w;
		ManualExploration::home_pose_flag = false;

		std::pair<double,double> polygonCenter = {home_pose_msg->pose.position.x, home_pose_msg->pose.position.y};
		int n_sides = 4;
		double side_length = 0.8;
		double circum_radius = side_length / std::sqrt(2);
		int points_per_line = 30;
		double polygon_angle = 45.0;

		square_trajectory = polygonPath(polygonCenter, n_sides, circum_radius, points_per_line, polygon_angle);
		current_trajectory_index = 0;
		position_tolerance = 0.2;
		reached_setpoint = false;

	}

	VehicleOdometry msg{};
	msg.pose_frame = msg.POSE_FRAME_FRD;
	msg.timestamp = uint64_t(pose_msg->header.stamp.sec)*1000000 + uint64_t(pose_msg->header.stamp.nanosec)/1000;
	msg.timestamp_sample = msg.timestamp;

	msg.position[0] = pose_msg->pose.position.x;
	msg.position[1] = pose_msg->pose.position.y;
	msg.position[2] = pose_msg->pose.position.z;

	msg.q[0] = pose_msg->pose.orientation.w;
	msg.q[1] = pose_msg->pose.orientation.x;
	msg.q[2] = pose_msg->pose.orientation.y;
	msg.q[3] = pose_msg->pose.orientation.z;

	visual_odometry_publisher->publish(msg);
	RCLCPP_INFO_ONCE(this->get_logger(), "Message sent to PX4");

	geometry_msgs::msg::TransformStamped mocap_tf;

	mocap_tf.header.stamp = this->get_clock()->now();
	mocap_tf.header.frame_id = "world";
	mocap_tf.child_frame_id = "mocap_pose";

	mocap_tf.transform.translation.x = msg.position[0];
	mocap_tf.transform.translation.y = msg.position[1];
	mocap_tf.transform.translation.z = msg.position[2];

	mocap_tf.transform.rotation.x = msg.q[0];
	mocap_tf.transform.rotation.y = msg.q[1];
	mocap_tf.transform.rotation.z = msg.q[2];
	mocap_tf.transform.rotation.w = msg.q[3];

	mocap_broadcaster->sendTransform(mocap_tf);

//	RCLCPP_INFO(this->get_logger(), "The mocap z coordinate is: %f", msg.position[2]);
}

void ManualExploration::vehicle_pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr pose_msg)
{
	geometry_msgs::msg::TransformStamped px4_tf;

	px4_tf.header.stamp = this->get_clock()->now();
	px4_tf.header.frame_id = "world";
	px4_tf.child_frame_id = "px4_pose";

	px4_tf.transform.translation.x = pose_msg->position[0];
	px4_tf.transform.translation.y = pose_msg->position[1];
	px4_tf.transform.translation.z = pose_msg->position[2];

	px4_tf.transform.rotation.x = pose_msg->q[0];
	px4_tf.transform.rotation.y = pose_msg->q[1];
	px4_tf.transform.rotation.z = pose_msg->q[2];
	px4_tf.transform.rotation.w = pose_msg->q[3];

	px4_broadcaster->sendTransform(px4_tf);

	current_odometry_pose = pose_msg;

	// RCLCPP_INFO(this->get_logger(), "The PX4 x, y, z coordinate is!!!: %f, %f, %f",pose_msg->position[0], pose_msg->position[1], pose_msg->position[2]);
}

void ManualExploration::vehicle_status_callback(const VehicleStatus::SharedPtr status_msg) const 
{
	if(static_cast<int>(status_msg->arming_state) == 1){
		RCLCPP_INFO_ONCE(this->get_logger(), "Disarmed");
	}
	if(static_cast<int>(status_msg->arming_state) == 2){
		RCLCPP_INFO_ONCE(this->get_logger(), "Armed");
	}
	if(status_msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD ){
		RCLCPP_INFO_ONCE(this->get_logger(), "Switched to Offboard Mode");
	}
}

void ManualExploration::arming_service_callback(const std::shared_ptr<rpi_interfaces::srv::ArmingControl::Request> arm_disarm_request, std::shared_ptr<rpi_interfaces::srv::ArmingControl::Response> arm_disarm_response)
{
	if (arm_disarm_request->flight_request == "arm")
	{
		arm();
		arm_disarm_response->response = "UAV Armed!";
	}
	else if(arm_disarm_request->flight_request == "disarm")
	{
		disarm();
		arm_disarm_response->response = "UAV Disarmed!";
	}
	else 
	{
		arm_disarm_response->response = "Invalid request! Check the request again!";
	}
}

void ManualExploration::takeoff_service_callback(const std::shared_ptr<rpi_interfaces::srv::Takeoff::Request> takeoff_request, std::shared_ptr<rpi_interfaces::srv::Takeoff::Response> takeoff_response)
{
	if(takeoff_request->trigger_takeoff == true){
		transitionTo(DroneState::TAKEOFF);
		

		takeoff_response->response = true;
	}
	
}

void ManualExploration::landing_service_callback(const std::shared_ptr<rpi_interfaces::srv::Land::Request> landing_request, std::shared_ptr<rpi_interfaces::srv::Land::Response> landing_response)
{
	if(landing_request->trigger_landing == true){
		transitionTo(DroneState::LAND);

		landing_response->response = true;
	}	
}

void ManualExploration::trajectory_service_callback(const std::shared_ptr<rpi_interfaces::srv::Trajectory::Request> trajectory_request, std::shared_ptr<rpi_interfaces::srv::Trajectory::Response> trajectory_response)
{
	if(trajectory_request->trigger_trajectory == true){
		transitionTo(DroneState::TRAJECTORY);

		trajectory_response->response = true;
	}
}

void ManualExploration::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
	offboard_control_mode_publisher->publish(msg);
}

void ManualExploration::publish_trajectory_setpoint(DroneState state, float x_position, float y_position, float z_position, float yaw)
{
	//Values are interpreted in NED Frame of Reference
	TrajectorySetpoint msg{};

	switch (state)
	{
		case DroneState::TAKEOFF:
		{
			RCLCPP_INFO_ONCE(this->get_logger(), "Sending Takeoff Setpoint");
			msg.position = {home_pose_msg->pose.position.x, home_pose_msg->pose.position.y, z_position};
			msg.yaw = yaw; // (-pi, pi)
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher->publish(msg);
			RCLCPP_INFO_ONCE(this->get_logger(), "Takeoff setpoints %f, %f", home_pose_msg->pose.position.x, home_pose_msg->pose.position.y);	
			break;
		}
		case DroneState::TRAJECTORY:
		{
			// RCLCPP_INFO(this->get_logger(), "Sending Trajectory Setpoint");
			if(square_trajectory.empty()){
				RCLCPP_ERROR(this->get_logger(), "Trajectory is empty. Generate the trajectory first!");
				break;
			}
			auto point = square_trajectory[current_trajectory_index];

			if(current_trajectory_index < square_trajectory.size()){
				
				float current_x = current_odometry_pose->position[0];
				float current_y = current_odometry_pose->position[1];
				float current_z = current_odometry_pose->position[2];

				float dx = static_cast<float>(std::get<0>(point)) - current_x;
				float dy = static_cast<float>(std::get<1>(point)) - current_y;
				float dz = z_position - current_z;

				float distance = static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));

				if(distance <= position_tolerance){
					RCLCPP_INFO(this->get_logger(), "Reached setpoint %f, %f, %f", static_cast<float>(std::get<0>(point)), static_cast<float>(std::get<1>(point)), static_cast<float>(std::get<2>(point)));

					current_trajectory_index++;
					reached_setpoint = true;
				}
				else 
				{
					msg.position = {static_cast<float>(std::get<0>(point)), static_cast<float>(std::get<1>(point)), z_position};
					msg.yaw = std::get<2>(point); //check this is important! Are the setpoints, in body frame or world fixed frame!
					msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
					trajectory_setpoint_publisher->publish(msg);
					reached_setpoint = false;
				}

			}
			else{
				current_trajectory_index = 0;
			}
			break;
		}
		case DroneState::LAND:
		{
			RCLCPP_INFO_ONCE(this->get_logger(), "Sending Landing Setpoint");
			 msg.position = {static_cast<float>(ManualExploration::home_pose_msg->pose.position.x), static_cast<float>(ManualExploration::home_pose_msg->pose.position.y), static_cast<float>(ManualExploration::home_pose_msg->pose.position.z)};
//			msg.position = {0.0, 0.0, 0.0};
			msg.yaw = yaw;
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher->publish(msg);
			break;
		}
		default:
			// RCLCPP_INFO(this->get_logger(), "Default case");
			break;
	}

}

void ManualExploration::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher->publish(msg);
}

int main(int argc, char* argv[])
{
	std::cout<< "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ManualExploration>());

	rclcpp::shutdown();
	return 0;
}

