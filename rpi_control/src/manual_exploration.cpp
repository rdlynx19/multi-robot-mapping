#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ManualExploration : public rclcpp::Node
{
	public:
		ManualExploration(): Node("manual_setpoints")
	{
		offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		visual_odometry_publisher = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

		//setting compatible qos profile for subscribers
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		optitrack_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/quad_pose", qos, std::bind(&ManualExploration::quad_pose_callback, this, std::placeholders::_1));
		vehicle_status_subscriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&ManualExploration::vehicle_status_callback, this, std::placeholders::_1));

		offboard_setpoint_counter = 0;

		auto timer_callback = [this]() -> void {
			if(offboard_setpoint_counter == 20){
				//Changing to offboard mode after 20 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				
				//Arm the quadrotor
				this->arm();
			}
			publish_offboard_control_mode();
			//publish_trajectory_setpoint();
			

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

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
		rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_publisher;

		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr optitrack_pose_subscriber;
		rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber;

		//common synced timestamped
		std::atomic<uint64_t> timestamp;

		//counter for the number of setpoints
		uint64_t offboard_setpoint_counter;

		void quad_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) const;
		void vehicle_status_callback(const VehicleStatus::SharedPtr status_msg) const;

		void publish_offboard_control_mode();
		void publish_trajectory_setpoint();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);	
};

void ManualExploration::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ManualExploration::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	
	RCLCPP_INFO(this->get_logger(), "Disarm command send");	
}

void ManualExploration::quad_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) const
{
	RCLCPP_INFO_ONCE(this->get_logger(), "Received first message from OptiTrack!");
	RCLCPP_INFO_ONCE(this->get_logger(), "Pos: %f, %f, %f", pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
	RCLCPP_INFO_ONCE(this->get_logger(), "Quat: %f, %f, %f, %f", pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.w, pose_msg->pose.orientation.w);
	
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
}

void ManualExploration::vehicle_status_callback(const VehicleStatus::SharedPtr status_msg) const 
{
	std::cout << "NAV_STATUS: " << static_cast<int>(status_msg->nav_state) << std::endl; 								// currently active mode
	std::cout << "OFFB_STATUS: " << static_cast<int> (VehicleStatus::NAVIGATION_STATE_OFFBOARD) << std::endl; 			
	std::cout << "ARM_STATUS: " << static_cast<int>(status_msg->arming_state) << std::endl; 							// Disarmed = 1, Armed = 2
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

void ManualExploration::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // (-pi, pi)
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher->publish(msg);
}

void ManualExploration::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
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

