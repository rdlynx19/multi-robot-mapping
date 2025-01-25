#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
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

		offboard_setpoint_counter = 0;

		auto timer_callback = [this]() -> void {
			if(offboard_setpoint_counter == 10){
				//Changing to offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				
				//Arm the quadrotor
				this->arm();
			}
			publish_offboard_control_mode();
			//publish_trajectory_setpoint();
			

			//stop the count after reaching 11
			if (offboard_setpoint_counter < 11) {
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

		//common synced timestamped
		std::atomic<uint64_t> timestamp;

		//counter for the number of setpoints
		uint64_t offboard_setpoint_counter;
	
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
















