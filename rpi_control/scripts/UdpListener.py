#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import struct
import tf2_ros
import tf_transformations  # For quaternion and transformation calculations
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class UdpListener(Node):
    def __init__(self):
        super().__init__('udp_listener')

        # Declare a parameter for the bird's name
        self.declare_parameter('bird_name', 'charlie_3')


        # Create a publisher for the PoseStamped messages
        self.pose_publisher = self.create_publisher(PoseStamped, 'quad_pose', 10)
        
        # Create a broadcaster for the TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set up the UDP server
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)  # Reduce buffer size
        self.udp_socket.setblocking(False)
        self.udp_socket.bind(("0.0.0.0", 54321))

        # Set a timer to check for new messages and broadcast at 100Hz
        self.create_timer(1/100.0, self.timer_callback)


    

    def timer_callback(self):
        try:
            # Receive data from UDP socket
            data, addr = self.udp_socket.recvfrom(1024)  # Buffer size of 1024 bytes
            message = data.decode('utf-8')
            self.get_logger().debug(f"Received message from {addr}: {message}")

            # Parse the received message
            message_parts = message.split(", ")
            x, y, z = float(message_parts[1]), float(message_parts[2]), float(message_parts[3])
            qx, qy, qz, qw = float(message_parts[4]), float(message_parts[5]), float(message_parts[6]), float(message_parts[7])

            # received_position = {'x': x, 'y': y, 'z': z}
            # received_orientation = {'x': qx, 'y': qy, 'z': qz, 'w': qw}

            # # Apply the inverted offset to the received pose
            # offset_position = self.offset['position']
            # offset_orientation = self.offset['orientation']
            # inverted_offset_position, inverted_offset_orientation = self.invert_offset(offset_position, offset_orientation)

            # # Adjust the position and orientation
            # adjusted_position = {
            #     'x': received_position['x'] + inverted_offset_position[0],
            #     'y': received_position['y'] + inverted_offset_position[1],
            #     'z': received_position['z'] + inverted_offset_position[2],
            # }
            # adjusted_orientation = tf_transformations.quaternion_multiply(
            #     (qx, qy, qz, qw), inverted_offset_orientation)
            
            # Set received position without reordering
            received_position = np.array([x, y, z])  
            received_orientation = np.array([qx, qy, qz, qw])

            # # Load the combined transform matrix
            # combined_inverted_transform = self.load_combined_inverse_offset_transform(self.offset['position'], self.offset['orientation'])

            # # Apply the combined transform to the position
            # adjusted_position_homogeneous = np.dot(combined_inverted_transform, received_position)
            # adjusted_position = adjusted_position_homogeneous[:3]  # Extract x, y, z from homogeneous coordinates

            # adjusted_orientation = tf_transformations.quaternion_multiply(received_orientation, orientation_offset)

            # Publish the adjusted PoseStamped message
            #pose_msg = PoseStamped()
            #pose_msg.header.stamp = self.get_clock().now().to_msg()
            #pose_msg.header.frame_id = "world"
            #pose_msg.pose.position.x = T_world_correctedbird[0,3]
            #pose_msg.pose.position.y = T_world_correctedbird[1,3]
            #pose_msg.pose.position.z = T_world_correctedbird[2,3]
            #pose_msg.pose.orientation.x = quaternion_world_correctedbird[0]
            #pose_msg.pose.orientation.y = quaternion_world_correctedbird[1]
            #pose_msg.pose.orientation.z = quaternion_world_correctedbird[2]
            #pose_msg.pose.orientation.w = quaternion_world_correctedbird[3]
            #self.pose_publisher.publish(pose_msg)

            # Adjusting the transforms for PX4, based on Saxion repo
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = -y
            pose_msg.pose.position.z = -z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = -qy
            pose_msg.pose.orientation.z = -qz
            pose_msg.pose.orientation.w = qw
            self.pose_publisher.publish(pose_msg)

            # Broadcast transform
            #t = TransformStamped()
            #t.header.stamp = self.get_clock().now().to_msg()
            #t.header.frame_id = "world"
            #t.child_frame_id = "metafly_base"
            #t.transform.translation.x = T_world_correctedbird[0,3]
            #t.transform.translation.y = T_world_correctedbird[1,3]
            #t.transform.translation.z = T_world_correctedbird[2,3]
            #t.transform.rotation.x = quaternion_world_correctedbird[0]
            #t.transform.rotation.y = quaternion_world_correctedbird[1]
            #t.transform.rotation.z = quaternion_world_correctedbird[2]
            #t.transform.rotation.w = quaternion_world_correctedbird[3]
            #self.tf_broadcaster.sendTransform(t)

            #Broadcast transform for PX4 version
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "quad_base"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        except socket.timeout:
            self.get_logger().warn('No data received, retrying...')
        except Exception as e:
            self.get_logger().debug(f"Error receiving data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
