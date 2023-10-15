#!/usr/bin/env python3

import rospy
import socket
from geometry_msgs.msg import Twist  # Import the Twist message

# Constants for ESP32 connection
ESP32_IP = "192.168.4.1"  # ESP32 Access Point IP
ESP32_PORT = 80

# Maximum allowable velocities
MAX_VX = 0.125 # Maximum linear x velocity
MAX_VY = 0.072 # Maximum linear y velocity
MAX_WZ = 0.72  # Maximum angular z velocity

# Establish socket connection with ESP32
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ESP32_IP, ESP32_PORT))

# Variables to store previous velocity values for smoothing
x_prev = [0,0,0,0]
y_prev = [0,0,0,0]
w_prev = [0,0,0,0]

def send_twist_to_esp32(twist_msg, module_id):
    """Sends the Twist message to the ESP32 module."""
    
    # Clamp the velocities to their maximum allowable values
    linear_x = max(min(twist_msg.linear.x, MAX_VX), -MAX_VX)
    linear_y = max(min(twist_msg.linear.y, MAX_VY), -MAX_VY)
    angular_z = max(min(twist_msg.angular.z, MAX_WZ), -MAX_WZ)
    
    # Remap the velocities to the range [-100, 100]
    linear_x = int(remap(linear_x, -MAX_VX, MAX_VX, -100, 100))
    linear_y = int(-remap(linear_y, -MAX_VY, MAX_VY, -100, 100))
    angular_z = int(-remap(angular_z, -MAX_WZ, MAX_WZ, -100, 100))
    
    # Smooth out abrupt changes in velocities
    if abs(linear_x - x_prev[module_id-1]) > 20: 
        linear_x = (linear_x + x_prev[module_id-1]) / 2
        x_prev[module_id-1] = linear_x
    if abs(linear_y - y_prev[module_id-1]) > 20: 
        linear_y = (linear_y + y_prev[module_id-1]) / 2
        y_prev[module_id-1] = linear_y
    if abs(angular_z - w_prev[module_id-1]) > 20: 
        angular_z = (angular_z + w_prev[module_id-1]) / 2
        w_prev[module_id-1] = angular_z
        
    # Construct the data string to send
    data_to_send = f"{module_id},{linear_x},{linear_y},{angular_z},{int(twist_msg.angular.x)}\n"

    # Send the data string to ESP32
    try:
        client_socket.send(data_to_send.encode())
        rospy.loginfo("Twist data sent to ESP32: %s", data_to_send)
    
    except Exception as e:
        rospy.logerr("Failed to send Twist data to ESP32: %s", str(e))

# Callback functions for each module's topic
def twist_callback1(twist_msg):
    send_twist_to_esp32(twist_msg, 1)

def twist_callback2(twist_msg):
    send_twist_to_esp32(twist_msg, 2)

def twist_callback3(twist_msg):
    send_twist_to_esp32(twist_msg, 3)

def twist_callback4(twist_msg):
    send_twist_to_esp32(twist_msg, 4)

def remap(value, original_min, original_max, target_min, target_max):
    """Remaps a value from one range to another."""
    return target_min + (value - original_min) * (target_max - target_min) / (original_max - original_min)

if __name__ == '__main__':
    rospy.init_node("esp32_ros_socket")
    
    # Subscribe to the cmd_vel topics for each module
    rospy.Subscriber("/module_1/cmd_vel", Twist, twist_callback1)
    rospy.Subscriber("/module_2/cmd_vel", Twist, twist_callback2)
    rospy.Subscriber("/module_3/cmd_vel", Twist, twist_callback3)
    rospy.Subscriber("/module_4/cmd_vel", Twist, twist_callback4)
    
    # Keep the node running
    rospy.spin()
