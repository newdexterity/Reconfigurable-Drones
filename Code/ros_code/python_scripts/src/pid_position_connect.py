#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
from vicon_bridge.msg import Markers
from math import sin, cos, sqrt, atan2, pi

ESP32_IP = "192.168.4.1"
ESP32_PORT = 80

MAX_VX = 0.3
MAX_VY = 0.3
MAX_WZ = 0.5

class ConnectorController:

    def __init__(self, module_name, connector_name, target_connector_name, angle_reference_module, _yaw_diff):
        self.module_name = module_name
        self.connector_name = connector_name
        self.target_connector_name = target_connector_name
        self.angle_reference_module = angle_reference_module if angle_reference_module else target_connector_name.split('_')[0]
        
        self.cmd_vel_pub = rospy.Publisher(f"/{module_name}/cmd_vel", Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)
        
        self.module_orientations = {}

        # Subscribe to vicon/module_n topics to get TransformStamped data
        rospy.Subscriber(f"vicon/{self.module_name}/{self.module_name}", TransformStamped, self.module_callback, self.module_name)
        rospy.Subscriber(f"vicon/{self.angle_reference_module}/{self.angle_reference_module}", TransformStamped, self.module_callback, self.angle_reference_module)
        
        # PID coefficients for X and Y
        self.Kp_x = 0.5
        self.Ki_x = 0.0
        self.Kd_x = 0
        self.prev_error_x = 0.05
        self.integral_x = 0

        self.Kp_y =1
        self.Ki_y = 0.0
        self.Kd_y = 0
        self.prev_error_y = 0.05
        self.integral_y = 0

        # PID coefficients for relative yaw
        self.Kp_theta = 1
        self.Ki_theta = 0.00
        self.Kd_theta = 0
        self.prev_error_theta = 0.05
        self.integral_theta = 0
        
        self.yaw_diff = _yaw_diff

    def module_callback(self, msg, module_name):
        # Extract the yaw from the quaternion data
        _, _, yaw = tf.transformations.euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        self.module_orientations[module_name] = yaw

    def get_relative_yaw(self):
        if self.module_name in self.module_orientations and self.angle_reference_module in self.module_orientations:
            yaw1 = self.module_orientations[self.module_name]
            yaw2 = self.module_orientations[self.angle_reference_module]
            relative_yaw = yaw1 - yaw2 + self.yaw_diff
            # Normalize to [-pi, pi]
            relative_yaw = atan2(sin(relative_yaw), cos(relative_yaw))
            return -relative_yaw
        else:
            return None

    def pid_control_x(self, error, prev_error, integral):
        derivative = error - prev_error
        if(error < 0.05):
            integral += error
        else:
            integral = 0
        output = self.Kp_x * error + self.Ki_x * integral + self.Kd_x * derivative
        return output, integral

    def pid_control_y(self, error, prev_error, integral):
        derivative = error - prev_error
        if(error < 0.05):
            integral += error
        else:
            integral = 0
        output = self.Kp_y * error + self.Ki_y * integral + self.Kd_y * derivative
        return output, integral
    
    def pid_control_theta(self, error, prev_error, integral):
            derivative = error - prev_error
            integral += error
            output = self.Kp_theta * error + self.Ki_theta * integral + self.Kd_theta * derivative
            return output, integral

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def control_connector(self, trans_self, trans_target):
        # Errors in vicon/world frame
        error_x_world = trans_target[0] - trans_self[0]
        error_y_world = trans_target[1] - trans_self[1]
        
        # Get the transform from vicon/world to the module's frame
        (trans, rot) = self.listener.lookupTransform('vicon/world', f'vicon/{self.module_name}/{self.module_name}', rospy.Time(0))
        quaternion_world = [rot[0], rot[1], rot[2], rot[3]]
        _, _, yaw_world_to_module = tf.transformations.euler_from_quaternion(quaternion_world)


        # Transform the error to the module's frame
        error_x_module = error_x_world * cos(yaw_world_to_module) + error_y_world * sin(yaw_world_to_module) - 0.04
        error_y_module = -error_x_world * sin(yaw_world_to_module) + error_y_world * cos(yaw_world_to_module) + 0.08

        
        # Control actions in module frame for X and Y
        linear_x, self.integral_x = self.pid_control_x(error_x_module, self.prev_error_x, self.integral_x)
        linear_y, self.integral_y = self.pid_control_y(error_y_module, self.prev_error_y, self.integral_y)
        
        relative_yaw = self.get_relative_yaw()
        if relative_yaw is not None:
            # print(self.prev_error_theta)
            angular_z, self.integral_theta = self.pid_control_theta(relative_yaw, self.prev_error_theta, self.integral_theta)
            self.prev_error_theta = relative_yaw
        else:
            angular_z = 0
        
        # print(error_x_module, error_y_module, relative_yaw)
        
        self.prev_error_x = error_x_module
        self.prev_error_y = error_y_module
        # rospy.loginfo(f'{error_x_module}, {error_y_module}')
        
        # if self.integral_x > 0.20: self.integral_x = 0.20
        # if self.integral_y > 0.30: self.integral_y = 0.30
        
        # Limit velocities
        linear_x = min(max(linear_x, -MAX_VX), MAX_VX)
        linear_y = min(max(linear_y, -MAX_VY), MAX_VY)
        angular_z = min(max(angular_z, -MAX_WZ), MAX_WZ)
        # print(error_x_module, error_y_module)
        
        if relative_yaw is not None and abs(relative_yaw) > 30*pi/180: 
            linear_x = 0
            linear_y = 0
        
        # Control for relative yaw

        # Create a Twist message and publish
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)
        
    def is_connected(self):
        # This threshold can be adjusted based on your requirements
        POSITION_THRESHOLD = 0.02
        ORIENTATION_THRESHOLD = 5 * pi / 180  # 5 degrees

        # Check if the position error and orientation error are below the thresholds
        if abs(self.prev_error_x) < POSITION_THRESHOLD and abs(self.prev_error_y) < POSITION_THRESHOLD and abs(self.prev_error_theta) < ORIENTATION_THRESHOLD:
            return True
        return False

    def vicon_callback(self, markers_msg):
        trans_self = None
        trans_target = None

        for marker in markers_msg.markers:
            if marker.marker_name == self.connector_name:
                trans_self = [marker.translation.x/1000, marker.translation.y/1000]
            elif marker.marker_name == self.target_connector_name:
                trans_target = [marker.translation.x/1000, marker.translation.y/1000]

        if trans_self and trans_target:
            self.control_connector(trans_self, trans_target)


if __name__ == "__main__":
    rospy.init_node("connector_controller")
    # rospy.loginfo("Connector controller node started.")
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.angular.z = 0
    module3_controller = ConnectorController("module_3", "module_3_connector", "module_1_claw", "module_1", 60*pi/180)
    module2_controller = ConnectorController("module_2", "module_2_connector", "module_3_claw", "module_3", 60*pi/180)
    module4_controller = ConnectorController("module_4", "module_4_connector", "module_2_claw", "module_2", 60*pi/180)

    
    # Start with module_3
    module3_subscriber = rospy.Subscriber("/vicon/markers", Markers, module3_controller.vicon_callback)
    while not module3_controller.is_connected() and not rospy.is_shutdown():
        rospy.sleep(0.1)

    module3_controller.cmd_vel_pub.publish(twist_msg)
    module3_subscriber.unregister()
    
    
    # Once module_3 is connected, start module_2
    module2_subscriber = rospy.Subscriber("/vicon/markers", Markers, module2_controller.vicon_callback)
    while not module2_controller.is_connected() and not rospy.is_shutdown():
        rospy.sleep(0.1)
        
    module2_controller.cmd_vel_pub.publish(twist_msg)
    module2_subscriber.unregister()
    


    # Once module_2 is connected, start module_4
    module4_subscriber = rospy.Subscriber("/vicon/markers", Markers, module4_controller.vicon_callback)
    while not module4_controller.is_connected() and not rospy.is_shutdown():
        rospy.sleep(0.1)
        
    module4_controller.cmd_vel_pub.publish(twist_msg)
    module4_subscriber.unregister()
    

    
    
