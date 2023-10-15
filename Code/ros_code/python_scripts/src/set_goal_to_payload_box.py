#!/usr/bin/env python3

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SetGoalToPayloadBox:
    def __init__(self, module_name):
        self.module_name = module_name
        self.goal_client = actionlib.SimpleActionClient('/{}/move_base'.format(self.module_name), MoveBaseAction)
        self.goal_client.wait_for_server()
        
        # Initialize TF listener
        self.listener = tf.TransformListener()
        
        # Rate of 1 Hz
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('map', 'vicon/payload_box/payload_box', rospy.Time(0))
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                
                goal.target_pose.pose.position.x = trans[0]
                goal.target_pose.pose.position.y = trans[1]
                _, _, yaw = euler_from_quaternion(rot)
                q = quaternion_from_euler(0, 0, yaw)
                goal.target_pose.pose.orientation.x = q[0]
                goal.target_pose.pose.orientation.y = q[1]
                goal.target_pose.pose.orientation.z = q[2]
                goal.target_pose.pose.orientation.w = q[3]
                
                # Adjust positions based on module name
                if self.module_name == "module_1":
                    goal.target_pose.pose.position.y = trans[1] + 0.5
                    _, _, yaw = euler_from_quaternion(rot)
                    q = quaternion_from_euler(0, 0, yaw+3.14)
                    goal.target_pose.pose.orientation.x = q[0]
                    goal.target_pose.pose.orientation.y = q[1]
                    goal.target_pose.pose.orientation.z = q[2]
                    goal.target_pose.pose.orientation.w = q[3]
                    
                elif self.module_name == "module_4":
                    goal.target_pose.pose.position.y = trans[1] - 0.5
                
                elif self.module_name == "module_3":
                    goal.target_pose.pose.position.x = trans[0] + 0.5
                    _, _, yaw = euler_from_quaternion(rot)
                    q = quaternion_from_euler(0, 0, yaw+3.14/2)
                    goal.target_pose.pose.orientation.x = q[0]
                    goal.target_pose.pose.orientation.y = q[1]
                    goal.target_pose.pose.orientation.z = q[2]
                    goal.target_pose.pose.orientation.w = q[3]
                    
                elif self.module_name == "module_2":
                    goal.target_pose.pose.position.x = trans[0] - 0.5
                    _, _, yaw = euler_from_quaternion(rot)
                    q = quaternion_from_euler(0, 0, yaw-3.14/2)
                    goal.target_pose.pose.orientation.x = q[0]
                    goal.target_pose.pose.orientation.y = q[1]
                    goal.target_pose.pose.orientation.z = q[2]
                    goal.target_pose.pose.orientation.w = q[3]

                # Send the goal to move_base
                self.goal_client.send_goal(goal)
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('set_goal_to_payload_box')
    module_name = rospy.get_param('~module_name', 'module_1')  # default to 'module_1' if not set
    SetGoalToPayloadBox(module_name)
    rospy.spin()
