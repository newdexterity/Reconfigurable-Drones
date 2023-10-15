#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class ViconToOdom:
    def __init__(self, module_name):
        self.module_name = module_name

        # Subscribers and Publishers
        self.vicon_sub = rospy.Subscriber('/vicon/{}/{}'.format(self.module_name, self.module_name), TransformStamped, self.vicon_callback)
        self.odom_pub = rospy.Publisher('/{}/odom'.format(self.module_name), Odometry, queue_size=10)

        # Transform broadcaster
        self.broadcaster = tf.TransformBroadcaster()

        # Store previous position and time to calculate velocities
        self.prev_position = {}
        self.prev_time = {}

    def vicon_callback(self, data):
        current_time = rospy.Time.now()

        # Prepare odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "vicon/{}/{}".format(self.module_name,self.module_name)
        odom.child_frame_id = "{}/odom".format(self.module_name)

        # Position and orientation can be directly set from Vicon data
        odom.pose.pose.position = data.transform.translation
        odom.pose.pose.orientation = data.transform.rotation

        # Compute linear velocity
        if self.module_name in self.prev_position and self.module_name in self.prev_time:
            dt = (current_time - self.prev_time[self.module_name]).to_sec()
            dx = data.transform.translation.x - self.prev_position[self.module_name].x
            dy = data.transform.translation.y - self.prev_position[self.module_name].y
            dz = data.transform.translation.z - self.prev_position[self.module_name].z
            odom.twist.twist.linear.x = dx / dt
            odom.twist.twist.linear.y = dy / dt
            odom.twist.twist.linear.z = dz / dt

        self.prev_position[self.module_name] = data.transform.translation
        self.prev_time[self.module_name] = current_time

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Broadcast the transform
        self.broadcaster.sendTransform(
            (data.transform.translation.x, data.transform.translation.y, data.transform.translation.z),
            (data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w),
            current_time,
            odom.child_frame_id,
            odom.header.frame_id
        )

if __name__ == '__main__':
    rospy.init_node('vicon_to_odom')

    module_name = rospy.get_param('~module_name', 'module_1')  # default to 'module_1' if not set
    converter = ViconToOdom(module_name)
    rospy.spin()
