#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from math import sin, cos
import struct
import sensor_msgs.msg as sensor_msgs

class ViconToPointCloud:
    def __init__(self, object_names, update_frequency):
        self.object_names = object_names
        self.listener = tf.TransformListener()
        self.update_frequency = update_frequency
        self.positions = {name: None for name in object_names}
        self.publishers = {name: rospy.Publisher('/{}/obstacles'.format(name), PointCloud2, queue_size=10) for name in object_names}
        
        # Dimensions for the payload box rectangle (assuming 0.5x0.3 meters for now)
        self.box_length = 0.5
        self.box_width = 0.3
        
        # Start the update and publish loop
        self.update_and_publish_loop()

    def generate_circle_points(self, center, radius, num_points=6):
        """Generate points in a circle around a center point."""
        circle_points = []
        for i in range(num_points):
            angle = 2 * 3.14159 * i / num_points
            x = center[0] + radius * cos(angle)
            y = center[1] + radius * sin(angle)
            circle_points.append((x, y, center[2]))
        return circle_points

    def generate_rectangular_points(self, center):
        """Generate points forming a rectangle around the center."""
        half_length = self.box_length / 2
        half_width = self.box_width / 2
        corners = [
            (center[0] - half_length, center[1] - half_width, center[2]),
            (center[0] + half_length, center[1] - half_width, center[2]),
            (center[0] - half_length, center[1] + half_width, center[2]),
            (center[0] + half_length, center[1] + half_width, center[2])
        ]
        return corners

    def update_position(self, object_name):
        try:
            # Lookup the transform
            (trans, rot) = self.listener.lookupTransform('vicon/world', 'vicon/{}/{}'.format(object_name, object_name), rospy.Time(0))
            
            # Extract and update the position
            self.positions[object_name] = (trans[0], trans[1], trans[2])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def publish_cloud(self, for_module):
        # Generate the cloud
        cloud_points = []
        for object_name, position in self.positions.items():
            if position and object_name != for_module:
                cloud_points.append(position)
                cloud_points.extend(self.generate_circle_points(position, 0.2))
                if object_name == "payload_box":
                    cloud_points.extend(self.generate_rectangular_points(position))

        # Publish the cloud
        cloud = sensor_msgs.PointCloud2()
        
        # Set the header information
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "vicon/world"
        
        # Define point fields
        cloud.fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1)
        ]
        cloud.point_step = 12  # 4 bytes * 3 for x, y, z
        cloud.height = 1  # Unorganized point cloud
        cloud.width = len(cloud_points)  # Number of points in the cloud
        
        # Convert tuples to PointCloud2 format and add to cloud
        packed_points = [struct.pack('fff', p[0], p[1], p[2]) for p in cloud_points]
        cloud.data = b''.join(packed_points)
        
        self.publishers[for_module].publish(cloud)

    def update_and_publish_loop(self):
        rate = rospy.Rate(self.update_frequency)
        while not rospy.is_shutdown():
            for object_name in self.object_names:
                self.update_position(object_name)
            for module in self.object_names:
                if "module" in module:  # only modules need obstacle clouds
                    self.publish_cloud(module)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('vicon_to_pointcloud')
    rospy.loginfo("vicon to pointcloud")
    
    # Fetch parameters
    update_frequency = rospy.get_param('~update_frequency', 10)  # Default 10 Hz if not set

    # Start the converter
    converter = ViconToPointCloud(["module_1", "module_2", "module_3", "module_4", "payload_box"], update_frequency)


# #!/usr/bin/env python3

# import rospy
# import tf
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
# from math import sin, cos
# import rospy
# import struct
# import sensor_msgs.msg as sensor_msgs

# class ViconToPointCloud:
#     def __init__(self, object_names, update_frequency):
#         # print(object_names)
#         self.object_names = object_names
#         self.listener = tf.TransformListener()
#         self.update_frequency = update_frequency
#         self.positions = {name: None for name in object_names}
#         self.publishers = {name: rospy.Publisher('/{}/obstacles'.format(name), PointCloud2, queue_size=10) for name in object_names}
        
#         # Dimensions for the payload box rectangle (let's assume 1x1 meters for now)
#         self.box_length = 0.5
#         self.box_width = 0.3
        
#         # Start the update and publish loop
#         self.update_and_publish_loop()

#     def generate_circle_points(self, center, radius, num_points=6):
#         """Generate points in a circle around a center point."""
#         circle_points = []
#         for i in range(num_points):
#             angle = 2 * 3.14159 * i / num_points
#             x = center[0] + radius * cos(angle)
#             y = center[1] + radius * sin(angle)
#             circle_points.append((x, y))
#         return circle_points

#     def generate_rectangular_points(self, center):
#         """Generate points forming a rectangle around the center."""
#         half_length = self.box_length / 2
#         half_width = self.box_width / 2
#         corners = [
#             (center[0] - half_length, center[1] - half_width),
#             (center[0] + half_length, center[1] - half_width),
#             (center[0] - half_length, center[1] + half_width),
#             (center[0] + half_length, center[1] + half_width)
#         ]
#         return corners

#     def update_position(self, object_name):
#         try:
#             # Lookup the transform
#             (trans, rot) = self.listener.lookupTransform('vicon/world', 'vicon/{}/{}'.format(object_name, object_name), rospy.Time(0))
#             # print(trans)
#             # Extract and update the position
#             self.positions[object_name] = (trans[0], trans[1],trans[2])

#             # rospy.loginfo(f"Position of {object_name} is {self.positions[object_name]}")
            
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             pass

#     def publish_cloud(self, for_module):
#         # Generate the cloud
#         cloud_points = []
#         for object_name, position in self.positions.items():
#             if position and object_name != for_module:
#                 cloud_points = [position]
#                 packed_point = struct.pack('fff', position[0], position[1], position[2])
#                 cloud_points.extend(self.generate_circle_points(position, 0.2))
#                 cloud_points.extend(self.generate_circle_points(position, 0.2))
#             if object_name == "payload_box":
#                 cloud_points.extend(self.generate_rectangular_points(position))

#         # print(cloud_points)
#         # Publish the cloud
#         cloud = sensor_msgs.PointCloud2()
        
#         # Set the header information
#         cloud.header.stamp = rospy.Time.now()
#         cloud.header.frame_id = "vicon/world"  # Replace with your frame_id
        
#         # Define point fields
#         cloud.fields = [
#             sensor_msgs.PointField(name="x", offset=0, datatype=sensor_msgs.PointField.FLOAT32),
#             sensor_msgs.PointField(name="y", offset=4, datatype=sensor_msgs.PointField.FLOAT32),
#             sensor_msgs.PointField(name="z", offset=8, datatype=sensor_msgs.PointField.FLOAT32)
#         ]
#         cloud.point_step = 12  # 4 bytes * 3 for x, y, z
#         cloud.height = 1  # Unorganized point cloud
#         cloud.width = 1  # One point
        
#         # Convert tuple to PointCloud2 format and add to cloud
        
#         cloud.data = packed_point
        
#         # header = rospy.Header()
#         # header.stamp = rospy.Time.now()
#         # header.frame_id = 'vicon/world'
#         # fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 4, PointField.FLOAT32, 1)]
#         # cloud_msg = pc2.create_cloud(header, fields, cloud_points)
#         # cloud_msg.is_dense = True
#         # cloud_msg.width = len(cloud_points)

#         self.publishers[for_module].publish(cloud)

#     def update_and_publish_loop(self):
#         rate = rospy.Rate(self.update_frequency)
#         while not rospy.is_shutdown():
#             for object_name in self.object_names:
#                 self.update_position(object_name)
#             for module in self.object_names:
#                 if "module" in module:  # only modules need obstacle clouds
#                     self.publish_cloud(module)
#             rate.sleep()

# if __name__ == '__main__':
#     print('starting vicon to pointcloud')
#     rospy.init_node('vicon_to_pointcloud')
#     rospy.loginfo(" vicon to pointcloud")
#     # Fetch parameters
#     update_frequency = rospy.get_param('~update_frequency', 10)  # Default 10 Hz if not set

#     # Start the converter
#     converter = ViconToPointCloud(["module_1", "module_2", "module_3", "module_4", "payload_box"], update_frequency)
