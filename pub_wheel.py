import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import std_msgs.msg
import time
from geometry_msgs.msg import Vector3  

class PointCloudVisualizer:
    def __init__(self):
        # Initialize arrays to store points
        self.points = np.empty((0, 3))
        self.r_points = np.empty((0, 3))  # Right side points
        self.l_points = np.empty((0, 3))  # Left side points
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Initialize the ROS node
        rospy.init_node('pylistener', anonymous=True)

        # Subscribe to the point cloud topic
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback_pointcloud)
        rospy.Subscriber('/imu/rpy', Vector3, self.callback_imu)

        # Set up publishers for right and left filtered points
        self.publisher = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)
        
    def callback_pointcloud(self, data):
        start_time = time.time()

        # Ensure the message is of type PointCloud2
        assert isinstance(data, PointCloud2)

        # Convert PointCloud2 to a numpy array with (x, y, z) points
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        self.points = np.round(np.array(list(gen)), 2)
        #print("self.points shape:",self.points.shape)
        
        processing_time = time.time() - start_time
        
        # Filter points for right (x=0.1) and left (x=-0.1)
        self.points = self.points[(self.points[:, 0] == 0.1) | (self.points[:, 0] == -0.1)]
        if len(self.points) > 0:
            self.transformed_points = self.apply_rpy_transformation(self.points)
            
        #print('Right points:', len(self.r_points), 'Left points:', len(self.l_points))
        print("Processing time: {:.6f} sec".format(processing_time))

        # Publish the filtered points
        self.publish_filtered_points()
        
    def apply_rpy_transformation(self, points):
        # Create rotation matrices for roll, pitch, and yaw
        self.alpha=np.radians(self.pitch+90)
        self.phi = np.radians(self.roll)
        #print("phi_radien:",self.phi)
        #print("phi_angle",self.roll)
        
        pitch_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(self.alpha), -np.sin(self.alpha)],
            [0, np.sin(self.alpha), np.cos(self.alpha)]
        ])


        roll_matrix = np.array([
            [np.cos(self.phi), -np.sin(self.phi), 0],
            [np.sin(self.phi), np.cos(self.phi), 0],
            [0, 0, 1]
        ])

        # Combine the rotation matrices into a single transformation matrix
        rotation_matrix =roll_matrix @ pitch_matrix
        #print("rotation_matrix shape:",rotation_matrix.shape)
        #print("points shape:",points.shape)
        # Apply the transformation to each point in the point cloud
        transformed_points = points @ rotation_matrix.T  # Matrix multiplication

        return transformed_points

    def callback_imu(self, data):
        # Update roll, pitch, and yaw from the IMU data
        self.pitch = data.x
        self.roll = data.y
        self.yaw = data.z
        
    def publish_filtered_points(self):
        # Create header for the PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_color_optical_frame"  # Adjust frame ID if necessary

        # Define the point fields for the PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        if len(self.transformed_points) > 0:
            cloud_msg = point_cloud2.create_cloud(header, fields, self.transformed_points)
            self.publisher.publish(cloud_msg)
            




if __name__ == '__main__':
    # Instantiate the visualizer and keep the node running
    visualizer = PointCloudVisualizer()
    rospy.spin()

