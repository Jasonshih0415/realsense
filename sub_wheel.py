import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import itertools
import std_msgs.msg
import time

class PointCloudVisualizer:
    def __init__(self):
        self.points = np.empty((0, 3))
        self.wh_points = np.empty((0, 3))

        rospy.init_node('sub_wheel', anonymous=True)
        rospy.Subscriber('/filtered_points', PointCloud2, self.callback_pointcloud)

    def callback_pointcloud(self, data):
        start_time = time.time()
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        self.points = np.round(np.array(list(gen)), 2)
        print('x', self.points[0][1] )
        
        
        processing_time = time.time() - start_time
        print("time: {:.6f} sec".format(processing_time))
if __name__ == '__main__':
    visualizer = PointCloudVisualizer()
    rospy.spin()
