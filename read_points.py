####################################################
##      File to read radar points in numpy        ##
####################################################


#!/usr/bin/env python
import rospy
import math
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import matplotlib.pyplot as plt


def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    # x = points[:,0]
    # y = points[:,1]
    # z = points[:,2]
    print(pc)
    


def read_radar():
   # pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.init_node('clustered_data', anonymous=True)
   rospy.loginfo("Getting the data")
   cloud = rospy.Subscriber("/deltarc_radar_position", PointCloud2,callback)
   rospy.spin()

if __name__ == '__main__':
  try:
     read_radar()
  except rospy.ROSInterruptException:
     pass