#!/usr/bin/env python
import rospy
import math
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from ti_mmwave_rospkg.msg import RadarScanArray
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from sklearn.cluster import DBSCAN
from collections import Counter
import matplotlib.pyplot as plt


"""
Direction Convention:
y-axis = Sideways, right -ve 
x-axis = Front and back, front +ve
"""
def get_cluster(arr):
  db = DBSCAN(eps=0.05, min_samples=3).fit(arr)
  labels = db.labels_
  Counter(labels)
  core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
  core_samples_mask[db.core_sample_indices_] = True
  labels = db.labels_
  n_clusters_ = len(set(labels)) - (1 if -1 else 0)
  unique_labels = set(labels)
  colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
  ans = []

  for k, col in zip(unique_labels, colors):
      if k == -1:
          # Black used for noise.
          col = 'k'

      class_member_mask = (labels == k)

      xy = arr[class_member_mask & ~core_samples_mask] 
      if (k != -1):
        point = np.mean(xy, axis = 0)
        ans.append(point)
      xy = arr[class_member_mask & core_samples_mask]

  points = np.asarray(ans)
  # print(np.shape(points))
  return points

def compute_velocity(points):
  time_elapsed = 0.0338
  print(points)
  if (len(points) > 0):
    print("Computing for this")
  pass  


def callback(data):
    print("start")
    # print(data.point_id)
    # print("end")
    # points[:,0]=pc['x']
    # points[:,1]=pc['y']
    # points[:,2]=pc['z']


def reader():
   # pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.init_node('clustered_data')
   rospy.loginfo("Getting the data")
   cloud = rospy.Subscriber("/ti_mmwave/radar_scan", RadarScanArray, callback)
   rospy.spin()
   # rate = rospy.Rate(10) # 10hz
   # while not rospy.is_shutdown():
   #     hello_str = "hello world %s" % rospy.get_time()
   #     rospy.loginfo(hello_str)
   #     pub.publish(hello_str)
   #     rate.sleep()

if __name__ == '__main__':
  try:
     reader()
  except rospy.ROSInterruptException:
     pass
