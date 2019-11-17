"""
msg types from RadarScan

uint16 point_id
float32 x
float32 y
float32 z
float32 range
float32 velocity
uint16 doppler_bin
float32 bearing
float32 intensity
"""

import rosbag
import numpy as np
from ti_mmwave_rospkg.msg import RadarScan
print("Hi")
bag = rosbag.Bag('input.bag',"r")
messages = bag.read_messages(topics=["/ti_mmwave/radar_scan"])
msg_count = bag.get_message_count(topic_filters=["/ti_mmwave/radar_scan"])

print('original', msg_count)

ids = []
points_cord = []
points_range = []
batch = []

# Of the type [x,y,z,range, velocity, bearing]
readings = np.array([0,0,0,0,0,0,0,0])
for msg_topic, msg, t in bag.read_messages(topics=["/ti_mmwave/radar_scan"]):
	# print("New message")
	# print(msg.point_id)
	if (msg.point_id != 0):
		readings[0], readings[1] = msg.point_id, msg.header.seq
		readings[2], readings[3], readings[4] = msg.x, msg.y, msg.z
		readings[5], readings[6], readings[7] = msg.range, msg.velocity, msg.bearing
		points_cord.append(readings) 
	else:
		# print("New batch")
		# print(points_cord)
		# batch.append(ids)
		batch.append(points_cord)
		ids = []
		points_cord = []


print(np.shape(batch[2]))
batch = np.asarray(batch)
print(batch[2])
# print(batch)

# for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
# for topic, msg, t in bag.read_messages(topics=['header']):
# 	print(msg)
bag.close()
