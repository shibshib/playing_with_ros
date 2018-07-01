#!/usr/bin/python
import rospy
from roslib import message
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import sys
from cloud_manipulations import CloudManipulations

# Angular rotations and linear translation (offset)
rotation=0
axis="x"
offset=0

# Publisher
pub = rospy.Publisher('velodyne_publish', PointCloud2, queue_size=10)

def callback(data):
	"""
	Callback function to take pointcloud data outputted by *.bag test file
	and perform manipulations on it, then publish the manipulated data. 
	"""
	cm = CloudManipulations()
	pointcloud = pc2.read_points(data, skip_nans=True, field_names=["x","y","z"])
	for p in pointcloud:
		x = p[0]
		y = p[1]
		z = p[2]
		
		# Apply some linear translation (offset)
		p = cm.apply_offset(p, offset)
		
		# Rotate by some degrees on an axis
		if (axis.lower() == "x"):
			rotated = cm.rotate_x(rotation, False)
		elif (axis.lower() == "y"):
			rotated = cm.rotate_y(rotation, False)
		elif (axis.lower() == "z"):
			rotated = cm.rotate_z(rotation, False)
		else:
			print ("Invalid axis ", axis.lower())
			return None

		# Apply the offset for rotation
		tm = cm.apply_rotational_offset(rotated, x, y, z)

		# Convert to point cloud
		pc = cm.mat_to_point_cloud(tm, 'velodyne_edited')	

		# Publish the pointcloud
		rospy.loginfo(pc)
		
		pub.publish(pc)									
	
def transform_velodyne_cloud():
	"""
	Function to initialize the node and subscriber
	"""
	rospy.init_node('velodyne_editor', anonymous=True)
	
	rospy.Subscriber("velodyne_points", PointCloud2, callback)

	rospy.spin()


if __name__ == '__main__':
	if (len(sys.argv) != 4 or sys.argv[1] == "help"):
		print("Usage: python node.py <angular_rotation> <rotation_axis> <offset>")
		sys.exit()
	elif(len(sys.argv) == 4):
		rotation = int(sys.argv[1])
		axis = sys.argv[2]
		offset = int(sys.argv[3])
	
	try:
		transform_velodyne_cloud()
	except rospy.ROSInterruptException as e:
		print("An error has occured {0}: {1}".format(e.errno, e.strerror))
	
