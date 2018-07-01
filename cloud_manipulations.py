#!/bin/python

import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2

"""
cloud_manipulations.py Point Cloud manipulation library. 
"""
class CloudManipulations:
	def apply_rotational_offset(self, matrix, x, y, z):
    		""" Function to apply offset to given matrix.
    		Input:  matrix : (4,4) ndarray
    			x, y, z : uint
        	Output: transformed matrix
    		"""
    		o_x = float(x) / 2 + 0.5
    		o_y = float(y) / 2 + 0.5
    		o_z = float(z) / 2 + 0.5
	
	    	offset_matrix = np.array([[1, 0, 0, o_x],
	    	                          [0, 1, 0, o_y],
	    	                          [0, 0, 1, o_z],
	    	                          [0, 0, 0, 1]])
		
		reset_matrix = np.array([[1, 0, 0, -o_x],
		                         [0, 1, 0, -o_y],
		                         [0, 0, 1, -o_z],
		                         [0, 0, 0, 1]])
		
	    	transform_matrix = np.dot(np.dot(offset_matrix, matrix), reset_matrix)

    		return transform_matrix
	
	def apply_offset(self, points, offset):
		"""
		Function to apply offset to given set of points
		Input: points tuple (x, y, z)
		Output: new points tuple (x + offset, y + offset, z + offset)
		"""
		return points + (offset,)	

	##convert a 4xn matrix (x y z 1) to a PointCloud 
	def mat_to_point_cloud(self, mat, frame_id):
		"""
		Function to convert a given matrix to a simple pointcloud. 
		*NOTE* the pointcloud generated is missing many fields such as 
		row_step, is_dense, and is_bigendian. This is skipped for now but
		will be done in the future.
		Input: matrix, frame id
		Output: pointcloud2
		"""
    		pc = PointCloud2()
    		pc.header.frame_id = frame_id
    		pc_list = []
		for n in range(mat.shape[1]):
    			column = mat[:,n]
        		point = Point32()
       	 		x, y, z = column[0], column[1], column[2]
			pc_list.append([x, y, z])
        	pc.data = np.asarray(pc_list, np.float32).tostring()
    		return pc 

	# Convert pointcloud2 to 4xn matrix (x y z 1)
	def point_cloud_to_mat(self, point_cloud):
		"""
		Function to extract a matrix from a point cloud.
		Input: point cloud
		output: a 4xn matrix
		"""
		if (type(point_cloud) == type(PointCloud2())):
			points = [[p[0], p[1], p[2], 1] for p in pc2.read_points(point_cloud, skip_nans=True)]
		else:
			print ("Type not recognized: ", type(point_cloud))
			return None
		
		return np.matrix(points).transpose()
		  
	
	def rotate_x (self, angle, degrees=True):
		"""
		Function to generate rotation matrix along x-axis
		Input: angle, degrees (optional)
		Output: rotation matrix along x-axis
		"""
		if degrees:
			cx = np.cos(np.deg2rad(angle))
			sx = np.sin(np.deg2rad(angle))
		else:
			cx = np.cos(angle)
			sx = np.sin(angle)
		
		return np.array([[1, 0, 0, 0],
				[0, cx, sx, 0],
				[0, -sx, cx, 0],
				[0, 0, 0, 1]])
	
	def rotate_y (self, angle, degrees=True):
		"""
		Function to generate rotation matrix along y-axis
		Input: angle, degrees (optional)
		Output: rotation matrix along y axis
		"""
		if degrees:
			cy = np.cos(np.deg2rad(angle))
			sy = np.sin(np.deg2rad(angle))
		else:
			cy = np.cos(angle)
			sy = np.sin(angle)
		
		return np.array([[cy, 0, -sy, 0],
                     		[0, 1, 0, 0],
                     		[sy, 0, cy, 0],
                     		[0, 0, 0, 1]])

	def rotate_z (self, angle, degrees=True):
		"""
		Function to generate rotation matrix along z-axis
		Input: angle, degrees (optional)
		Output: rotation matrix along z-axis
		"""
		if degrees:
			cz = np.cos(np.deg2rad(angle))
			sz = np.sin(np.deg2rad(angle))
		else:
			cz = np.cos(angle)
			sz = np.sin(angle)

		return np.array([[cz, sz, 0, 0],
                     		[-sz, cz, 0, 0],
                     		[0, 0, 1, 0],
                     		[0, 0, 0, 1]])	
