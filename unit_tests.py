#!/bin/python

import unittest
from cloud_manipulations import CloudManipulations
import numpy as np

class TestPointCloudManipulationMethods(unittest.TestCase):
	rotation_angle = 30
	
	def test_rotations(self):
		cm = CloudManipulations()
		rotated_along_x = cm.rotate_x (self.rotation_angle, False)
		test_matrix_x = cm.apply_rotational_offset(rotated_along_x, 1.0, 2.2, 3.4)
 
		rotated_along_y = cm.rotate_y (self.rotation_angle, False)
		test_matrix_y = cm.apply_rotational_offset(rotated_along_y, 1.0, 2.2, 3.4)

		rotated_along_z = cm.rotate_z (self.rotation_angle, False)
		test_matrix_z = cm.apply_rotational_offset(rotated_along_z, 1.0, 2.2, 3.4)

		actual_rotated_along_x = np.array([[1., 0., 0., 0.],
       						   [0., 0.15425145, -0.98803162,  3.52686725],
       						   [0., 0.98803162,  0.15425145,  0.27979621],
       						   [0 , 0.        ,  0.        ,  1.        ]])

		actual_rotated_along_y = np.array([[ 0.15425145,  0.        ,  0.98803162, -1.32792102],
       						   [ 0.        ,  1.        ,  0.        ,  0.        ],
       						   [-0.98803162,  0.        ,  0.15425145,  2.84867843],
       						   [ 0.        ,  0.        ,  0.        ,  1.        ]])
		

		actual_rotated_along_z = np.array([[ 0.15425145, -0.98803162,  0.        ,  2.42659915],
       						   [ 0.98803162,  0.15425145,  0.        ,  0.36516606],
       						   [ 0.        ,  0.        ,  1.        ,  0.        ],
       						   [ 0.        ,  0.        ,  0.        ,  1.        ]])
		# Check X rotation	
		rotated = np.around(test_matrix_x, decimals=8).flatten().tolist()
		actual = actual_rotated_along_x.flatten().tolist()
		self.assertTrue(actual == rotated)		
		
		# Check Y rotation
		rotated = np.around(test_matrix_y, decimals=8).flatten().tolist()
		actual = actual_rotated_along_y.flatten().tolist()
		self.assertTrue(actual == rotated)
		# Check Z rotation
		rotated = np.around(test_matrix_z, decimals=8).flatten().tolist()
		actual = actual_rotated_along_z.flatten().tolist()
		self.assertTrue(actual == rotated)			
				
	def test_apply_offset(self):
		cm = CloudManipulations()
		offset = 1
		points = (1, 2, 3)
		actual_points = (2, 3, 4)
		test_points = cm.apply_offset(points, 1)
		self.assertTrue(actual_points, test_points)	

	def test_mat_to_point_cloud(self):
		cm = CloudManipulations()
		ones_matrix = np.ones((4,4))
		mat = np.ones((4,4))
		test_data = '\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?'	
		point_cloud = cm.mat_to_point_cloud(ones_matrix, 'velodyne_pt')
		self.assertEquals(point_cloud.data, test_data)		

if __name__ == '__main__':
	suite = unittest.TestLoader().loadTestsFromTestCase(TestPointCloudManipulationMethods)
	unittest.TextTestRunner(verbosity=2).run(suite)
