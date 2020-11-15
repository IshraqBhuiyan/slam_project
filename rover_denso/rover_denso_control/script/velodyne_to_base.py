#!/usr/bin/env python
import sys
import copy
import math
import rospy
import tf
import geometry_msgs.msg
import numpy as np
from pyquaternion import Quaternion
## END_SUB_TUTORIAL

def create_H(trans, rot):
	H = np.zeros((4,4))
	q = Quaternion([rot[3],rot[0], rot[1], rot[2]])
	H[0:3,0:3] = q.rotation_matrix
	for i in range(3): H[i,3] = trans[i]
	H[3,3] = 1
	return H

if __name__=='__main__':
	rospy.init_node('velodyne_to_base')
	t  = tf.TransformListener()
	br = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
		try:
			(trans_vel_baselink, rot_vel_baselink) = t.lookupTransform('/velodyne_sensor', '/base_link', rospy.Time(0))
			(trans_world_vel, rot_world_vel) = t.lookupTransform('/map', '/velodyne', rospy.Time(0))

			H_vel_baselink = create_H(trans_vel_baselink, rot_vel_baselink)
			H_world_vel = create_H(trans_world_vel, rot_world_vel)

			H_world_baselink = np.matmul(H_world_vel, H_vel_baselink)

			q = Quaternion(matrix=H_world_baselink).elements
			trans_world_baselink = (H_world_baselink[0,3], H_world_baselink[1,3], H_world_baselink[2,3])
			rot_world_baselink = (q[1], q[2], q[3], q[0])

			br.sendTransform(trans_world_baselink, rot_world_baselink, rospy.Time.now(), "/base_link", "/map")
			# br.sendTransform((0,0,0), (1,0,0,0), rospy.Time(0), "/world", "/map")

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("TF lookup error")
			continue
