#!/usr/bin/env python
##################################################
# Author: Anthoney Park	
# Date: 9/3/2014
##################################################
import rospy
from openravepy import *
import rospkg
import numpy, time
from openravepy import ikfast
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2

import geometry_msgs
from tf import transformations as tr
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math


rospack = rospkg.RosPack()
rave_to_moveit_path = rospack.get_path('adept_motion_planning')

env=Environment()
env.Load(rave_to_moveit_path + '/scenes/test2.env.xml')
env.SetViewer('qtcoin')
obj_name = ''

#The main function grabs the environment you want, setting the robot as the first
#robot tag put into your env file. Then it sets up the ikfast, the target for the robot
#and the database grasps are store in. Then send to the TransformToPose function
def main(mesh_name): 
	#If there was a previous object, delete it:
	global env
	global obj_name
	if obj_name != '':
		print "Removing object: ", obj_name
		remove = env.GetKinBody(obj_name)
		env.RemoveKinBody(remove)
	else:
		print "No object in simulation environment to remove."

	#Must be placed in the openrave openrave/src/ folder
	newObject = mesh_name #raw_input("Enter filename (include tag) for loading: ")
	env.Load(newObject)
	comp = newObject.split('.')
	path = comp[0].split('/')
	obj_name = path[-1]
	print "obj_name: ", obj_name

	#REMOVE CODE!
	#remove = env.GetKinBody(comp[0])
	#env.RemoveKinBody(remove)
	#############trimeshFromPointCloud()

	#grasp_target = raw_input("Please enter name of object you want to grasp: ")
	grasp_target = obj_name

	robot = env.GetRobots()[0]
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
	if not ikmodel.load():
		ikmodel.autogenerate()
	target = env.GetKinBody(grasp_target)

	#aabb=env.GetRobots()[0].ComputeAABB()
	#env.assertGreaterEqual(aabb.pos()[-1]-aabb.extents()[-1],0)
	#if aabb.pos()[-1]-aabb.extents()[-1] < 0:
	#	print "Bounding box assertion failed in main()"
	#	print aabb.pos()[-1]
	#	print aabb.extents()[-1]
	#	exit()

	print "Before gmodel"
	gmodel = databases.grasping.GraspingModel(robot,target)

	print "before gmodel.load"
	#gmodel.load()
	#if not gmodel.load():
	#gmodel.numthreads = 1
	gmodel.autogenerate()
	
	print "just before grasp num"
	graspnum = input("Please enter number of valid grasps: ")
	validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=graspnum)
	basemanip = interfaces.BaseManipulation(robot)
	pose_array = []
	print "HERE LOLLOLOLOLOLLLOOLOOROFLRORLFOFLROFLRO"
	with robot:
		for x in range (0,graspnum):
			grasp = validgrasps[x]
			gmodel.setPreshape(grasp)
			T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
			p = TransformToPose(T)
			pose_array.append(p)
			traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
			print('Finished Grasp')
			robot.GetController().SetPath(traj)
			#robot.WaitForController(0)
		
		print pose_array

	publish_poses(pose_array)
	#raveLogInfo((T))

#The TransformToPose function will take the trajectory that openrave finds and
#transform them into orientation and position, for setting up a pose that can be
#sent to MoveIt! storing in a message list
def TransformToPose( G ):
	"""
	Compute geometry_msgs::Pose from 4x4 transform
	G is a 4x4 matrix which is [R t; 0 0 0 1]
	"""
	tj = array( G )[0:3,3]
	q = tr.quaternion_from_matrix( G )
	orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] )
	position = geometry_msgs.msg.Point( tj[0], tj[1], tj[2] )
	pose = geometry_msgs.msg.Pose( position, orientation )
	return pose

def callback(data):
	rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
	main(data.data)

#def callbackcloud(data):
#	cloud = trimeshFromPointCloud(data)
#	env.load(cloud)

def listen_STLpath():
	#rospy.Subscriber("camera/depth_registered/2hz_playback_pts", PointCloud2, callbackcloud)
	rospy.Subscriber("object_path", String, callback)
	rospy.spin()

def publish_poses(pose_array):
	pub = rospy.Publisher('adept_wrist_orientation', PoseArray, queue_size=10)
	pose_msg = PoseArray()
	pose_msg.poses = pose_array
	pose_msg.header.stamp = rospy.Time.now()
	pose_msg.header.frame_id = "/world"
	
	pub.publish(pose_msg)

#def test_publish():
#	rostopic pub object_path std_msgs/String "hello there"


if __name__ == '__main__':
	rospy.init_node('SimEnvLoading', anonymous=True)
	#main()
	listen_STLpath()
