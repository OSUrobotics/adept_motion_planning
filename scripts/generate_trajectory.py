#!/usr/bin/env python

from openravepy import *
import numpy as np
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import time
from visualization_msgs.msg import Marker, MarkerArray

class GenerateTrajectory():
    def __init__(self,robot,scene,group):
        self.robot = robot
        self.scene = scene
        self.group = group
        self.marker_pub = rospy.Publisher("visualization_markers",MarkerArray,queue_size = 1)
        self.MarkerArray = MarkerArray()
        self.cpose = geometry_msgs.msg.Pose()
    
    def execute_trajectory(self,points = None,group=None):
        if group == None:
            try:
                group = self.group
            except:
                print "\n\nNo group specified"
                print "Usage: execute_trajectory(class_name,group_name,cartesian points)"
        #group.set_start_state_to_current_state()
        group.allow_replanning(True)
        self.cpose = group.get_current_pose().pose
        waypoints = [copy.deepcopy(self.cpose)]
        if points == None:
            print "\n\nFunction needs cartesian points"
            print "Usage: execute_trajectory(class_name,group_name,cartesian points)"
            return
        for point in points:
            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = self.cpose.position.x + point[0]
            wpose.position.y = self.cpose.position.y + point[1]
            wpose.position.z = self.cpose.position.z + point[2]
            print "printing",point[0],point[1],point[2]
            wpose.orientation.w = 1
            waypoints.append(wpose)
            del wpose
        print waypoints
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01,0.0)
        print plan

        print "executing plan"
        print
        print fraction
        rospy.sleep(5)
        return waypoints

    def show_way_points(self,points):
        for i in range(len(points)):
            marker = Marker()
            marker.header.frame_id = "world"
           # marker.header.frame_id = "world"
            marker.type = marker.SPHERE
            marker.id = i
            marker.scale.x = 0.07
            marker.scale.y = 0.07
            marker.scale.z = 0.07
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = self.cpose.position.x + points[i][0]
            marker.pose.position.y = self.cpose.position.y + points[i][1]
            marker.pose.position.z = self.cpose.position.z + points[i][2]
            self.MarkerArray.markers.append(marker)
            del marker
        current_time = time.time()
        #while not (time.time() - current_time) >=30:
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.MarkerArray)




if __name__=="__main__":
    # Waiting for rviz to launch
    #rospy.sleep(10)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("generate_trajectory",anonymous = True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #group = moveit_commander.MoveGroupCommander("arm")
    group = moveit_commander.MoveGroupCommander("arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=1)
    trajectory =  GenerateTrajectory(robot,scene,group)
    points = [[0.1,0.1,-0.2],[0.1,0.1,-0.4]]
    waypoints = trajectory.execute_trajectory(points)
    moveit_commander.roscpp_shutdown()
    trajectory.show_way_points(points)
