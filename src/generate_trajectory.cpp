/*
 * =====================================================================================
 *
 *       Filename:  generate_trajectory.cpp
 *
 *    Description:  This file consist of code for generating cartesian plan for the robot
 *
 *        Version:  1.0
 *        Created:  03/05/2016 12:47:46 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Saurabh Dixit (mn), dixits@oregonstate.edu
 *        Company:  Robotics, Oregon State University
 *
 * =====================================================================================
 */



#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);

  moveit::planning_interface::MoveGroup group("arm");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // This will print the reference frame of the robot
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  // This will print the end-effector link for this group
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  /* Following program is to generate the cartesian path
   *
   *
   *
   *
   * */
  
  
  
  
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::PoseStamped 

  geometry_msgs::Pose target_pose3 = group.getCurrentPose()->pose();
  target_pose3.position.x += 0.2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing cartesian path (%.2f%% acheived)",
        fraction * 100.0);    
  ros::shutdown();  
  return 0;

}
