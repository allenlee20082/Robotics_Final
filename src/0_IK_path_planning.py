#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    # print "============ Planning frame: %s" % planning_frame

    # move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])

    group_names = robot.get_group_names()


    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles

  def go_to_joint_state(self):
    
    move_group = self.move_group
    joint_angles = self.joint_angles

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = joint_angles[0]
    joint_goal[1] = joint_angles[1]
    joint_goal[2] = joint_angles[2]
    joint_goal[3] = joint_angles[3]

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print ("current pose:")
    # print (current_pose.position) 
    print ("x: %.5f" %current_pose.position.x)
    print ("y: %.5f" %current_pose.position.y)
    print ("z: %.5f" %current_pose.position.z)

    current_rpy = self.move_group.get_current_rpy('link5')
    print ("rol: %.5f" %current_rpy[0])
    print ("pit: %.5f" %current_rpy[1])
    print ("yaw: %.5f" %current_rpy[2])
    print ("")
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x,y,z,q): 
  link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040]
  '''
  Write your code here!
  x,y,z,q is in world frame (the same as link0 frame)
  The end-effector should parallel to the ground.
  '''

  theta0 = atan2(y, x)
  
  beta = q
  alpha = -link[0] - link[1] + link[5] + z - link[4] * cos(beta)
  gamma = sqrt(x*x + y*y) - link[4] * sin(beta)
  c2 = (alpha*alpha + gamma*gamma - link[2]*link[2] - link[3]*link[3]) / (2*link[2]*link[3])
  s2 = sqrt(1 - c2*c2)
  theta2 = atan2(s2, c2)

  c1 = ((link[2] + link[3]*c2)*alpha + gamma*link[3]*s2) / (link[2]*link[2] + link[3]*link[3] + 2*link[2]*link[3]*c2)
  s1 = (gamma - link[3]*c1*s2) / (link[2] + link[3]*c2)
  theta1 = atan2(s1, c1)

  theta3 = beta - theta1 - theta2

  print(theta1, theta2, theta3)
  

  # joint_angle = your_IK_solution
  if(theta0 < 0):
    theta0 = theta0
  else:
    theta0 = theta0
  
  joint_angle=[theta0, theta1, theta2 , theta3]
  return joint_angle



def main():
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print "ctrl + z to close"
    while not rospy.is_shutdown():
  
        try:
          x_input=float(raw_input("x:  "))
          y_input=float(raw_input("y:  "))
          z_input=float(raw_input("z:  "))
          q_input=float(raw_input("q:  "))

          path_object.joint_angles = Your_IK(x_input,y_input,z_input,q_input)
          '''
          You just need to solve IK of a point, path planning will automatically be taken.  
          '''
          path_object.go_to_joint_state()

        except Exception as e:
          '''go back to home if weird input'''
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()
          print(e)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

