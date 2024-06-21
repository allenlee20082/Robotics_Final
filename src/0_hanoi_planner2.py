#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
from math import atan2, acos, asin, sqrt, sin, cos, pi
import rospkg

'''tower number'''
pillar_num = {'pillar1' : 0, 'pillar2' : 0, 'pillar3' : 0}

'''tower height'''
height = [0.0014+0.025, 0.0014+0.015+0.025, 0.0014+0.015+0.015+0.025]

'''define hanoi return list'''
hanoi_list = []

'''Variable for end-effector'''
EefState = 0

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_heitght = 0.025   #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#You may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0,15) as lab4 shown

'''Robot arm geometry'''
l0 = 0.06;l1 = 0.082;l2 = 0.132;l3 = 0.1664;l4 = 0.048;d4 = 0.004

'''Hanoi tower mesh file path'''
rospack = rospkg.RosPack()
FILE_PATH = rospack.get_path('myplan')+ "/mesh"
MESH_FILE_PATH = [FILE_PATH +"/tower1.stl",FILE_PATH +"/tower2.stl",FILE_PATH +"/tower3.stl"]

'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''

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
    group_names = robot.get_group_names()
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles
    self.box_name = ''

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
    # print "current pose:"
    # print current_pose.position 
    return all_close(joint_goal, current_joints, 0.01)
  
  def attach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Make sure the mesh has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def detach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.remove_attached_object(link_name, name=mesh_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_mesh(self, mesh_name, mesh_pose, file_path, size_tuple): 
    '''
    Description: 
        1. Add a mesh to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
    '''
    scene = self.scene
    mesh_pose.pose.orientation.w = 0.7071081
    mesh_pose.pose.orientation.x = 0.7071081
    mesh_pose.pose.orientation.y = 0
    mesh_pose.pose.orientation.z = 0
    #deal with orientation-definition difference btw .stl and robot_urdf
    scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  
  def add_box(self, box_name , box_pose, size_tuple):  
    '''
    Description: 
        1. Add a box to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
        3. Google scene.add_box for more details
    '''
    scene = self.scene
    scene.add_box(box_name, box_pose, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

def create_env(pathPlanObject):
  box_pose = geometry_msgs.msg.PoseStamped() 
  box_pose.header.frame_id = 'world'         
  box_pose.pose.orientation.w = 1.0          
  box_pose.pose.position.x = 0.25             # Specify x of the box
  box_pose.pose.position.y = -0.075             # Specify y of the box
  box_pose.pose.position.z = 0.1095/2          # Specify z of the box
  pathPlanObject.add_box('box_1', box_pose, (0.1, 0.00005, 0.1095)) #Specify box name, box pose, size in xyz

  # box_pose2 = geometry_msgs.msg.PoseStamped() # Set the parameter
  # box_pose2.header.frame_id = 'world'         # Put the box in 'world' frame
  # box_pose2.pose.orientation.w = 1.0          # Orieantion in quaterian
  # box_pose2.pose.position.x = 0.25             # Specify x of the box
  # box_pose2.pose.position.y = -0.075             # Specify y of the box
  # box_pose2.pose.position.z = 0.1          # Specify z of the box
  # pathPlanObject.add_box('box_2', box_pose2, (0.05, 0.05, 0.2)) #Specify box name, box pose, size in xyz

  # box_pose3 = geometry_msgs.msg.PoseStamped() # Set the parameter
  # box_pose3.header.frame_id = 'world'         # Put the box in 'world' frame
  # box_pose3.pose.orientation.w = 1.0          # Orieantion in quaterian
  # box_pose3.pose.position.x = 0.25          # Specify x of the box
  # box_pose3.pose.position.y = 0             # Specify y of the box
  # box_pose3.pose.position.z = 0.35          # Specify z of the box
  # pathPlanObject.add_box('box_3', box_pose3, (0.3, 0.5, 0.05))
  
  box_pose_bottom = geometry_msgs.msg.PoseStamped()    
  box_pose_bottom.header.frame_id = 'world'         # Put the box in 'world' frame
  box_pose_bottom.pose.orientation.w = 1.0          # Orieantion in quaterian
  box_pose_bottom.pose.position.x = 0.00             # Specify x of the box
  box_pose_bottom.pose.position.y = 0.00             # Specify y of the box
  box_pose_bottom.pose.position.z = -0.025          # Specify z of the box
  pathPlanObject.add_box('box_bottom', box_pose_bottom, (1, 1, 0.05)) #Specify box name, box pose, size in xyz
  return


def Your_IK(x,y,z): 
  global l0,l1,l2,l3,l4,d4
  
  link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040]
  '''
  Write your code here!
  x,y,z,q is in world frame (the same as link0 frame)
  The end-effector should parallel to the ground.
  '''

  theta0 = atan2(y, x)
  
  beta = pi / 2
  alpha = -link[0] - link[1] + link[5] + z - link[4] * cos(beta)
  gamma = sqrt(x*x + y*y) - link[4] * sin(beta)
  c2 = (alpha*alpha + gamma*gamma - link[2]*link[2] - link[3]*link[3]) / (2*link[2]*link[3])
  s2 = sqrt(1 - c2*c2)
  theta2 = atan2(s2, c2)

  c1 = ((link[2] + link[3]*c2)*alpha + gamma*link[3]*s2) / (link[2]*link[2] + link[3]*link[3] + 2*link[2]*link[3]*c2)
  s1 = (gamma - link[3]*c1*s2) / (link[2] + link[3]*c2)
  theta1 = atan2(s1, c1)

  theta3 = beta - theta1 - theta2

  # print(theta1, theta2, theta3)
  

  # # joint_angle = your_IK_solution
  # if(theta0 < 0):
  #   theta0 = theta0 + 0.05
  # else:
  #   theta0 = theta0 + 0.1
  
  # joint_angle=[theta0, theta1, theta2 - 0.04, theta3]

  joint_angle = [theta0, theta1, theta2, theta3]
  return joint_angle
  


def pub_EefState_to_arm():
  
  global pub_EefState, rate
  pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
  rate = rospy.Rate(100) # 100hz

def get_coor(agent):
  if agent == 1:
    return [0.25, 0.15, 0.07]
  elif agent == 2:
    return [0.25, 0, 0.07]
  else:
    return [0.25, -0.15, 0.07]
  
def get_tower_num(num):
  if num == 1:
    return 'pillar1'
  elif num == 2:
    return 'pillar2'
  else:
    return 'pillar3'

def hanoi_tower(n, source, dest, inter):
  global hanoi_list
  string = ''
  if n == 1:
    s_coor = get_coor(source)
    s_coor.append(1)
    d_coor = get_coor(dest)
    d_coor.append(0)
    i_coor = get_coor(inter)
    step_pair = [s_coor, d_coor, get_tower_num(source), get_tower_num(dest), 'tower3']
    hanoi_list.append(step_pair)
    return

  hanoi_tower(n-1, source, inter, dest)

  s_coor = get_coor(source)
  s_coor.append(1)
  d_coor = get_coor(dest)
  d_coor.append(0)
  i_coor = get_coor(inter)
  if n == 1:
    string = 'tower3'
  elif n == 2:
    string = 'tower2'
  else:
    string = 'tower1'

  step_pair = [s_coor, d_coor, get_tower_num(source), get_tower_num(dest), string]
  hanoi_list.append(step_pair)

  hanoi_tower(n-1, inter, dest, source)

  return
  
def place_tower(obj, from_place, to_place):
  global pillar_num
  list = [1, 2, 3]
  list.remove(from_place)
  list.remove(to_place)
  mesh_pose = geometry_msgs.msg.PoseStamped()
  mesh_pose.header.frame_id = 'world'
  mesh_pose.pose.position.x = 0.25
  if from_place == 1:
    pillar_num['pillar1'] = 3
    mesh_pose.pose.position.y = 0.15
    mesh_pose.pose.position.z = 0.0014
    obj.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015
    obj.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015 + 0.015
    obj.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))





  elif from_place == 2:
    pillar_num['pillar2'] = 2
    mesh_pose.pose.position.y = 0.00
    mesh_pose.pose.position.z = 0.0014
    obj.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015
    obj.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015 + 0.015
    obj.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))
    
  else:
    pillar_num['pillar3'] = 3
    mesh_pose.pose.position.y = -0.15
    mesh_pose.pose.position.z = 0.0014
    obj.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015
    obj.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose.pose.position.z = 0.0014 + 0.015 + 0.015
    obj.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))
  
  return list[0]
      

def main():
  global pub_EefState, EefState, hanoi_list, pillar_num
  path_object = MoveGroupPythonIntefaceTutorial()
  try:

    print "ctrl + z to close"
    while not rospy.is_shutdown():
      try:
        hanoi_list = []
        pillar_num['pillar1'] = 0
        pillar_num['pillar2'] = 0
        pillar_num['pillar3'] = 0
        from_input = int(raw_input("source pillar: "))
        to_input = int(raw_input("to pillar: "))
        create_env(path_object)
        inter_input = place_tower(path_object, from_input, to_input)

        pillar_num[get_tower_num(from_input)] = 3


        hanoi_tower(3, from_input, to_input, inter_input)
        # point_set = hanoi_tower(3, from_input, to_input, inter_input)
        print len(hanoi_list)
        print hanoi_list

        # extract point from point_set
        for i in range(len(hanoi_list)):
          temp_pair = hanoi_list[i]
          source_point = temp_pair[0]
          dest_point = temp_pair[1]
          source_pillar = temp_pair[2]
          dest_pillar = temp_pair[3]


          # manipulate end effectorlen(point_set)
          source_point.append(1)
          dest_point.append(0)         

          # go from source
          source_pillar = temp_pair[2]
          # print(source_pillar)
          x_input = source_point[0]
          y_input = source_point[1]
          z_input = height[pillar_num[source_pillar]-1]
          # print(pillar_num[source_pillar]-1)
          # print(z_input)
          
          path_object.joint_angles = Your_IK(x_input,y_input,z_input)
          path_object.go_to_joint_state()
          path_object.attach_mesh(temp_pair[4], 'link5')
          # print(temp_pair[4])

          # go to destination
          x_input = dest_point[0]
          y_input = dest_point[1]
          dest_pillar = temp_pair[3]
          z_input = height[pillar_num[dest_pillar]]
          path_object.joint_angles = Your_IK(x_input,y_input,z_input)
          path_object.go_to_joint_state()
          path_object.detach_mesh(temp_pair[4], 'link5')
          # print(dest_pillar)
          # print(z_input)

          # print("\n")

          if source_pillar == 'pillar1':
            pillar_num['pillar1'] -= 1
            if dest_pillar == 'pillar2':
              pillar_num['pillar2'] += 1
            else:
              pillar_num['pillar3'] += 1
          elif source_pillar == 'pillar2':
            pillar_num['pillar2'] -= 1
            if dest_pillar == 'pillar1':
              pillar_num['pillar1'] += 1
            else:
              pillar_num['pillar3'] += 1
          else:
            pillar_num['pillar3'] -= 1
            if dest_pillar == 'pillar1':
              pillar_num['pillar1'] += 1
            else:
              pillar_num['pillar2'] += 1

          # pub_EefState.publish(EefState)  #publish end-effector state
          # pub_EefState.publish(EefState)

        path_object.joint_angles = [0,-pi/2,pi/2,0]
        path_object.go_to_joint_state()
          


        
          
           

      except Exception as e:
        print(e)
        path_object.joint_angles = [0,-pi/2,pi/2,0]
        path_object.go_to_joint_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

