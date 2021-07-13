#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonInterfaceTutorialCustom(object):
  """
  MoveGroupPythonInterfaceTutorialCustom
  """

  def __init__(self):
    super(MoveGroupPythonInterfaceTutorialCustom, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial_custom', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = 'panda_arm'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_pub = rospy.Publisher(
      "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
      )

    # Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print '========== Planning frame: {}'.format(planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print '========== End effector link: {}'.format(eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print '========== Available Planning Groups: {}'.format(group_names)

    # Sometimes for debugging it is useful to print the entire state of the robot:
    print '========== Printing robot state'
    print robot.get_current_state()
    print ""

    # Misc Variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_pub = display_trajectory_pub
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    """
    Planning to a Joint Goal
    """
    # The Pandas zero configuration is at a singularity so the first thing we
    # want to do is move it to a slightly better configuration.

    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

  def go_to_pose_goal(self):
    """
    Planning to a Pose Goal
    """
    # We can plan a motion for this group to a desired pose for the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    self.move_group.set_pose_target(pose_goal)

    # Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there 
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

  def plan_cartesian_path(self, scale=1):
    """
    Cartesian Paths
    """
    # You can plan a Cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through. If executing interactively in a Python
    # shell, set scale = 1.0.

    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1 # First move up (z)
    wpose.position.y += scale * 0.2 # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # Second move forward/backwards (x)
    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Third move sideways (y)
    wpose.position.y -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    # Note: We are just planning, not asking move_group to actually move the robot
    # yet:
    return plan, fraction

  def display_trajectory(self, plan):
    """
    Displaying a Trajectory
    """
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_pub.publish(display_trajectory)

  def execute_plan(self, plan):
    """
    Executing a Plan
    """
    self.move_group.execute(plan, wait=True)

  def add_box(self, timeout=4):
    """
    Adding Objects to the Planning Scene
    """
    # First, we will create a box in the planning scene at the location of the left
    # finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = 'panda_leftfinger'
    box_pose.pose.orientation.w = 1.0
    # slightly above the end effector
    box_pose.pose.position.z = 0.07
    self.box_name = 'box'
    self.scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def wait_for_state_update(
    self, box_is_known=False, box_is_attached=False, timeout=4):
    """
    Ensuring Collision Updates Are Receieved
    """
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def attach_box(self, timeout=4):
    """
    Attaching Objects to the Robot
    """
    # Next, we will attach the box to the Panda wrist. Manipulating objects
    # requires the robot be able to touch them without the planning scene reporting
    # the contact as a collision. By adding link names to the touch_links array, we
    # are telling the planning scene to ignore collisions between those links and
    # the box. For the Panda robot, we set grasping_group = 'hand'. If you are
    # using a different robot, you should change this value to the name of your end
    # effector group name.
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    return  self.wait_for_state_update(box_is_attached=True, box_is_known=False,
    timeout=timeout)

  def detach_box(self, timeout=4):
    """
    Detaching Objects from the Robot
    """
    self.scene.remove_attached_object(self.eef_link, self.box_name)

    return  self.wait_for_state_update(box_is_attached=False, box_is_known=True,
    timeout=timeout)

  def remove_box(self, timeout=4):
    """
    Removing Objects from the Planning Scene
    """
    # Note: The object must be detached before we can remove it from the world
    self.scene.remove_world_object(self.box_name)

    return  self.wait_for_state_update(
      box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
  try:
    print "Setting up the moveit_commander ..."
    tutorial = MoveGroupPythonInterfaceTutorialCustom()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    tutorial.detach_box()

    print "============ Press `Enter` to remove the box from the planning scene ..."
    raw_input()
    tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    print 'ROS'
    return
  except KeyboardInterrupt:
    print 'KEYBOARD INTERRRUPTED'
    return
  
if __name__ == "__main__":
  main()
