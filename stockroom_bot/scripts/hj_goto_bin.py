#!/usr/bin/env python
import os, sys, rospy, tf, actionlib
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from control_msgs.msg import PointHeadAction, PointHeadGoal

if __name__ == '__main__':
  os.system("rosservice call /move_base/clear_costmaps")
  rospy.init_node('go_to_bin')
  args = rospy.myargv(argv=sys.argv)
  if len(args) != 2:
    print "usage: go_to_bin.py BIN_NUMBER"
    sys.exit(1)
  bin_number = int(args[1])
  move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  move_base.wait_for_server()
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = 'map'
  goal.target_pose.pose.position.x = 0.5 * (bin_number % 6) - 1.5;
  goal.target_pose.pose.position.y = 1.1 * (bin_number / 6) - 0.55;
  if bin_number >= 6:
    yaw = 1.57
  else:
    yaw = -1.57
  orient = Quaternion(*quaternion_from_euler(0, 0, yaw))
  goal.target_pose.pose.orientation = orient
  move_base.send_goal(goal)
  move_base.wait_for_result()

  rospy.sleep(2)
  
  print "look at the bin"
  head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
  head_client.wait_for_server()
  goal = PointHeadGoal()
  goal.target.header.stamp = rospy.Time.now()
  goal.target.header.frame_id = "base_link"
  goal.target.point.x = 0.7
  goal.target.point.y = 0
  goal.target.point.z = 0.4
  goal.min_duration = rospy.Duration(1.0)
  head_client.send_goal(goal)
  head_client.wait_for_result()

