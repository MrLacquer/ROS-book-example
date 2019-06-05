#!/usr/bin/env python

import os, sys, rospy, tf, actionlib
import copy, math
from math import pi
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from control_msgs.msg import PointHeadAction, PointHeadGoal

from ar_track_alvar_msgs.msg import AlvarMarkers

class AR_Mover():
      def ar_callback(self, msg):
            self.marker = msg.markers
            self.m_id = self.marker[1].id

            self.pos_x = self.marker[1].pose.pose.position.x
            self.pos_y = self.marker[1].pose.pose.position.y
            self.pos_z = self.marker[1].pose.pose.position.z

            self.dist = math.sqrt(
            (self.pos_x * self.pos_x) + (self.pos_y * self.pos_y)
            )
          
            '''
            self.ori_x = self.marker.pose.pose.orientation.x
            self.ori_y = self.marker.pose.pose.orientation.y
            self.ori_z = self.marker.pose.pose.orientation.z
            self.ori_w = self.marker.pose.pose.orientation.w
            '''

            print 'id: ', self.m_id
            print 'pos: ', self.pos_x, self.pos_y, self.pos_z
            print 'dis: ', self.dist
            #print('pos: ', self.pos_x, self.pos_y, self.pos_z)
            #print('ori: ', self.ori_x, self.ori_y, self.ori_z, self.ori_w)
#
      def go_to_bin(self, args):
            
            if len(args) != 2:
                  print("usage: go_to_bin.py BIN_NUMBER")
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

      def look_at_bin(self):
            print("look at the bin")
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

            rospy.loginfo("Wating for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)

def shutdown_callback():
      print "Shutting down position controller."

if __name__ == '__main__':
      rospy.init_node('hj_marker_controll', anonymous=True)      
      os.system("rosservice call /move_base/clear_costmaps")
      
      args = rospy.myargv(argv=sys.argv)

      ar = AR_Mover()

      ar.go_to_bin(args)
      ar.look_at_bin()

      rospy.on_shutdown(shutdown_callback)
      rospy.spin()
      

      
      
  
      



