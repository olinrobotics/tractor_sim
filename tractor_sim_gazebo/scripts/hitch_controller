#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

class _HitchCtrlr:
    def __init__(self):
        """Initialize this _HitchCtrlr"""

        rospy.init_node("hitch_controller")

        # Publishers and subscribers
        self._base_link_sub = \
            rospy.Subscriber("/gazebo/tractor_sim_base_link", Pose,
                             self._base_link_cb, queue_size=1)
        self._hitch_sub = \
            rospy.Subscriber("/gazebo/tractor_sim_hitch", Pose,
                             self._hitch_cb, queue_size=1)
        self._hitch_cmd_sub = \
            rospy.Subscriber("/hitch_cmd", Point,
                             self._hitch_cmd_cb, queue_size=1)
        self._hitch_vel_pub = \
            rospy.Publisher("/hitch_velocity", Float32, queue_size=1)

        self._pub_freq = 100;
        self._sleep_timer = rospy.Rate(self._pub_freq)

        #TODO: Make height more accurate (use trig + full msg, not just z)
        self._base_link_pose = 0;
        self._hitch_pose = 0;
        self._hitch_cmd = 0;


    def spin(self):
        """Control the hitch"""
        while not rospy.is_shutdown():
            hitch_vel = \
                self._ctrl_hitch(self._hitch_cmd, self._hitch_pose,
                                 self._base_link_pose)

            # Publish the hitch force command
            self._hitch_vel_pub.publish(hitch_vel)

            self._sleep_timer.sleep()



    def _base_link_cb(self, base_link_pose):
        self._base_link_pose = base_link_pose.position.z

    def _hitch_cb(self, hitch_pose):
        self._hitch_pose = hitch_pose.position.z

    def _hitch_cmd_cb(self, hitch_cmd):
        self._hitch_cmd = hitch_cmd.x

    def _ctrl_hitch(self, hitch_cmd, hitch_pose, base_link_pose):

        curr_height = hitch_pose - base_link_pose

        hitch_velocity = Float32()
        hitch_velocity.data = (hitch_cmd-curr_height)*-2
        return hitch_velocity


if __name__ == '__main__':
    hitch_ctlr = _HitchCtrlr()
    hitch_ctlr.spin()
