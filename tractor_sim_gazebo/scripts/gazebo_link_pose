#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose:
  link_name = ''
  link_pose = Pose()
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]
    except ValueError:
      pass

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp_hitch = GazeboLinkPose('tractor_sim::hitch')
    gp_base_link = GazeboLinkPose('tractor_sim::base_link')
    publish_rate = 100

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp_hitch.pose_pub.publish(gp_hitch.link_pose)
      gp_base_link.pose_pub.publish(gp_base_link.link_pose)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
