#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkState

class _HitchCtrlr():
    def __init__(self):
        """Initialize this _HitchCtrlr"""

        rospy.init_node("hitch_controller")

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency",
                                             self._DEF_PUB_FREQ))
            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publishing frequency is invalid. "
                          "The default frequency will be used instead.")
            pub_freq = self._DEF_PUB_FREQ


        self._sleep_timer = rospy.Rate(pub_freq)

        self._hitch_cmd_lock = threading.Lock()
        self._hitch_height = 0;

        # Publishers and subscribers
        self._sim_hitch_cmd_pub = \
            rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)

        self._hitch_cmd_sub = \
            rospy.Subscriber("/hitch_cmd", Point,
                             self._hitch_cmd_cb, queue_size=1)


    def spin(self):
        """Control the hitch"""
        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            if delta_t > 0.0:
                with self._hitch_cmd_lock:
                    # if self._estop == True: # Abruptly stop when estopped (simulate engine shut off)
                    #     steer_ang = 0
                    #     steer_ang_vel = 0
                    #     speed = 0
                    #     accel = 0
                    # elif self._cmd_activate == False: # Slow tractor to a stop (simulate disactivate command)
                    #     steer_ang = 0
                    #     steer_ang_vel = self._steer_ang_vel
                    #     speed = 0
                    #     accel = self._accel
                    # else: # If not estopped or disactivated, run tractor
                    hitch_height = self._hitch_height
                    hitch_cmd = self._ctrl_hitch(hitch_height)
                    # Publish the hitch joint command
                    self._sim_hitch_cmd_pub.publish(hitch_cmd)

            self._sleep_timer.sleep()

    def _hitch_cmd_cb(self, hitch_cmd):
        with self._hitch_cmd_lock:
            self._hitch_height = hitch_cmd.z

    def _ctrl_hitch(self, hitch_height):
        hitch_cmd = LinkState()
        hitch_cmd.link_name = "tractor_sim::hitch"
        hitch_cmd.pose.position.z = hitch_height
        hitch_cmd.reference_frame = "base_link"

        return hitch_cmd

    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.

if __name__ == '__main__':
    hitch_ctlr = _HitchCtrlr()
    hitch_ctlr.spin()























"""easy_install pygazebo"""






# import trollius
# from trollius import From
#
# import pygazebo
# import pygazebo.msg.joint_cmd_pb2
#
# @trollius.coroutine
# def publish_loop():
#     manager = yield From(pygazebo.connect())
#
#     publisher = yield From(
#         manager.advertise('/gazebo/default/model/joint_cmd',
#                           'gazebo.msgs.JointCmd'))
#
#     message = pygazebo.msg.joint_cmd_pb2.JointCmd()
#     message.name = 'tractor_sim::hitch'
#     message.axis = 0
#     message.force = -200.0
#
#     while True:
#         yield From(publisher.publish(message))
#         yield From(trollius.sleep(1.0))
#
# loop = trollius.get_event_loop()
# loop.run_until_complete(publish_loop())

"""
get point of base link
get point of hitch
do trig to get height
send force commands to link proportional to distance

Subscribed to:
hitch height (gravl)
base link pose (sim)
hitch pose (sim)





# How to apply forces (need to convert from c++)

gazebo::physics::LinkPtr link=  _model->GetChildLink("Path::to::Link");
link->AddForce(gazebo::math::Vector3(x, y, z)); //for global force
link->AddRelativeForce(math::Vector3(x, y, z)); // for relative force depends on actual pose and angular


# How to get states
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState # For getting information about link states

model_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
rospy.wait_for_service('/gazebo/get_link_state')
print "Link 7 Pose:"    , endl , model_info_prox( "lbr4_allegro::lbr4_7_link" , "world" )
"""
