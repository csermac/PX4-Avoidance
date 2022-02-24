#!/usr/bin/env python3


from mavros_msgs.msg import ParamValue, State
# from mavros_msgs.msg import Thrust
from mavros_msgs.srv import SetMode
from gazebo_msgs.srv import GetModelState
import rospy
from geometry_msgs.msg import TwistStamped
from px4_modules.mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
#from std_msgs.msg import Header
from std_msgs.msg import Empty
from threading import Thread
#import sys
# from px4_tests.srv import Landing, LandingResponse
from px4_tests.srv import TakeOff, TakeOffResponse
#from px4_tests.srv import Landing, LandingResponse
# using numpy for angles manipulation
import tf

# from camera_link_repub import LinkRepublisher
#from geometry_msgs.msg import Quaternion, Vector3
#from six.moves import xrange
#from tf.transformations import quaternion_from_euler
#from mavros import command

import numpy as np

#import mavros.command as mrc

PKG = 'px4_test'


class ProvaPers(MavrosTestCommon):
    """
    Tests flying in offboard control by sending attitude and thrust setpoints
    via MAVROS.

    For the test to be successful it needs to cross a certain boundary in time.
    """

    def __init__(self):
        super(ProvaPers, self).setUp()

        # self.rep = LinkRepublisher()
        # self.camera_link_thread = Thread(target=self.rep.camera_link, args=())
        # self.camera_link_thread.daemon = True
        # # self.rep.camera_link()
        # self.camera_link_thread.start()

        # definition of rate value separate to use it later in the code
        self.rate_value = 100
        self.rate = rospy.Rate(self.rate_value)  # Hz

        # used to get the transformation between camera_link and base_link
        self.tf_listener = tf.TransformListener()

        # the velocity message used by the drone
        self.vel = TwistStamped()
        self.vel.header.frame_id = 'base_link'

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        # velocity publisher. its callback sends to the threaded function send_vel
        self.vel_subscriber = rospy.Subscriber('/mavros/vel_ctl', TwistStamped, self.vel_sub_stamped_callback, queue_size=1)
        # mavros builtin service that provides many infos
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=1)
        self.vel_thread = Thread(target=self.send_vel, args=())

        #self.land = rospy.Service('/mavros/landing', Landing, self.land_srv_callback)
        # dumb takeoff service
        self.takeoff = rospy.Service('/mavros/takeoff', TakeOff, self.takeoff_srv_callback)
        # connect to takeoff service through a proxy
        self.takeoff_srv = rospy.ServiceProxy('/mavros/takeoff', TakeOff)

        # change mode through service instead of using superclass MavrosTestCommon and its filthy methods
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # send setpoints in seperate thread to better prevent failsafe
        self.vel_thread.daemon = True
        self.vel_thread.start()

        # formerly present in the mission launcher
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.log_topic_vars()

        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1 << 2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)

        self.base_mode = 0
        self.custom_mode = 'OFFBOARD'
        self.set_mode_srv(self.base_mode, self.custom_mode)
        rospy.loginfo("Armed through service")

    def vel_sub_stamped_callback(self, velocity:TwistStamped):

        if self.state.armed is False:
            self.set_arm(True, 5)
        # simply setting velocity will automatically change since it is constantly published
        out_vel, out_rot = self.transform_twist(velocity.twist.angular, velocity.twist.linear)

        self.vel.twist.linear.x = out_vel[0]
        self.vel.twist.linear.y = out_vel[1]
        self.vel.twist.linear.z = out_vel[2]
        self.vel.twist.angular = out_rot
        print("assigned rotated values to vel")
        rospy.loginfo("Assigned rotated velocity values")

    def transform_twist(self, twist_ang, twist_vel):
        map_to_base_trans, map_to_base_rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time.now())
        rospy.loginfo(f"obtained transformatioon: {map_to_base_rot}, {map_to_base_trans} ")
        angles = tf.transformations.euler_from_quaternion(map_to_base_rot)

        out_vel = [twist_vel.x * np.cos(angles[2]) - twist_vel.y * np.sin(angles[2]),
                   twist_vel.x * np.sin(angles[2]) + twist_vel.y * np.cos(angles[2]),
                   twist_vel.z]

        out_rot = twist_ang

        return out_vel, out_rot

    def tearDown(self):
        super(ProvaPers, self).tearDown()

    # removed due to discover of built in service for takeoff and landing
    # def land_srv_callback(self, message):
    #     print("Hola tierra")
    #     LandingResponse().result = True
    #     return LandingResponse()

    def takeoff_srv_callback(self, message):
        """
            Once the controller is implemented the takeoff will be similar to
            what the builtin service does, that is going to a chosen position
            and keep it.
        """
        # check for arming state, just in case
        if self.state.armed is False:
            self.set_arm(True, 5)

        self.vel.twist.linear.z = 1

        # set altitude to reach for takeoff
        # might be put into the service as a message instead of the empty request now used
        altitude = 1.0
        # the division returns [s], so it has to be multiplied by the frequency
        wait_for = (altitude / self.vel.twist.linear.z) * self.rate_value

        # reset i from previous usages
        i = 0
        # manual duration because it takes some time to start moving
        for i in np.arange(wait_for * 6):
            if i % 10 == 0:
                print("waited for ", i / 10,"seconds")
            # failsafe method has to be added later, for now going at 0,5m/s
            # for 2 sec is going to be trusted
            self.rate.sleep()

        rospy.loginfo("takeoff succeeded, stopping and hovering. waited for %s second to reach position", i)

        # rezero the vertical speed once finished
        self.vel.twist.linear.z = 0
        TakeOffResponse().result = True
        return TakeOffResponse()

    # Helper methods

    def send_vel(self):

        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0

        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':

    rospy.init_node('vel_control', anonymous=False)

    ps = ProvaPers()

    ps.takeoff_srv(Empty)
    sleep = rospy.Rate(2)

    # while not rospy.is_shutdown():
    #     position = ps.get_model_state('iris','iris::base_link')
    #     print("here comes the position: ", position)
    #     sleep.sleep()

    rospy.spin()

    #ps.test_velctl()

    # al posto di questo posso provare a scrivere una funzione usando i metodi definiti sopra
    #rostest.rosrun(PKG, 'prova_pers', ProvaPers)
