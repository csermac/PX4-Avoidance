#!/usr/bin/env python3

from __future__ import print_function

# from test_cmd_vel_topic_stamped import ProvaPers

import threading


# import roslib
#from roslib import load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
# from mavros_msgs.msg import AttitudeTarget

import sys
import select
import termios
import tty

# NOTE: for some reason,still unknown, the drone moves holding down the keys.

msg = """
Reading from the keyboard  and Publishing to TwistStamped!


For Holonomic mode (strafing), hold down the shift key:
---------------------------
        k
   h         l
        j

t : up (+z)
b : down (-z)

[ to increase speed counterclock-wise
] to increase speed clock-wise

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {

    # vim-like for movement in cross-directions
    'h':(-1,0,0,0),
    'j':(0,-1,0,0),
    'k':(0,1,0,0),
    'l':(1,0,0,0),

    'H':(0,0,0,1),
    'L':(0,0,0,-1),

    't':(0,0,1,0),
    'b':(0,0,-1,0),

    #ROTATION CONSIDERIG THAT A NEGATIVE NUMBER IS CLOCK-WISE, CONTROLLANDUM
    #']':(0,0,0,-1),
    #'[':(0,0,0,1),
}

speedBindings = {
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        #self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.publisher = rospy.Publisher('/mavros/vel_ctl', TwistStamped, queue_size=1)
        #self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        #self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = TwistStamped()
        twist.header = Header()
        twist.header.frame_id = "base_link"
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.twist.linear.x = self.x * self.speed
            twist.twist.linear.y = self.y * self.speed
            twist.twist.linear.z = self.z * self.speed
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.twist.linear.x = 0
        twist.twist.linear.y = 0
        twist.twist.linear.z = 0
        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        twist.twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    # ps = ProvaPers()
    # ps.takeoff_srv_callback(Empty)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
