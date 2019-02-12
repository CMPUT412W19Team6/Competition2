#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from geometry_msgs.msg import Twist
from smach import State, StateMachine
import smach_ros
from dynamic_reconfigure.server import Server
from demo3.cfg import Demo3Config
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, Sound
from tf.transformations import decompose_matrix, compose_matrix
from ros_numpy import numpify
from sensor_msgs.msg import Joy, LaserScan, Image
import numpy as np
import angles as angles_lib
import math
import random


START = False
POSE = [0, 0, 0, 0]
turn_direction = 1

# TODO: update POSE from callback


class WaitForButton(State):
    def __init__(self):
        State.__init__(self, outcomes=["pressed", "exit"])
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        global START
        while not START and not rospy.is_shutdown():
            self.rate.sleep()
        if rospy.is_shutdown():
            return "exit"
        else:
            return "pressed"


class FollowLine(State):
    """
    should follow the white line, until see full red
    """

    def __init__(self):
        State.__init__(self, outcomes=["see_red", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class Turn(State):
    """
    Turning a specific angle, based on Sean's example code from demo2
    """

    def __init__(self, angle=90):
        State.__init__(self, outcomes=["success", "exit", 'failure'])
        self.tb_position = None
        self.tb_rot = None
        # angle defines angle target relative to goal direction
        self.angle = angle

        # pub / sub
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)
        self.cmd_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1)

    def odom_callback(self, msg):
        tb_pose = msg.pose.pose
        __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def execute(self, userdata):
        global turn_direction
        global START
        global POSE

        if not START:
            return 'exit'
        start_pose = POSE
        if self.angle == 0:  # target is goal + 0
            goal = start_pose[1]
        elif self.angle == 90:  # target is goal + turn_direction * 90
            goal = start_pose[1] + np.pi/2 * turn_direction

        goal = angles_lib.normalize_angle(goal)

        cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)
        speed = 0.55
        rate = rospy.Rate(30)

        direction = turn_direction

        if self.angle == 0:
            direction = turn_direction * -1

        while not rospy.is_shutdown():
            cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)

            # slow down turning as we get closer to the target heading
            if cur < 0.1:
                speed = 0.15
            if cur < 0.0571:
                break
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = direction * speed
            self.cmd_pub.publish(msg)
            rate.sleep()

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        return 'success'


class DepthCount(State):
    """
    Count the number of objects based on the depth image
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class Signal1(State):
    """
    Make sound and LED lights based on count for phase 1
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class LookForSide(State):
    """
    Looking for the half red line while following the white line
    """

    def __init__(self):
        State.__init__(self, outcomes=["see_half_red", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class FindGreen(State):
    """
    Recognize the Green image shape based on NN?
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class MoveForward(State):
    """
    Move forward a bit until can't see the red
    """

    def __init__(self):
        State.__init__(self, outcomes=["no_more_red", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class CheckShape(State):
    """
    determine if the RED pattern is the shape we see in phase 3
    """

    def __init__(self):
        State.__init__(self, outcomes=["matched", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class Signal3(State):
    """
    Make a sound for phase 3
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


def joy_callback(msg):
    global START

    if msg.buttons[0] == 1:  # button A
        START = True
    elif msg.buttons[1] == 1:  # button B
        START = False


if __name__ == "__main__":
    rospy.init_node('demo3')
    rospy.Subscriber("/joy", Joy, callback=joy_callback)

    sm = StateMachine(outcomes=['success', 'failure'])
    with sm:
        StateMachine.add("Wait", WaitForButton(),
                         transitions={'pressed': 'Phase1', 'exit': 'failure'})

        StateMachine.add("Ending", FollowLine(),
                         transitions={"see_red": "Wait", "failure": "failure", "exit": "Wait"})

        # Phase 1 sub state
        phase1_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase1_sm:
            StateMachine.add("Finding1", FollowLine(), transitions={
                             "see_red": "Turn11", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn11", Turn(90), transitions={
                             "success": "Count1", "failure": "failure", "exit": "exit"})  # turn left 90 degrees
            StateMachine.add("Count1", DepthCount(), transitions={
                             "success": "MakeSignal1", "failure": "failure", "exit": "exit"})
            StateMachine.add("MakeSignal1", Signal1(), transitions={
                             "success": "Turn12", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn12", Turn(-90), transitions={
                "success": "success", "failure": "failure", "exit": "exit"})  # turn right 90 degrees
        StateMachine.add("Phase1", phase1_sm, transitions={
                         'success': 'Phase2', 'failure': 'failure', 'exit': 'Wait'})

        # Phase 2 sub state
        phase2_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase2_sm:
            StateMachine.add("Finding2", FollowLine(), transitions={
                "see_red": "LookForSide2", "failure": "failure", "exit": "exit"})
            StateMachine.add("LookForSide2", LookForSide(), transitions={
                "see_half_red": "Turn21", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn21", Turn(90), transitions={
                             "success": "FollowToEnd", "failure": "failure", "exit": "exit"})
            StateMachine.add("FollowToEnd", FollowLine(), transitions={
                             "see_red": "failure", 'failure': "FindGreenShape", "exit": "exit"})
            StateMachine.add("FindGreenShape", FindGreen(), transitions={
                "success": "TurnBack",  "failure": "failure", "exit": "exit"})
            StateMachine.add("TurnBack", Turn(180), transitions={
                "success": "MoveForward", "failure": "failure", "exit": "exit"})
            StateMachine.add("MoveForward", MoveForward(), transitions={
                "no_more_red": "Turn22", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn22", Turn(90), transitions={
                "success": "FinishingPhase2", "failure": "failure", "exit": "exit"})  # turn left 90
            StateMachine.add("FinishingPhase2", FollowLine(), transitions={
                "see_red": "success", "failure": "failure", "exit": "exit"})
        StateMachine.add("Phase2", phase2_sm, transitions={
            'success': 'Phase3', 'failure': 'failure', 'exit': 'Wait'})

        # Phase 3 sub state
        phase3_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase3_sm:
            StateMachine.add("Finding3", FollowLine(), transitions={
                "see_red": "LookForSide31", "failure": "failure", "exit": "exit"})
            StateMachine.add("LookForSide31", LookForSide(), transitions={
                "see_half_red": "Turn31", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn31", Turn(90), transitions={
                             "success": "CheckShape", "failure": "failure", "exit": "exit"})  # turn left 90
            StateMachine.add("CheckShape", CheckShape(), transitions={
                             "matched": "Signal3", "failure": "TurnRight", "exit": "exit"})
            StateMachine.add("Signal3", Signal3(), transitions={
                             "success": "TurnRightMoveOn", "failure": "failure", "exit": "exit"})
            StateMachine.add("TurnRight", Turn(-90), transitions={
                             "success": "LookForSide31", "failure": "failure", "exit": "exit"})
            StateMachine.add("TurnRightMoveOn", Turn(-90), transitions={
                             "success": "success", "failure": "failure", "exit": "exit"})

        StateMachine.add("Phase3", phase3_sm, transitions={
            'success': 'Ending', 'failure': 'failure', 'exit': 'Wait'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
