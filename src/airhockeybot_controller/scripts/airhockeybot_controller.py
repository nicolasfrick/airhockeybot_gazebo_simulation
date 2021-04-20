#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import random

import rospy
import pynput.keyboard             as kbd

from   serial_readin               import SerialReadIn
from   serial_interface            import SerialInterface
from   gazebo_interface            import GazeboInterface
from   goal_state                  import GoalState

# ros control loop frequency
F_CTRL_LOOP = 100
# stepwidth to increment joint position with
MOVE = 0.2  # meter
# FACTOR for puk speed (*MOVE)
FACTOR = 8

# keyboard dictionary maps to action COMMANDS
COMMANDS = {
    # player bat/ puk orthogonal (up, down, right, left)
    "8": ("player_x", -MOVE),
    "2": ("player_x", MOVE),
    "4": ("player_y", -MOVE),
    "6": ("player_y", MOVE),
    # player bat/ puk diagonal (up left, up right, down right, down left)
    "9": ("player_xy", -MOVE, MOVE),
    "7": ("player_xy", -MOVE, -MOVE),
    "1": ("player_xy", MOVE, -MOVE),
    "3": ("player_xy", MOVE, MOVE),
    # robot (up, down, right, left)
    "w": ("bot_x", MOVE),
    "x": ("bot_x", -MOVE),
    "a": ("bot_y", MOVE),
    "d": ("bot_y", -MOVE),
    # random bot, init puk, switch player/puk control
    "0": "0",
    "p": "p",
    "r": "r",
}


class AirhockeybotController(object):
    """Connects action node and simulation or serial interface."""

    def __init__(self, usb_port, baud_rate, remote_ctrl):
        """sys.argv remote_ctrl: use keyboard remote control."""
        rospy.init_node("airhockeybot_controller")
        self.use_sim = rospy.get_param("/airhockeybot_controller/use_simulation")
        rospy.loginfo("Controller: Initialized")
        # control instance (Gazebo interface or serial com)
        self.ctrl = None
        # keyboard controller
        self.remote = None
        # random control for bot
        self.rand = False
        # puk/ player ctrl
        self.puk_ctrl = False
        # goal state class
        self.goal_state = GoalState()
        # blocking point variable
        self.action = None
        # Subscriber
        # rospy.Subscriber("/airhockeybot/blocking_point",
        #                  BlockingPoint, self.update_blocking_point)

        # Assign controller instance
        if self.use_sim:
            rospy.loginfo("Controller: make gazebo interface")
            self.ctrl = GazeboInterface()
        else:
            rospy.loginfo("Controller: make serial interface")
            self.ctrl = SerialInterface(usb_port, baud_rate)
        if remote_ctrl:
            rospy.loginfo("Controller: make remote serialReadIn")
            self.remote = SerialReadIn()
        # rospy.loginfo("Controller: Play loop starting")
        self.play()

    def update_blocking_point(self, data):
        """Blocking point callback."""
        self.action = (data.x, data.y)

    def play(self):
        rcmd = None
        cnt = 0
        rate = rospy.Rate(F_CTRL_LOOP)
        # ROS loop
        while not rospy.is_shutdown():
            # rospy.loginfo("Play loop")
            # assign keyboard command
            if self.remote is not None:
                rcmd, _ = self.get_remote_ctrl_cmd()
                # invert random bot flag
                if rcmd   == 'r':
                    self.rand = not self.rand
                # reset puk pose
                elif rcmd == 'p':
                    self.ctrl.reset_puck_state()
                # switch player/puk control
                elif rcmd == '0':
                    self.puk_ctrl = not self.puk_ctrl

            # execute command
            if self.ctrl is not None:
                # random bot command
                if cnt % 50 == 0 and self.rand:
                    self.ctrl.bot_pos_xy((random.random()*1000, random.random()*1000))
                # keyboard command
                if rcmd is not None:
                    if not self.puk_ctrl:
                        if rcmd[0]   == 'player_x':
                            self.ctrl.player_add_pos_x(rcmd[1])
                        elif rcmd[0] == 'player_y':
                            self.ctrl.player_add_pos_y(rcmd[1])
                        elif rcmd[0] == 'player_xy':
                            self.ctrl.player_add_pos_xy((rcmd[1], rcmd[2]))
                    else:
                        if rcmd[0]   == 'player_x':
                            self.ctrl.move_puck(twist_lin_x=rcmd[1]*FACTOR)
                        elif rcmd[0] == 'player_y':
                            self.ctrl.move_puck(twist_lin_y=rcmd[1]*FACTOR)
                        elif rcmd[0] == 'player_xy':
                            self.ctrl.move_puck(twist_lin_x=rcmd[1]*FACTOR, twist_lin_y=rcmd[2]*FACTOR)
                    if rcmd[0]       == 'bot_x':
                        self.ctrl.bot_add_pos_x(rcmd[1])
                    elif rcmd[0]     == 'bot_y':
                        self.ctrl.bot_add_pos_y(rcmd[1])
                # Blocking Point command
                # rospy.loginfo("Marker")
                if self.action is not None:
                    # rospy.loginfo("Trying to execute action")
                    self.ctrl.bot_pos_xy(self.action)
                    self.action = None
                    
            # publish joint states from serial interface
            self.ctrl.check_periodic_messages()
            # update goal states
            self.update_goals()
            cnt += 1
            rate.sleep()

    def update_goals(self):
        reset_state = False
        bot_goal, player_goal = self.ctrl.get_goal()
        self.goal_state.update_goal_state(bot_goal, player_goal, reset_state)

    def get_remote_ctrl_cmd(self):
        """Assign keyboard key to action command (see COMMANDS dict)."""
        cmd = key = None
        if self.remote.input_queue.qsize() > 0:
            key = self.remote.input_queue.get()
            try:
                # arrows
                cmd = COMMANDS[key]
            except (KeyError, AttributeError):
                pass
            try:
                # chars
                cmd = COMMANDS[key.char]
            except (KeyError, AttributeError):
                pass
        return cmd, key


if __name__ == "__main__":
    ac = None
    try:
        ac = AirhockeybotController(sys.argv[1], sys.argv[2], sys.argv[3])
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException")
