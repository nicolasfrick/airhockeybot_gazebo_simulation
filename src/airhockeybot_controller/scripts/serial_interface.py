#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time

import rospy
import serial

from sensor_msgs.msg import JointState

# dict used to update joint positions
# for real bot only joint_x/y is updated
joint_states = {"joint_names": ['joint_x',
                                'joint_y',
                                'player_joint_x',
                                'player_joint_y'],
                "joint_pos":   [0.1, 0.41, 0.0, 0.0],
                "joint_vel":   [0.0, 0.0, 0.0, 0.0],
                "joint_eff":   [0.0, 0.0, 0.0, 0.0]
               }

class SerialInterface:
    """Communicate over serial."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, waittime=0.1):
        self.ser = serial.Serial(port, baudrate)
        self.waittime = waittime
        time.sleep(waittime)
        self.joint_states = joint_states
        self.joint_state_pub = rospy.Publisher("/airhockeybot/joint_states", JointState, queue_size=1)

    def check_periodic_messages(self, waittime=0.1):
        """Check for periodic messages like current x,y."""
        lines = []
        while self.ser.in_waiting:
            lines.append("{}".format(self.ser.readline().decode().strip()))
        self.get_position_from_msg(lines)

    def get_position_from_msg(self, lines):
        for m in lines:
            if m.startswith("I1"):
                # I1 x:0.00	y:0.00	x_speed:0.00	y_speed: 0.00
                sub = m[3:]

                split = sub.split('\t')

                x = float(split[0][2:])
                y = float(split[1][2:])
                speed = float(split[2][2:])
                # x = float()
                self.set_joint_states(x, y, speed)

    def send_command(self, command, waittime=0.1):
        """Send a G-Code to the robot."""
        if not command.endswith("\n"):
            command += "\n"
        self.ser.write(command.encode())
        time.sleep(waittime)
        self.check_periodic_messages()

    def set_joint_states(self, x, y, speed):
        joint_states.update({"joint_names": ['joint_x',
                                             'joint_y',
                                             'player_joint_x',
                                             'player_joint_y'],
                            "joint_pos":   [x, y, 0.0, 0.0],
                            "joint_vel":   [speed, 0.0, 0.0, 0.0],
                            "joint_eff":   [0.0, 0.0, 0.0, 0.0]
                         })
        self.publish_joint_states()

    def publish_joint_states(self):
        msg = JointState()
        msg.name     = self.joint_states["joint_names"]
        msg.position = self.joint_states["joint_pos"]
        msg.velocity = self.joint_states["joint_vel"]
        msg.effort   = self.joint_states["joint_eff"]
        self.joint_state_pub.publish(msg)

    def bot_pos_xy(self, action):
        """Move bot to action = (x pos, y pos)."""
        self.send_command("M1 {},{};".format(action[0], action[1]))

    def bot_add_pos_x(self, action):
        """Increments actual joint position."""
        x_pos = joint_states['joint_pos'][0]
        y_pos = joint_states['joint_pos'][1]
        self.send_command("M2 {},{};".format(x_pos + action, y_pos))

    def bot_add_pos_y(self, action):
        """Increments actual joint position."""
        x_pos = joint_states['joint_pos'][0]
        y_pos = joint_states['joint_pos'][1]
        self.send_command("M2 {},{};".format(x_pos, y_pos + action))

    def reset_puck_state(self,
                         x=0.901, y=0.442, z=0.775,
                         ox=0.0, oy=0.0, oz=0.0,
                         tlx=0.0, tly=0.0, tlz=0.0,
                         tax=0.0, tay=0.0, taz=0.0):
        """-Interface method- not used here."""
        pass

    def move_puck(self, twist_lin_x=0.0, twist_lin_y=0.0, twist_lin_z=0.0,
                      twist_ang_x=0.0, twist_ang_y=0.0, twist_ang_z=0.0):
        """-Interface method- not used here."""
        pass

    def player_pos_xy(self, action):
        """-Interface method- not used here."""
        pass

    def player_add_pos_xy(self, action):
        """-Interface method- not used here."""
        pass

    def player_add_pos_x(self, action):
        """-Interface method- not used here."""
        pass

    def player_add_pos_y(self, action):
        """-Interface method- not used here."""
        pass

    def get_goal(self):
        """Not implemented."""
        return False, False

if __name__ == "__main__":
    ser = SerialInterface()
    while True:
        command = input("Command: ")
        ser.send_command(command)
