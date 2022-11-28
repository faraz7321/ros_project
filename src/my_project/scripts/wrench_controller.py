#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import print_function
import os
import time
import sys
import select
import termios
import tty
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Wrench, Accel, Vector3


class KeyBoardVehicleTeleop:
    def __init__(self):
        # Class Variables
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 1
        self.force = Vector3(0, 0, 0)
        self.torque = Vector3(0, 0, 0)
        self.force_increment = 100
        self.force_limit = 2000
        self.torque_increment = 100
        self.torque_limit = 2000
        # User Interface
        self.msg = """
    Control Your Vehicle!
    ---------------------------
    Moving around:
        W/S: X-Axis
        A/D: Y-Axis
        X/Z: Z-Axis

        Q/E: Yaw
        I/K: Pitch
        J/L: Roll

    Slow / Fast: 1 / 2

    CTRL-C to quit
            """

        self._msg_type = 'wrench'
        # Name Publisher topics accordingly

        self._output_pub = rospy.Publisher(
            'thruster_manager/input', Wrench, queue_size=1)
        print(self.msg)

        # Ros Spin
        rate = rospy.Rate(10)  # 50hz
        while not rospy.is_shutdown():
            rate.sleep()
            self._parse_keyboard()

    # Every spin this function will return the key being pressed
    # Only works for one key per spin currently, thus limited control exploring alternative methods
    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key

    def _speed_windup(self, speed, increment, limit, reverse):
        if reverse == True:
            speed -= increment * self.speed
            if speed < -limit * self.speed:
                speed = -limit * self.speed
        else:
            speed += increment * self.speed
            if speed > limit * self.speed:
                speed = limit * self.speed

        return speed

    def _parse_keyboard(self):
        # Save key peing pressed
        key_press = self._get_key()

        # Set Vehicle Speed #
        if key_press == "1":
            self.speed = 1
        if key_press == "2":
            self.speed = 2

        cmd = Wrench()

        # If a key is pressed assign relevent linear / angular vel
        if key_press != '':
            # Linear velocities:
            # Forward
            if key_press == "w":
                self.force.x = self._speed_windup(
                    self.force.x, self.force_increment, self.force_limit, False)
            # Backwards
            if key_press == "s":
                self.force.x = self._speed_windup(
                    self.force.x, self.force_increment, self.force_limit, True)
            # Left
            if key_press == "a":
                self.force.y = self._speed_windup(
                    self.force.y, self.force_increment, self.force_limit, False)
            # Right
            if key_press == "d":
                self.force.y = self._speed_windup(
                    self.force.y, self.force_increment, self.force_limit, True)
            # Up
            if key_press == "x":
                self.force.z = self._speed_windup(
                    self.force.z, self.force_increment, self.force_limit*0.5, False)
            # Down
            if key_press == "z":
                self.force.z = self._speed_windup(
                    self.force.z, self.force_increment, self.force_limit*0.5, True)

            # Angular Velocities
            # Roll Left
            if key_press == "j":
                self.torque.x = self._speed_windup(
                    self.torque.x, self.torque_increment, self.torque_limit, True)
            # Roll Right
            if key_press == "l":
                self.torque.x = self._speed_windup(
                    self.torque.x, self.torque_increment, self.torque_limit, False)
            # Pitch Down
            if key_press == "i":
                self.torque.y = self._speed_windup(
                    self.torque.y, self.torque_increment, self.torque_limit, False)
            # Pitch Up
            if key_press == "k":
                self.torque.y = self._speed_windup(
                    self.torque.y, self.torque_increment, self.torque_limit, True)
            # Yaw Left
            if key_press == "q":
                self.torque.z = self._speed_windup(
                    self.torque.z, self.torque_increment, self.torque_limit, False)
            # Yaw Right
            if key_press == "e":
                self.torque.z = self._speed_windup(
                    self.torque.z, self.torque_increment, self.torque_limit, True)

        else:
            # If no button is pressed reset velocities to 0
            self.force = Vector3(0, 0, 0)
            self.torque = Vector3(0, 0, 0)

        # Store velocity message into Twist format
        cmd.force = self.force
        cmd.torque = self.torque

        # If ctrl+c kill node
        if (key_press == '\x03'):
            rospy.loginfo('Keyboard Interrupt Pressed')
            rospy.loginfo('Shutting down [%s] node' % node_name)

            # Set twists to 0
            cmd.force = Vector3(0, 0, 0)
            cmd.torque = Vector3(0, 0, 0)
            self._output_pub.publish(cmd)

            exit(-1)

        # Publish message
        self._output_pub.publish(cmd)


if __name__ == '__main__':

    # Wait for 5 seconds, so the instructions are the last thing to print in terminal
    time.sleep(5)
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    teleop = KeyBoardVehicleTeleop()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.loginfo('Shutting down [%s] node' % node_name)
