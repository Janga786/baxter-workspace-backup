#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


rospy.init_node('baxter_joint_angle_display')


limb = baxter_interface.Limb('right')


angle = {'right_s0': -0.16566992509162468, 'right_s1': -1.408577858475781, 'right_e0': 0.6197282383057071, 'right_e1': 2.4010634282376437, 'right_w0': -0.06366020269724466, 'right_w1': 0.5813787186085718, 'right_w2': -3.0518547774980305}
print( angle )

limb.move_to_joint_position(angle)

SolvePositionIK(
    x = 0.0
    y = 0.0
    z = 0.0
)