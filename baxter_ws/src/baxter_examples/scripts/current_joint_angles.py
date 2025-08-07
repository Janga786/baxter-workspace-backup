#!/usr/bin/env python

import rospy

import baxter_interface

rospy.init_node('baxter_joint_angle_display')

limbR = baxter_interface.Limb('right')
limbL = baxter_interface.Limb('left')

anglesR = limbR.joint_angles()
anglesL = limbL.joint_angles()

print( "Right Angles: ", anglesR )
# print( "Left Angles: ", anglesL )



