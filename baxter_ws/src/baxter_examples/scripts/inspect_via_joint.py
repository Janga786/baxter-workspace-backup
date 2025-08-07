#!/usr/bin/env python

import rospy

import baxter_interface

rospy.init_node('Inspect using joint angles')

limb = baxter_interface.Limb('right')

angles = limb.joint_angles()


angles['right_s0']=0.0
angles['right_s1']=0.0
angles['right_e0']=0.0
angles['right_e1']=0.0
angles['right_w0']=0.0
angles['right_w1']=0.0
angles['right_w2']=0.0


limb.move_to_joint_positions(angles)


position_1 = {'right_s0': -0.16566992509162468, 'right_s1': -1.408577858475781, 'right_e0': 0.6197282383057071, 'right_e1': 2.4010634282376437, 'right_w0': -0.06366020269724466, 'right_w1': 0.5813787186085718, 'right_w2': -3.0518547774980305}
position_2 = {'right_s0': -1.4576652436881143, 'right_s1': 0.13614079492483047, 'right_w0': -3.0441848735586037, 'right_w1': -1.1508690861110316, 'right_w2': -2.6238741376780004, 'right_e0': 0.6066894016086811, 'right_e1': 1.6010924473554007}
position_3 = {'right_s0': -0.6093738679874806, 'right_s1': -0.02569417819708068, 'right_w0': -2.998932440315984, 'right_w1': -1.573097297976492, 'right_w2': -2.641131421541711, 'right_e0': 0.5303738574113818, 'right_e1': 1.0553787820651646}
position_4 = {'right_s0': 0.35473305719850196, 'right_s1': -0.21437381510698658, 'right_w0': -0.4640291883353377, 'right_w1': 1.8495973349928376, 'right_w2': -3.0579907006495723, 'right_e0': 0.39346607209260864, 'right_e1': 0.7842476778064178}
position_5 = {'right_s0': 0.6784030034423242, 'right_s1': -0.43756801974431425, 'right_w0': -0.7159855327455169, 'right_w1': 1.5719468123855778, 'right_w2': -3.060675167028372, 'right_e0': 0.39998549044112164, 'right_e1': 1.3625584348392188}
position_6 = {'right_s0': 1.3142380400208282, 'right_s1': -0.5522330836387489, 'right_w0': -1.092577816171386, 'right_w1': 1.2106943368385628, 'right_w2': -3.0161897241796947, 'right_e0': 0.24198546928892403, 'right_e1': 2.105772126569702}


limb.move_to_joint_positions(position_1)
limb.move_to_joint_positions(position_2)
limb.move_to_joint_positions(position_3)
limb.move_to_joint_positions(position_4)
limb.move_to_joint_positions(position_5)
limb.move_to_joint_positions(position_6)
limb.move_to_joint_positions(position_1)

limb.move_to_neutral()




quit()