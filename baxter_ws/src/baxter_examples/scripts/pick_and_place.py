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

limb.set_joint_position_speed(0.25)
limb.move_to_joint_positions(angles)


position_1 = {'right_s0': 0.8160777791550401, 'right_s1': -1.0592137340348782, 'right_w0': 0.10507768397015084, 'right_w1': 1.7985924737956476, 'right_w2': 0.501228222441559, 'right_e0': -0.2247281854252131, 'right_e1': 0.8145437983671547}
position_2 = {'right_s0': 0.9234564343070191, 'right_s1': -0.4437039428958559, 'right_w0': 0.15838351634916897, 'right_w1': 0.42721364942608775, 'right_w2': 0.5250049246537829, 'right_e0': -0.2339320701525256, 'right_e1': 1.4281361135213202}
gripper = baxter_interface.Gripper('right')
position_3 = {'right_s0': 0.9951700361406621, 'right_s1': 0.06596117387907278, 'right_w0': 0.03911651009107805, 'right_w1': 1.1765632643081123, 'right_w2': -0.03796602450016399, 'right_e0': 0.0007669903939427069, 'right_e1': 0.2024854640008746}

position_4 = {'right_s0': 1.1125195664138963, 'right_s1': -0.7125340759727747, 'right_w0': 0.11581554948534874, 'right_w1': 1.1777137498990264, 'right_w2': 0.960271973216269, 'right_e0': -0.1250194342126612, 'right_e1': 1.0270001374892845}

position_5 = {'right_s0': 1.0082088728376881, 'right_s1': -0.48320394818390533, 'right_w0': 0.16106798272796843, 'right_w1': 0.8801214770492561, 'right_w2': 0.28532042654668693, 'right_e0': -0.17065536265225228, 'right_e1': 1.1520195717019457}

position_6 = {'right_s0': 0.08360195293975504, 'right_s1': 0.013805827090968724, 'right_w0': 0.8854904098068551, 'right_w1': -1.5715633171886063, 'right_w2': 0.07133010663667173, 'right_e0': -0.19865051203116108, 'right_e1': 0.18791264651596318}

position_7 = {'right_s0': 0.0790000105760988, 'right_s1': 0.05253884198507542, 'right_w0': -0.5798447378206864, 'right_w1': -1.562359432461294, 'right_w2': 0.01802427425765361, 'right_e0': -0.27650003701634585, 'right_e1': 0.17679128580379394}

position_8 = {'right_s0': 0.8114758367913839, 'right_s1': -0.6795534890332383, 'right_w0': 0.21705828148578604, 'right_w1': 0.680320479427181, 'right_w2': -0.24236896448589537, 'right_e0': -0.14764565083397108, 'right_e1': 1.5063691337034764}



gripper.calibrate()
limb.move_to_joint_positions(position_1)
limb.move_to_joint_positions(position_2)
gripper.close()
rospy.sleep(0.5)
limb.move_to_joint_positions(position_5)
limb.move_to_joint_positions(position_3)
gripper.open()
rospy.sleep(0.5)
limb.set_joint_position_speed(0.75)
limb.move_to_joint_positions(position_4)
limb.move_to_neutral()


for i in range(2):
    limb.move_to_joint_positions(position_6)
    limb.move_to_joint_positions(position_7)

limb.set_joint_position_speed(0.7)

limb.move_to_neutral()
limb.move_to_joint_positions(position_4)
limb.move_to_joint_positions(position_3)
gripper.close()
rospy.sleep(0.5)
limb.move_to_joint_positions(position_5)
limb.move_to_joint_positions(position_2)
gripper.open()
rospy.sleep(0.5)
limb.move_to_joint_positions(position_8)
limb.move_to_neutral()


quit()