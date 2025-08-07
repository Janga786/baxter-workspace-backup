#!/usr/bin/env python

import numpy
import cmath
import rospy
import rospy
import baxter_interface

rospy.init_node('Hello_Baxter')

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

# Inspection
image_close_1 = {'right_s0': 0.6339175605936472, 'right_s1': -0.041033986075934815, 'right_w0': 0.49585928968396, 'right_w1': 1.2935292993843752, 'right_w2': -3.0595246814374577, 'right_e0': -0.5430291989114364, 'right_e1': 0.08782040010643993}
image_close_2 = {'right_s0': 0.9790632378678653, 'right_s1': -0.08053399136398422, 'right_w0': 0.6895243641544935, 'right_w1': 1.514806028036846, 'right_w2': -2.991646031573528, 'right_e0': -0.6818544602150665, 'right_e1': 0.0839854481367264}
image_close_3 = {'right_s0': 1.358339987672534, 'right_s1': -0.0617427267123879, 'right_w0': 0.7800292306397328, 'right_w1': 1.4166312576121796, 'right_w2': -2.880048929254864, 'right_e0': -0.8103253512004698, 'right_e1': 0.004601942363656241}
image_close_4 = {'right_s0': 1.562359432461294, 'right_s1': -0.41800976469877527, 'right_w0': 0.6193447431087358, 'right_w1': 1.4925633066125075, 'right_w2': -2.8842673764215494, 'right_e0': -0.5825292041994858, 'right_e1': 0.7478156340941392}
image_close_5 = {'right_s0': 1.32612639112694, 'right_s1': -0.5637379395478895, 'right_w0': 0.7110000951848893, 'right_w1': 1.2666846355963803, 'right_w2': -3.0388159408010047, 'right_e0': -0.8206797215186964, 'right_e1': 1.3219079439602552}
image_close_6 = {'right_s0': 1.066500142777334, 'right_s1': -0.5779272618358297, 'right_w0': 0.8344855486096651, 'right_w1': 1.1616069516262295, 'right_w2': -3.0579907006495723, 'right_e0': -0.9529855644738133, 'right_e1': 1.440791455021375}
image_close_7 = {'right_s0': 1.2590147316569533, 'right_s1': -0.7071651432151758, 'right_w0': 0.8931603137462821, 'right_w1': 1.0680341235652193, 'right_w2': -3.056840215058658, 'right_e0': -1.1673593795808, 'right_e1': 2.122645915236441}
image_close_8 = {'right_s0': 1.6087623512948277, 'right_s1': -0.3762087882288977, 'right_w0': 1.1397477253988624, 'right_w1': 1.5650438988400934, 'right_w2': -3.0595246814374577, 'right_e0': -1.285859395444948, 'right_e1': 1.868388599644434}
image_close_9 = {'right_s0': 1.5412671966278695, 'right_s1': 0.1407427372884867, 'right_w0': 1.763310915674283, 'right_w1': 2.011432308114749, 'right_w2': -2.748126581496719, 'right_e0': -1.5957235145978017, 'right_e1': -0.04947088040930459}
image_far_1   = {'right_s0': 1.4580487388850858, 'right_s1': -0.42031073588060336, 'right_w0': 2.2733595276461833, 'right_w1': 2.094267270660561, 'right_w2': -3.045335359149518, 'right_e0': -2.200495440221626, 'right_e1': -0.04908738521233324}
image_far_2   = {'right_s0': 1.214145793611305, 'right_s1': -0.7547185476396235, 'right_w0': 0.5322913333962386, 'right_w1': 2.095417756251475, 'right_w2': 3.0487868159222598, 'right_e0': -0.7512670908668814, 'right_e1': 0.6028544496389676}
image_far_3   =  {'right_s0': 0.8156942839580688, 'right_s1': -1.058063248443964, 'right_w0': 0.6147428007450796, 'right_w1': 2.0958012514484463, 'right_w2': 2.17326728123666, 'right_e0': -0.9821311994436361, 'right_e1': 0.9426311941555867}


limb.move_to_joint_positions(image_close_1)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_2)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_3)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_4)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_5)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_6)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_7)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_8)
rospy.sleep(1)
limb.move_to_joint_positions(image_close_9)
rospy.sleep(1)
limb.move_to_joint_positions(image_far_1)
rospy.sleep(1)
limb.move_to_joint_positions(image_far_2)
rospy.sleep(1)
limb.move_to_joint_positions(image_far_3)
rospy.sleep(1)

# quit
quit()