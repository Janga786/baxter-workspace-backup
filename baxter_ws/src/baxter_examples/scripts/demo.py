#!/usr/bin/env python

import rospy

import baxter_interface

rospy.init_node('Inspect using joint angles')

limbR = baxter_interface.Limb('right')
limbL = baxter_interface.Limb('left')

anglesR = limbR.joint_angles()
anglesL = limbL.joint_angles()


anglesR['right_s0']=0.0
anglesR['right_s1']=0.0
anglesR['right_e0']=0.0
anglesR['right_e1']=0.0
anglesR['right_w0']=0.0
anglesR['right_w1']=0.0
anglesR['right_w2']=0.0

anglesL['left_s0']=0.0
anglesL['left_s1']=0.0
anglesL['left_e0']=0.0
anglesL['left_e1']=0.0
anglesL['left_w0']=0.0
anglesL['left_w1']=0.0
anglesL['left_w2']=0.0

neutralL=anglesL
neutralR=anglesR

# Define joint position speed constants
SPEED_SLOW = 0.25
SPEED_FAST = 0.9
SPEED_WAVE = 0.75

limbR.set_joint_position_speed(SPEED_SLOW)
limbL.set_joint_position_speed(SPEED_SLOW)
limbR.move_to_neutral()
limbL.move_to_neutral()
gripper = baxter_interface.Gripper('right')
gripper.calibrate()


# Right arm pick and place positions
grab = {'right_s0': -0.2699806186678328, 'right_s1': -0.1342233189399737, 'right_w0': 0.2059369207736168, 'right_w1': 0.009587379924283835, 'right_w2': -0.11236409271260656, 'right_e0': -0.09012137128826805, 'right_e1': 0.07554855380335662}
place = {'right_s0': 1.1416652013837192, 'right_s1': -0.5449466748962932, 'right_w0': 0.39308257689563725, 'right_w1': 0.5871311465631421, 'right_w2': -0.0732475826215285, 'right_e0': -0.2607767339405203, 'right_e1': 1.5915050674311169}
interim_place_1 = {'right_s0': 1.1573885044595447, 'right_s1': -0.8095583608065271, 'right_w0': 0.28608741694062967, 'right_w1': 0.8183787503368682, 'right_w2': -0.0617427267123879, 'right_e0': -0.3148495567134812, 'right_e1': 1.6532477941435046}
pick = {'right_s0': 1.1639079228080578, 'right_s1': -0.501228222441559, 'right_w0': 0.4548253036080252, 'right_w1': 0.5530000740326917, 'right_w2': -0.13038836697026016, 'right_e0': -0.2688301330769188, 'right_e1': 1.5853691442795752}


# Left arm wave positions
srt_wave = {'left_w0': -0.23354857495555426, 'left_w1': -1.5692623460067783, 'left_w2': -0.029529130166794215, 'left_e0': 0.2757330466224031, 'left_e1': 0.5752427954570302, 'left_s0': -0.882038953034113, 'left_s1': -0.38004374019861126}
wave_position_1 = {'left_w0': 0.6419709597300457, 'left_w1': -1.4473108733698878, 'left_w2': 0.05829126993964572, 'left_e0': 0.3765922834258691, 'left_e1': 0.37199034106221285, 'left_s0': -0.896228275322053, 'left_s1': -0.3470631532590749}
wave_position_2 = {'left_w0': -1.5665778796279788, 'left_w1': -1.4864273834609658, 'left_w2': -0.014956312681882784, 'left_e0': 0.5307573526083531, 'left_e1': 0.3857961681531816, 'left_s0': -0.9177040063524488, 'left_s1': -0.3359417925469056}


# The wave dance
straight_armR = {'right_s0': -0.7788787450488188, 'right_s1': 0.023009711818281205, 'right_w0': -0.14879613642488512, 'right_w1': 0.01687378866673955, 'right_w2': 0.18522818013716372, 'right_e0': 0.0790000105760988, 'right_e1': -0.04908738521233324}
straight_armL = {'left_w0': -0.11619904468232009, 'left_w1': 0.013805827090968724, 'left_w2': 0.2396844981070959, 'left_e0': 0.05829126993964572, 'left_e1': -0.0502378708032473, 'left_s0': 0.7113835903818606, 'left_s1': -0.03144660615165098}
hand_upR = {'right_s0': -0.7792622402457902, 'right_s1': -0.006902913545484362, 'right_w0': 0.25885925795566356, 'right_w1': -0.870150601928001, 'right_w2': 0.04371845245473429, 'right_e0': -0.2044029399857314, 'right_e1': -0.0502378708032473}
elbow_upR = {'right_s0': -0.8981457513069098, 'right_s1': -0.7581700044123657, 'right_w0': 0.2945243112739994, 'right_w1': 1.0983302441259561, 'right_w2': -0.04065049087896346, 'right_e0': 0.005752427954570301, 'right_e1': 0.9671748867617533}
shoulder_upR = {'right_s0': -0.8429224429430349, 'right_s1': -0.884339924215941, 'right_w0': 0.049854375606275945, 'right_w1': -1.5696458412037497, 'right_w2': 0.05829126993964572, 'right_e0': -0.05714078434873166, 'right_e1': 1.9132575376900822}
shoulder_upL = {'left_w0': 0.13230584295511694, 'left_w1': -1.5719468123855778, 'left_w2': -0.0625097171063306, 'left_e0': -0.1606844875309971, 'left_e1': 1.7966749978107908, 'left_s0': 0.8728350683068005, 'left_s1': -0.8364030245945219}
elbow_upL = {'left_w0': 0.011504855909140603, 'left_w1': 1.2122283176264481, 'left_w2': 0.1507136124097419, 'left_e0': -0.10085923680346595, 'left_e1': 0.922305948716105, 'left_s0': 0.8325680726248084, 'left_s1': -0.9717768291254096}
hand_upL = {'left_w0': -0.20133497840996056, 'left_w1': -0.8586457460188603, 'left_w2': 0.0038349519697135344, 'left_e0': 0.16912138186436687, 'left_e1': -0.04947088040930459, 'left_s0': 0.756252528427509, 'left_s1': 0.018791264651596317}







# Move to grab position
limbR.move_to_joint_positions(grab)
rospy.sleep(4.0)

# Move left arm to wave position
limbL.move_to_joint_positions(srt_wave)
limbL.set_joint_position_speed(SPEED_WAVE)

# Perform wave motion with left arm
for i in range(2):
    limbL.move_to_joint_positions(wave_position_1)
    limbL.move_to_joint_positions(wave_position_2)
limbL.set_joint_position_speed(0.25)

# Place Duck
gripper.close()
rospy.sleep(0.5)
limbR.move_to_joint_positions(place)
gripper.open()
rospy.sleep(0.5)
limbR.move_to_joint_positions(interim_place_1)

# Wave dance
limbR.move_to_joint_positions(straight_armR)
limbL.move_to_joint_positions(straight_armL)
limbR.set_joint_position_speed(SPEED_FAST)
limbL.set_joint_position_speed(SPEED_FAST)
limbR.move_to_joint_positions(hand_upR)
limbR.move_to_joint_positions(elbow_upR)
limbL.move_to_joint_positions(elbow_upL)
limbR.move_to_joint_positions(straight_armR)
limbL.move_to_joint_positions(hand_upL)
limbL.move_to_joint_positions(elbow_upL)
limbR.move_to_joint_positions(elbow_upR)
limbR.move_to_joint_positions(hand_upR)
limbR.move_to_joint_positions(straight_armR)
limbR.set_joint_position_speed(0.25)
limbL.set_joint_position_speed(0.25)

# Grab and give duck back
limbR.move_to_joint_positions(interim_place_1)
limbR.move_to_joint_positions(pick)
rospy.sleep(0.5)
gripper.close()
rospy.sleep(0.5)
limbR.move_to_joint_positions(grab)



quit()