#!/usr/bin/env python

import rospy
import baxter_interface

def print_pose(limb_name, limb_object):
    try:
        current_pose = limb_object.endpoint_pose()
        rospy.loginfo("\n*** %s Arm End-Effector Pose: ***", limb_name.capitalize())
        rospy.loginfo("Position: \n x: %f\n y: %f\n z: %f" %
              (current_pose['position'].x,
               current_pose['position'].y,
               current_pose['position'].z))
        rospy.loginfo("Orientation: \n x: %f\n y: %f\n z: %f\n w: %f" %
              (current_pose['orientation'].x,
               current_pose['orientation'].y,
               current_pose['orientation'].z,
               current_pose['orientation'].w))
    except Exception as e:
        rospy.logerr("Could not get pose for %s arm: %s" % (limb_name, e))

def main():
    rospy.init_node('baxter_get_endpoint_pose')
    rospy.loginfo("Querying current robot end-effector poses...")
    left_limb = baxter_interface.Limb('left')
    right_limb = baxter_interface.Limb('right')
    print_pose('left', left_limb)
    print_pose('right', right_limb)

if __name__ == '__main__':
    main()
