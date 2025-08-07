#!/usr/bin/env python

import rospy
import baxter_interface

def enable_baxter():
    print("Initializing ROS node...")
    rospy.init_node('enable_baxter', anonymous=True)
    
    print("Connecting to Baxter...")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    
    print("Current robot state:", "Enabled" if rs.state().enabled else "Disabled")
    
    if not rs.state().enabled:
        print("Enabling robot...")
        rs.enable()
        rospy.sleep(1.0)
        
        if rs.state().enabled:
            print("SUCCESS: Robot enabled successfully!")
            return True
        else:
            print("ERROR: Failed to enable robot")
            return False
    else:
        print("SUCCESS: Robot is already enabled!")
        return True

if __name__ == '__main__':
    try:
        enable_baxter()
    except Exception as e:
        print("ERROR enabling robot:", str(e))