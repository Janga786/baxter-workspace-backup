#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import time

class BaxterEnableService(Node):
    def __init__(self):
        super().__init__('baxter_enable_service')
        
        # Create service to enable/disable Baxter
        self.enable_service = self.create_service(
            SetBool,
            '/baxter/enable_robot',
            self.enable_robot_callback
        )
        
        self.get_logger().info('🤖 Baxter Enable Service started')
        self.get_logger().info('Call: ros2 service call /baxter/enable_robot std_srvs/srv/SetBool "{data: true}"')
        
    def enable_robot_callback(self, request, response):
        """Enable or disable Baxter robot"""
        
        if request.data:
            self.get_logger().info('🔋 Enabling Baxter robot...')
            success = self.enable_baxter()
        else:
            self.get_logger().info('🔌 Disabling Baxter robot...')
            success = self.disable_baxter()
            
        response.success = success
        if success:
            response.message = f"Baxter {'enabled' if request.data else 'disabled'} successfully"
        else:
            response.message = f"Failed to {'enable' if request.data else 'disable'} Baxter"
            
        return response
    
    def enable_baxter(self):
        """Enable Baxter using ROS1 commands"""
        try:
            # Script to enable Baxter via ROS1
            enable_script = '''
import rospy
from std_msgs.msg import Bool

rospy.init_node('baxter_enabler', anonymous=True)

# Enable robot
enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
rospy.sleep(2.0)  # Wait for publisher to connect

print("Enabling Baxter robot...")
enable_msg = Bool()
enable_msg.data = True

for i in range(10):
    enable_pub.publish(enable_msg)
    rospy.sleep(0.5)
    
print("Baxter enable commands sent!")

# Check if robot is enabled
import rospy
from baxter_core_msgs.msg import AssemblyState

def state_callback(msg):
    if msg.enabled:
        print("✅ Baxter is now ENABLED!")
        rospy.signal_shutdown("Robot enabled")
    else:
        print("⏳ Waiting for robot to enable...")

rospy.Subscriber('/robot/state', AssemblyState, state_callback)
rospy.sleep(5.0)  # Wait up to 5 seconds
'''
            
            # Write script to temp file
            with open('/tmp/enable_baxter.py', 'w') as f:
                f.write(enable_script)
            
            self.get_logger().info('📡 Sending enable command to Baxter...')
            
            # Execute via ROS1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/enable_baxter.py:/tmp/enable_baxter.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/enable_baxter.py'
            ], capture_output=True, text=True, timeout=15)
            
            if result.returncode == 0:
                self.get_logger().info('✅ Baxter enable command completed')
                self.get_logger().info(f'Output: {result.stdout}')
                return True
            else:
                self.get_logger().error(f'❌ Enable command failed: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error enabling Baxter: {e}')
            return False
    
    def disable_baxter(self):
        """Disable Baxter using ROS1 commands"""
        try:
            # Script to disable Baxter via ROS1
            disable_script = '''
import rospy
from std_msgs.msg import Bool

rospy.init_node('baxter_disabler', anonymous=True)

# Disable robot  
enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
rospy.sleep(2.0)

print("Disabling Baxter robot...")
disable_msg = Bool()
disable_msg.data = False

for i in range(5):
    enable_pub.publish(disable_msg)
    rospy.sleep(0.5)
    
print("Baxter disable commands sent!")
'''
            
            with open('/tmp/disable_baxter.py', 'w') as f:
                f.write(disable_script)
            
            # Execute via ROS1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/disable_baxter.py:/tmp/disable_baxter.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/disable_baxter.py'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info('✅ Baxter disabled successfully')
                return True
            else:
                self.get_logger().error(f'❌ Disable command failed: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error disabling Baxter: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    service = BaxterEnableService()
    
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()