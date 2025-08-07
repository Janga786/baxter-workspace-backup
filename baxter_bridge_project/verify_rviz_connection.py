#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class RVizConnectionVerifier(Node):
    def __init__(self):
        super().__init__('rviz_connection_verifier')
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.joint_received = False
        self.get_logger().info('🔍 Verifying RViz connection to Baxter...')
        
    def joint_callback(self, msg):
        if not self.joint_received:
            self.joint_received = True
            self.get_logger().info('✅ Joint states being received!')
            self.get_logger().info(f'📊 Tracking {len(msg.name)} joints: {msg.name[:5]}...')
            
            # Check for Baxter-specific joints
            baxter_joints = [name for name in msg.name if 'left_' in name or 'right_' in name]
            if baxter_joints:
                self.get_logger().info(f'🤖 Baxter joints found: {len(baxter_joints)} joints')
                self.get_logger().info('✅ RViz should show the Baxter robot model!')
            else:
                self.get_logger().warn('⚠️ No Baxter joints found in joint states')

def main():
    rclpy.init()
    
    verifier = RVizConnectionVerifier()
    
    print("🔍 Verifying RViz connection...")
    print("This will check if joint states are flowing to RViz")
    print("Press Ctrl+C to stop")
    
    try:
        # Spin for a few seconds to receive messages
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 10:
            rclpy.spin_once(verifier, timeout_sec=0.1)
            
        if not verifier.joint_received:
            print("❌ No joint states received - check if bridges are running")
        else:
            print("✅ Connection verified - RViz should show Baxter!")
            
    except KeyboardInterrupt:
        print("\nVerification stopped")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()