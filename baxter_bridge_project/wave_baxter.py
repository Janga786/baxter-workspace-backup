
import rclpy
from rclpy.node import Node
from baxter_core_msgs.msg import JointCommand
import time

class BaxterWaver(Node):
    def __init__(self):
        super().__init__('baxter_waver')
        self.publisher_ = self.create_publisher(JointCommand, '/robot/limb/left/joint_command', 10)
        self.get_logger().info('BaxterWaver node started and publisher created.')

    def wave(self):
        joint_names = [
            'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'
        ]

        # Define wave motion keyframes (joint positions in radians)
        # These are example values and might need adjustment for a natural wave
        keyframes = [
            # Position 1: Arm slightly bent, ready to wave
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # Position 2: Arm up
            [0.0, -0.5, 0.0, 1.5, 0.0, 0.0, 0.0],
            # Position 3: Wave right
            [0.0, -0.5, 0.0, 1.5, 0.0, -0.5, 0.0],
            # Position 4: Wave left
            [0.0, -0.5, 0.0, 1.5, 0.0, 0.5, 0.0],
            # Position 5: Back to arm up
            [0.0, -0.5, 0.0, 1.5, 0.0, 0.0, 0.0],
            # Position 6: Back to initial position
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]

        self.get_logger().info('Starting wave motion...')
        for _ in range(3):  # Repeat wave 3 times
            for i, positions in enumerate(keyframes):
                msg = JointCommand()
                msg.mode = JointCommand.POSITION_MODE
                msg.names = joint_names
                msg.command = [float(p) for p in positions]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published keyframe {i+1}: {positions}')
                time.sleep(2)  # Wait for 2 seconds between keyframes

        self.get_logger().info('Wave motion completed.')

def main(args=None):
    rclpy.init(args=args)
    baxter_waver = BaxterWaver()
    baxter_waver.wave()
    baxter_waver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
