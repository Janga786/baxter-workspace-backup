import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class BaxterWave(Node):

    def __init__(self):
        super().__init__('baxter_wave')
        self.publisher_ = self.create_publisher(JointState, '/robot/limb/left/joint_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every 1 second
        self.joint_names = [
            'left_s0',
            'left_s1',
            'left_e0',
            'left_e1',
            'left_w0',
            'left_w1',
            'left_w2'
        ]
        self.wave_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # Home position
            [0.0, -0.5, 0.0, 1.0, 0.0, 0.0, 0.0], # Wave up
            [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0], # Wave down
            [0.0, -0.5, 0.0, 1.0, 0.0, 0.0, 0.0], # Wave up
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Back to home
        ]
        self.current_step = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.wave_positions[self.current_step]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending joint command: {msg.position}')
        self.current_step = (self.current_step + 1) % len(self.wave_positions)

def main(args=None):
    rclpy.init(args=args)
    baxter_wave = BaxterWave()
    rclpy.spin(baxter_wave)
    baxter_wave.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()