import time
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MuJoCoSimPublisher(Node):
    def __init__(self):
        super().__init__('mujoco_sim_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10)
        self.timer = self.create_timer(0.4, self.timer_callback) # Publish rate

        # Load MuJoCo Model
        self.model = mujoco.MjModel.from_xml_path("/home/jackychow/lerobot/mujoco/so101.xml")
        self.data = mujoco.MjData(self.model)
        self.get_logger().info("MuJoCo initialization successfully!")

    def timer_callback(self):
        # Step simulation forward
        mujoco.mj_step(self.model, self.data)

        # Build ROS 2 JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i}' for i in range(1, 7)]

        # Extract first 6 joint positions from MuJoCo data
        msg.position = list(self.data.qpos[:6])

        # Publish to ROS 2 topic
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSimPublisher()

    # Launch passive MuJoCo viewer
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        while viewer.is_running() and rclpy.ok():
            mujoco.mj_step(node.model, node.data)
            viewer.sync()
            rclpy.spin_once(node.node if hasattr(node, 'node') else node, timeout_sec=0.001)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
