import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lerobot.motors import SerialMotorsBus, Motor, MotorNormMode

class SO101HardwareBridge(Node):
    def __init__(self):
        super().__init__('so101_hardware_bridge')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.listener_callback,
            10
        )
        
        self.get_logger().info("Connecting to Follower Arm (/dev/ttyACM0) with SerialMotorsBus...")
        self.motors_dict = {f'joint_{i}': Motor(i, 'sts3215', MotorNormMode.RANGE_M100_100) for i in range(1, 7)}
        
        # Connect using the official recommended SerialMotorsBus
        self.bus = SerialMotorsBus(port='/dev/ttyACM0', motors=self.motors_dict)
        self.bus.connect()
        self.bus.enable_torque()
        self.get_logger().info("✅ Real follower arm connected and torque enabled!")

        # Software Slew Limiter initialization: start from current actual position to prevent sudden jerks
        self.cmd = {n: self.bus.read('Present_Position', n, normalize=False) for n in self.motors_dict}
        self.is_safe = True

    def listener_callback(self, msg):
        if not self.is_safe:
            return
            
        try:
            # 1. Unit conversion & direction correction (Radians to raw encoder counts 0-4095)
            rad_to_counts = 2048.0 / 3.14159  
            sign_multipliers = [1.0, -1.0, -1.0, 1.0, 1.0, 1.0]  
            
            raw_targets = {}
            for i in range(1, min(len(msg.position) + 1, 7)):
                joint_name = f'joint_{i}'
                mujoco_rad = msg.position[i-1]
                raw_target = 2048 + (mujoco_rad * rad_to_counts * sign_multipliers[i-1])
                raw_targets[joint_name] = int(raw_target)

            # 2. Software Slew Limiter: Limit maximum step per loop to ensure smooth motion
            MAX_STEP = 15  
            positions = {}
            for n, tgt in raw_targets.items():
                if n in self.cmd:
                    step = max(-MAX_STEP, min(MAX_STEP, tgt - self.cmd[n]))
                    self.cmd[n] += step
                    positions[n] = self.cmd[n]

            # 3. Safety Check: Read actual position from hardware and check for excessive tracking errors
            actual_positions = {}
            for joint_name in positions.keys():
                actual_positions[joint_name] = self.bus.read('Present_Position', joint_name, normalize=False)
            
            for joint_name, target in positions.items():
                if joint_name in actual_positions:
                    actual = actual_positions[joint_name]
                    if abs(target - actual) > 300:  # Safety error threshold
                        self.get_logger().error(f"🚨 Safety Alert: {joint_name} error too high! Target: {target}, Actual: {actual}")
                        self.emergency_stop()
                        return

            # Write smoothed commands to the physical robot
            self.bus.sync_write('Goal_Position', positions, normalize=False)
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop, triggering emergency stop: {e}")
            self.emergency_stop()

    def emergency_stop(self):
        self.is_safe = False
        try:
            self.bus.disable_torque()
            self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED: Real robot torque disabled!")
        except Exception:
            pass

    def destroy_node(self):
        if hasattr(self, 'bus'):
            try:
                self.bus.disable_torque()
                self.bus.disconnect()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = SO101HardwareBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
