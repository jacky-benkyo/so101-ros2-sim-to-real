import time
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Sim(Node):
    def __init__(self):
        super().__init__('mujoco_sim')
        self.declare_parameter('xml', '/home/jackychow/lerobot/mujoco/so101.xml')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('realtime_factor', 1.0)   # 1.0 = 現實速度
        self.declare_parameter('max_speed', 1.5)         # rad/s 關節速度上限
        
        self.model = mujoco.MjModel.from_xml_path(self.get_parameter('xml').value)
        self.data = mujoco.MjData(self.model)

        # 修正：將關節類型強制轉為 int 進行比較，避開 pybind11 與 numpy 類型不匹配問題
        hinge_type = int(mujoco.mjtJoint.mjJNT_HINGE)
        slide_type = int(mujoco.mjtJoint.mjJNT_SLIDE)
        
        J = [j for j in range(self.model.njnt)
             if int(self.model.jnt_type[j]) in (hinge_type, slide_type)]
             
        self.names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in J]
        self.qadr = [int(self.model.jnt_qposadr[j]) for j in J]
        self.nu = self.model.nu
        
        self.get_logger().info(f'✅ Successfully loaded joints={self.names} (total: {len(self.names)}), nu={self.nu}, dt={self.model.opt.timestep}')
        
        if self.nu != len(J):
            self.get_logger().warn('actuator count != joint count: check mapping')

        q = np.array([self.data.qpos[a] for a in self.qadr], dtype=float)
        self.cmd = q.copy()
        self.goal = q.copy()

        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_subscription(JointState, '/vla/joint_cmd', self.on_cmd, 10)
        self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.publish_state)

    def on_cmd(self, m):
        if m.name:
            for n, p in zip(m.name, m.position):
                if n in self.names:
                    self.cmd[self.names.index(n)] = p
        elif len(m.position) == len(self.cmd):
            self.cmd[:] = m.position

    def publish_state(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = self.names
        m.position = [float(self.data.qpos[a]) for a in self.qadr]
        self.pub.publish(m)

    def run(self):
        dt = self.model.opt.timestep
        t0 = time.perf_counter()
        sim0 = self.data.time
        with mujoco.viewer.launch_passive(self.model, self.data) as v:
            while v.is_running() and rclpy.ok():
                rt = self.get_parameter('realtime_factor').value
                step = self.get_parameter('max_speed').value * dt     # 每個 sim step 最大移動量
                target_t = sim0 + (time.perf_counter() - t0) * rt
                with v.lock():
                    while self.data.time < target_t:
                        self.goal += np.clip(self.cmd - self.goal, -step, step)
                        self.data.ctrl[:self.nu] = self.goal[:self.nu]
                        mujoco.mj_step(self.model, self.data)
                v.sync()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.002)

def main():
    rclpy.init()
    n = Sim()
    n.run()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()