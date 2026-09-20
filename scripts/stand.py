#!/usr/bin/env python3
"""Move the hexapod from whatever pose it is in to the standing pose.

Waits for the position controller to be listening and for /joint_states to
report all 18 joints, then linearly interpolates from the measured pose to
[coxa 0.0, femur -0.5, tibia -1.0] on every leg over 2 s at 50 Hz.

    ros2 run hexapod_description stand.py
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = '/hexapod_position_controller/commands'

# Must match the `joints:` list of hexapod_position_controller in
# config/controllers.yaml and the ros2_control block in urdf/hexapod.urdf.
JOINT_NAMES = [
    f'leg_{leg}_{segment}_joint'
    for leg in range(1, 7)
    for segment in ('coxa', 'femur', 'tibia')
]

# Standing pose, per leg. Both values are inside the URDF limits
# (femur +/-1.570796, tibia [-2.181662, 0.0]).
STAND_POSE = {'coxa': 0.0, 'femur': -0.5, 'tibia': -1.0}

RATE_HZ = 50.0
DURATION_S = 2.0


class Stand(Node):

    def __init__(self):
        super().__init__('stand')

        self.publisher = self.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)

        # joint_state_broadcaster publishes /joint_states best-effort-friendly;
        # depth 1 keeps only the freshest sample.
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self._lock = threading.Lock()
        self._positions = {}

        self.target = [STAND_POSE[name.split('_')[2]] for name in JOINT_NAMES]

    def _on_joint_states(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._positions[name] = position

    def current_pose(self):
        """Measured positions in JOINT_NAMES order, or None if any are missing."""
        with self._lock:
            if not all(name in self._positions for name in JOINT_NAMES):
                return None
            return [self._positions[name] for name in JOINT_NAMES]

    def wait_for_subscriber(self, timeout_s=60.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f'{COMMAND_TOPIC} has a subscriber.')
                return True
            time.sleep(0.1)
        self.get_logger().error(f'Timed out waiting for a subscriber on {COMMAND_TOPIC}.')
        return False

    def wait_for_joint_states(self, timeout_s=60.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pose = self.current_pose()
            if pose is not None:
                self.get_logger().info(f'/joint_states has all {len(JOINT_NAMES)} joints.')
                return pose
            time.sleep(0.1)
        with self._lock:
            missing = [n for n in JOINT_NAMES if n not in self._positions]
        self.get_logger().error(f'Timed out; /joint_states never reported: {missing}')
        return None

    def interpolate_to_stand(self, start):
        steps = int(RATE_HZ * DURATION_S)
        period = 1.0 / RATE_HZ
        msg = Float64MultiArray()

        self.get_logger().info(
            f'Standing: {steps} steps over {DURATION_S:.1f} s at {RATE_HZ:.0f} Hz.')

        next_tick = time.monotonic()
        for step in range(1, steps + 1):
            alpha = step / steps
            msg.data = [
                s + alpha * (t - s) for s, t in zip(start, self.target)
            ]
            self.publisher.publish(msg)

            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

        # Hold the final pose briefly so the servos settle on it.
        msg.data = list(self.target)
        for _ in range(int(RATE_HZ * 0.5)):
            self.publisher.publish(msg)
            time.sleep(period)


def main():
    rclpy.init()
    node = Stand()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        if not node.wait_for_subscriber():
            exit_code = 1
        else:
            start = node.wait_for_joint_states()
            if start is None:
                exit_code = 1
            else:
                node.get_logger().info(
                    'start pose: [' + ', '.join(f'{p:.3f}' for p in start) + ']')
                node.interpolate_to_stand(start)

                reached = node.current_pose()
                if reached is not None:
                    error = max(abs(r - t) for r, t in zip(reached, node.target))
                    node.get_logger().info(
                        'final pose: [' + ', '.join(f'{p:.3f}' for p in reached) + ']')
                    node.get_logger().info(f'max joint error: {error:.4f} rad')
                node.get_logger().info('Standing pose reached.')
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        # Wake spin() and let the thread finish before tearing the node down;
        # destroying it under a live executor aborts at interpreter exit.
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
