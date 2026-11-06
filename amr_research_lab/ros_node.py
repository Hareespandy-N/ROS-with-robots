"""ROS 2 Humble adapter for this project's internal numerical simulator."""
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

from .simulation import Simulation

FRAME = 'amr_lab/map'
BASE_FRAME = 'amr_lab/base_link'
PREFIX = '/amr_lab/sim/'


class SimulationNode(Node):
    def __init__(self):
        super().__init__('amr_lab_simulation')
        scenario = self.declare_parameter('scenario', 'warehouse').value
        controller = self.declare_parameter('controller', 'regulated').value
        self.max_steps = self.declare_parameter('max_steps', 0).value
        if not isinstance(self.max_steps, int) or isinstance(self.max_steps, bool) or self.max_steps < 0:
            raise ValueError('max_steps must be a non-negative integer')
        self.sim = Simulation(scenario, controller)
        self.done = False
        self.odom_pub = self.create_publisher(Odometry, PREFIX + 'odom', 10)
        self.path_pub = self.create_publisher(Path, PREFIX + 'planned_path', 10)
        self.velocity_pub = self.create_publisher(Twist, PREFIX + 'cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, PREFIX + 'obstacles', 10)
        self.timer = self.create_timer(self.sim.limits.dt, self.tick)
        self.get_logger().info('Numerical simulation only; no hardware subscription or driver is present.')

    def tick(self):
        self.sim.step()
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = float(self.sim.pose.x)
        odom.pose.pose.position.y = float(self.sim.pose.y)
        odom.pose.pose.orientation.z = math.sin(self.sim.pose.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.sim.pose.yaw / 2)
        odom.twist.twist.linear.x = float(self.sim.v)
        odom.twist.twist.angular.z = float(self.sim.w)
        self.odom_pub.publish(odom)
        velocity = Twist()
        velocity.linear.x, velocity.angular.z = float(self.sim.v), float(self.sim.w)
        self.velocity_pub.publish(velocity)
        path = Path()
        path.header = odom.header
        for x, y in self.sim.path:
            point = PoseStamped()
            point.header = odom.header
            point.pose.position.x, point.pose.position.y = float(x), float(y)
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        self.path_pub.publish(path)
        markers = MarkerArray()
        reset = Marker()
        reset.header, reset.action = odom.header, Marker.DELETEALL
        markers.markers.append(reset)
        obstacles = (*self.sim.world.obstacles, *self.sim.scenario.dynamic_at(self.sim.time))
        for index, rect in enumerate(obstacles):
            marker = Marker()
            marker.header, marker.ns, marker.id = odom.header, 'simulated_obstacles', index
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose.position.x = float((rect.x1 + rect.x2) / 2)
            marker.pose.position.y = float((rect.y1 + rect.y2) / 2)
            marker.pose.position.z, marker.pose.orientation.w = 0.25, 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = float(rect.x2-rect.x1), float(rect.y2-rect.y1), 0.5
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.6, 0.7, 0.85, 0.9
            markers.markers.append(marker)
        self.obstacle_pub.publish(markers)
        self.done = (self.sim.status != 'running'
                     or bool(self.max_steps and self.sim.steps >= self.max_steps))
        if self.done:
            self.timer.cancel()
            result = self.sim.metrics()
            if result['status'] == 'running':
                result['status'] = 'step_limit'
            self.get_logger().info(json.dumps(result))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.sim.status in ('running', 'completed') else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
