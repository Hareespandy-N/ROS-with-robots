"""A separate simulated pose-alignment state machine, without charger I/O."""
from dataclasses import dataclass
import math

from .navigation import Pose, finite, integrate, wrap
from .simulation import clamp, ramp


@dataclass
class DockingController:
    goal: Pose
    timeout: float = 30.0
    state: str = 'ALIGN'
    elapsed: float = 0.0
    settled: float = 0.0

    def __post_init__(self):
        finite(self.timeout)
        if self.timeout <= 0:
            raise ValueError('timeout must be positive')

    def update(self, pose, dt, obstructed=False):
        finite(dt)
        if dt <= 0:
            raise ValueError('dt must be positive')
        if self.state in ('DOCKED', 'FAILED'):
            return 0.0, 0.0
        self.elapsed += dt
        if obstructed or self.elapsed >= self.timeout:
            self.state = 'FAILED'
            return 0.0, 0.0
        dx, dy = self.goal.x - pose.x, self.goal.y - pose.y
        distance = math.hypot(dx, dy)
        if distance <= 0.02:
            self.state = 'SETTLE'
            yaw_error = wrap(self.goal.yaw - pose.yaw)
            self.settled = self.settled + dt if abs(yaw_error) <= 0.03 else 0.0
            if self.settled >= 0.4:
                self.state = 'DOCKED'
                return 0.0, 0.0
            return 0.0, clamp(1.8 * yaw_error, -0.8, 0.8)
        self.settled = 0.0
        heading = wrap(math.atan2(dy, dx) - pose.yaw)
        self.state = 'ALIGN' if abs(heading) > 0.15 else 'APPROACH'
        return (0.0 if self.state == 'ALIGN' else min(0.15, distance * 0.8),
                clamp(1.8 * heading, -0.8, 0.8))


def run_docking():
    """Integrate proposed commands with shared rate limits in an empty plane."""
    pose, goal = Pose(0, 0, math.pi / 2), Pose(1, 0, 0)
    controller = DockingController(goal)
    v = w = 0.0
    dt = 0.05
    while controller.state not in ('DOCKED', 'FAILED'):
        desired_v, desired_w = controller.update(pose, dt)
        v, w = ramp(v, desired_v, 0.3 * dt), ramp(w, desired_w, 1.5 * dt)
        pose = integrate(pose, v, w, dt)
    return {'status': controller.state, 'duration_s': controller.elapsed,
            'xy_error_m': math.hypot(goal.x - pose.x, goal.y - pose.y),
            'yaw_error_rad': abs(wrap(goal.yaw - pose.yaw)),
            'measurement': 'ideal_simulator_ground_truth',
            'hardware_charge_command_sent': False}
