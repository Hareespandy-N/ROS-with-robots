"""Deterministic ideal-kinematics experiments; no hardware interfaces."""
from dataclasses import asdict, dataclass
import math

from .navigation import Pose, Rect, World, finite, integrate, plan, wrap


@dataclass(frozen=True)
class Limits:
    dt: float = 0.05
    radius: float = 0.22
    max_v: float = 0.6
    max_w: float = 1.5
    max_a: float = 0.5
    max_alpha: float = 2.0
    xy_tolerance: float = 0.04
    yaw_tolerance: float = 0.04
    timeout: float = 120.0

    def __post_init__(self):
        values = tuple(asdict(self).values())
        finite(*values)
        if min(values) <= 0 or self.dt > self.timeout:
            raise ValueError('limits must be positive; dt must not exceed timeout')


@dataclass(frozen=True)
class Scenario:
    name: str
    world: World
    start: Pose
    goal: Pose
    blockage: Rect | None = None
    block_window: tuple = (3.0, 7.0)

    def __post_init__(self):
        if not isinstance(self.world, World) or not all(isinstance(p, Pose) for p in (self.start, self.goal)):
            raise ValueError('a World and start/goal Pose values are required')
        if self.blockage is not None and not isinstance(self.blockage, Rect):
            raise ValueError('blockage must be a Rect or None')
        if not isinstance(self.block_window, (tuple, list)) or len(self.block_window) != 2:
            raise ValueError('block_window requires exactly two timestamps')
        try:
            finite(*self.block_window)
            valid_window = 0 <= self.block_window[0] < self.block_window[1]
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError('block_window timestamps must be finite numbers') from error
        if not valid_window:
            raise ValueError('block_window must satisfy 0 <= start < end')
        object.__setattr__(self, 'block_window', tuple(self.block_window))

    def dynamic_at(self, t):
        return (self.blockage,) if (self.blockage is not None
                                    and self.block_window[0] <= t < self.block_window[1]) else ()


def scenario_named(name):
    if name == 'open_floor':
        return Scenario(name, World(10, 7), Pose(0.8, 0.8, 0), Pose(9, 5.7, math.pi / 2))
    if name == 'warehouse':
        return Scenario(name, World(10, 7, (Rect(2.7, 0.6, 3.5, 4.8),
                                            Rect(5.5, 2.2, 6.3, 6.3))),
                        Pose(0.8, 0.8, 0), Pose(9, 5.7, math.pi / 2))
    if name == 'temporary_blockage':
        return Scenario(name, World(10, 4), Pose(0.8, 2, 0), Pose(9.2, 2, 0),
                        Rect(3.2, 0.1, 3.5, 3.9))
    raise ValueError('unknown scenario: ' + str(name))


SCENARIOS = ('open_floor', 'warehouse', 'temporary_blockage')
CONTROLLERS = ('baseline', 'regulated')


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def ramp(previous, target, delta):
    return previous + clamp(target - previous, -delta, delta)


def swept_clear(world, pose, v, w, duration, radius, extra=()):
    """Conservative sampled disk check, including a half-sample travel margin.

    This bounds spatial gaps for the ideal constant-twist model. It is not a
    sensor-based collision monitor or a physical braking guarantee.
    """
    finite(v, w, duration, radius)
    if duration <= 0 or radius <= 0:
        raise ValueError('positive duration and radius required')
    steps = max(1, math.ceil(abs(v) * duration / 0.025))
    margin = abs(v) * duration / (2 * steps)
    for i in range(steps + 1):
        p = pose if i == 0 else integrate(pose, v, w, duration * i / steps)
        if world.clearance(p.x, p.y, extra) <= radius + margin:
            return False
    return True


@dataclass(frozen=True)
class Sample:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    clearance: float
    blocked: bool


class Simulation:
    def __init__(self, scenario='warehouse', controller='regulated', limits=None):
        if controller not in CONTROLLERS:
            raise ValueError('unknown controller')
        self.scenario = scenario_named(scenario) if isinstance(scenario, str) else scenario
        if not isinstance(self.scenario, Scenario):
            raise ValueError('scenario must be a name or Scenario')
        self.limits = limits or Limits()
        self.controller = controller
        self.world = self.scenario.world
        self.pose = self.scenario.start
        self.goal = self.scenario.goal
        # Deliberate tracking margin, shared by both controllers.
        self.path = plan(self.world, (self.pose.x, self.pose.y),
                         (self.goal.x, self.goal.y), self.limits.radius + 0.13)
        self.v = self.w = 0.0
        self.steps = self.index = self.settled = 0
        self.status = 'running'
        self.trace = [self._sample(False)]

    @property
    def time(self):
        return self.steps * self.limits.dt

    def _sample(self, blocked):
        return Sample(self.time, self.pose.x, self.pose.y, self.pose.yaw, self.v, self.w,
                      self.world.clearance(self.pose.x, self.pose.y,
                                           self.scenario.dynamic_at(self.time)) - self.limits.radius,
                      blocked)

    def _desired(self):
        limits = self.limits
        distance = math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y)
        if distance <= limits.xy_tolerance:
            return 0.0, clamp(2 * wrap(self.goal.yaw - self.pose.yaw), -limits.max_w, limits.max_w)
        self.index = min(range(self.index, len(self.path)),
                         key=lambda i: math.hypot(self.path[i][0] - self.pose.x,
                                                   self.path[i][1] - self.pose.y))
        lookahead = 0.45 if self.controller == 'baseline' else clamp(0.3 + abs(self.v) * 0.5, 0.3, 0.6)
        carrot_index, along = self.index, 0.0
        while carrot_index + 1 < len(self.path) and along < lookahead:
            along += math.dist(self.path[carrot_index], self.path[carrot_index + 1])
            carrot_index += 1
        dx = self.path[carrot_index][0] - self.pose.x
        dy = self.path[carrot_index][1] - self.pose.y
        heading = wrap(math.atan2(dy, dx) - self.pose.yaw)
        if abs(heading) > 0.55:
            return 0.0, clamp(2 * heading, -limits.max_w, limits.max_w)
        lateral = -math.sin(self.pose.yaw) * dx + math.cos(self.pose.yaw) * dy
        curvature = 2 * lateral / max(dx * dx + dy * dy, 1e-8)
        v = min(limits.max_v, math.sqrt(2 * limits.max_a * max(distance - 0.015, 0)))
        if self.controller == 'regulated':
            clearance = self.world.clearance(self.pose.x, self.pose.y,
                                             self.scenario.dynamic_at(self.time)) - limits.radius
            v = min(v, limits.max_v / max(1.0, abs(curvature) * 0.8),
                    limits.max_v * clamp(clearance / 0.8, 0.15, 1.0))
        v = min(v, limits.max_w / max(abs(curvature), 1e-8))
        return v, v * curvature

    def _step_clear(self, v, w):
        """Split numerical physics at scripted obstacle events inside this step."""
        start, end = self.time, self.time + self.limits.dt
        boundaries = [start, *(event for event in self.scenario.block_window
                                if start < event < end), end]
        for lower, upper in zip(boundaries, boundaries[1:]):
            offset = lower - start
            pose = self.pose if offset == 0 else integrate(self.pose, v, w, offset)
            extra = self.scenario.dynamic_at((lower + upper) / 2)
            if not swept_clear(self.world, pose, v, w, upper-lower, self.limits.radius, extra):
                return False
        final = integrate(self.pose, v, w, self.limits.dt)
        return self.world.clearance(final.x, final.y, self.scenario.dynamic_at(end)) > self.limits.radius

    def step(self):
        if self.status != 'running':
            return
        limits = self.limits
        extra = self.scenario.dynamic_at(self.time)
        if self.world.clearance(self.pose.x, self.pose.y, extra) <= limits.radius:
            self.status = 'collision'
            return
        desired_v, desired_w = self._desired()
        candidate_v = ramp(self.v, desired_v, limits.max_a * limits.dt)
        candidate_w = ramp(self.w, desired_w, limits.max_alpha * limits.dt)
        horizon = max(0.6, abs(candidate_v) / limits.max_a + 0.25)
        blocked = not swept_clear(self.world, self.pose, candidate_v, candidate_w,
                                  horizon, limits.radius, extra)
        if blocked:
            candidate_v = ramp(self.v, 0.0, limits.max_a * limits.dt)
        if not self._step_clear(candidate_v, candidate_w):
            # Collision failure stops numerical integration. Do not claim that
            # a discontinuous emergency stop satisfied acceleration limits.
            self.status = 'collision'
            return
        self.v, self.w = candidate_v, candidate_w
        self.pose = integrate(self.pose, self.v, self.w, limits.dt)
        self.steps += 1
        self.trace.append(self._sample(blocked))
        distance = math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y)
        at_goal = (distance <= limits.xy_tolerance
                   and abs(wrap(self.goal.yaw - self.pose.yaw)) <= limits.yaw_tolerance
                   and abs(self.v) < 0.005 and abs(self.w) < 0.04)
        self.settled = self.settled + 1 if at_goal else 0
        if self.settled * limits.dt >= 0.3:
            self.status = 'completed'
        elif self.time >= limits.timeout:
            self.status = 'timeout'

    def run(self):
        while self.status == 'running':
            self.step()
        return self

    def metrics(self):
        trace, dt = self.trace, self.limits.dt
        return {
            'scenario': self.scenario.name, 'controller': self.controller,
            'status': self.status, 'measurement': 'ideal_simulator_ground_truth',
            'duration_s': self.time,
            'distance_m': sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(trace, trace[1:])),
            'min_clearance_m': min(s.clearance for s in trace),
            'blocked_time_s': sum(s.blocked for s in trace[1:]) * dt,
            'max_speed_mps': max(abs(s.v) for s in trace),
            'max_yaw_rate_radps': max(abs(s.w) for s in trace),
            'max_acceleration_mps2': max((abs(b.v - a.v) / dt for a, b in zip(trace, trace[1:])), default=0),
            'max_yaw_acceleration_radps2': max((abs(b.w - a.w) / dt for a, b in zip(trace, trace[1:])), default=0),
            'final_xy_error_m': math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y),
            'final_yaw_error_rad': abs(wrap(self.goal.yaw - self.pose.yaw)),
            'goal': asdict(self.goal), 'final_pose': asdict(self.pose),
            'limits': asdict(self.limits), 'steps': self.steps,
        }
