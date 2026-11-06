"""Planar geometry and footprint-aware A*. Independently authored, MIT."""
from dataclasses import dataclass
import heapq
import math


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('values must be finite')


def wrap(angle):
    finite(angle)
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        finite(self.x, self.y, self.yaw)


@dataclass(frozen=True)
class Rect:
    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self):
        finite(self.x1, self.y1, self.x2, self.y2)
        if self.x1 >= self.x2 or self.y1 >= self.y2:
            raise ValueError('rectangle must have positive area')

    def distance(self, x, y):
        if self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2:
            return -min(x - self.x1, self.x2 - x, y - self.y1, self.y2 - y)
        return math.hypot(max(self.x1 - x, 0, x - self.x2),
                          max(self.y1 - y, 0, y - self.y2))


@dataclass(frozen=True)
class World:
    width: float
    height: float
    obstacles: tuple = ()
    resolution: float = 0.1

    def __post_init__(self):
        finite(self.width, self.height, self.resolution)
        if min(self.width, self.height, self.resolution) <= 0:
            raise ValueError('world dimensions and resolution must be positive')
        if not all(isinstance(r, Rect) for r in self.obstacles):
            raise ValueError('obstacles must be rectangles')
        if self.width / self.resolution * self.height / self.resolution > 1_000_000:
            raise ValueError('educational grid is limited to one million cells')

    def clearance(self, x, y, extra=()):
        finite(x, y)
        return min(x, y, self.width - x, self.height - y,
                   *(r.distance(x, y) for r in (*self.obstacles, *extra)))


def integrate(pose, v, w, dt):
    """Exact unicycle integration for one constant-twist interval, metres/radians."""
    finite(v, w, dt)
    if dt <= 0:
        raise ValueError('dt must be positive')
    yaw = pose.yaw + w * dt
    if abs(w) < 1e-10:
        return Pose(pose.x + v * dt * math.cos(pose.yaw),
                    pose.y + v * dt * math.sin(pose.yaw), wrap(yaw))
    return Pose(pose.x + v / w * (math.sin(yaw) - math.sin(pose.yaw)),
                pose.y - v / w * (math.cos(yaw) - math.cos(pose.yaw)), wrap(yaw))


def segment_clear(world, a, b, radius):
    """Sample at <= quarter-grid spacing; conservative extra margin is explicit."""
    distance = math.dist(a, b)
    steps = max(1, math.ceil(distance / (world.resolution / 4)))
    margin = world.resolution / 4
    return all(world.clearance(a[0] + (b[0] - a[0]) * i / steps,
                               a[1] + (b[1] - a[1]) * i / steps) > radius + margin
               for i in range(steps + 1))


def plan(world, start, goal, radius=0.22):
    """Eight-neighbour A* with footprint inflation and no diagonal corner cuts.

    Endpoints must connect safely to the nearest grid point. A coarse grid can
    reject a physically possible route; it must never invent a valid route.
    """
    finite(*start, *goal, radius)
    if len(start) != 2 or len(goal) != 2 or radius <= 0:
        raise ValueError('two-dimensional endpoints and positive radius required')
    if any(world.clearance(*p) <= radius for p in (start, goal)):
        raise ValueError('start or goal is outside the free footprint region')
    res = world.resolution
    source = tuple(round(x / res) for x in start)
    target = tuple(round(x / res) for x in goal)

    def point(cell):
        return cell[0] * res, cell[1] * res

    def free(cell):
        return world.clearance(*point(cell)) > radius + res / 4

    if not (segment_clear(world, start, point(source), radius)
            and segment_clear(world, point(target), goal, radius)):
        raise ValueError('endpoint cannot connect to this grid at this footprint')
    frontier = [(math.dist(source, target), 0.0, source)]
    costs, parent = {source: 0.0}, {}
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if cost > costs[current]:
            continue
        if current == target:
            cells = [current]
            while current in parent:
                current = parent[current]
                cells.append(current)
            path = [point(c) for c in reversed(cells)]
            if path[0] != tuple(start):
                path.insert(0, tuple(start))
            if path[-1] != tuple(goal):
                path.append(tuple(goal))
            return path
        for dx, dy in moves:
            nxt = current[0] + dx, current[1] + dy
            if not free(nxt):
                continue
            if dx and dy and not (free((current[0] + dx, current[1]))
                                  and free((current[0], current[1] + dy))):
                continue
            if not segment_clear(world, point(current), point(nxt), radius):
                continue
            candidate = cost + math.hypot(dx, dy)
            if candidate < costs.get(nxt, math.inf):
                costs[nxt], parent[nxt] = candidate, current
                heapq.heappush(frontier, (candidate + math.dist(nxt, target), candidate, nxt))
    raise ValueError('no footprint-safe grid path exists')
