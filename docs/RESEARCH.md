# Research sources and implementation map

These are references, not redistributed papers. The project implements an
original educational experiment; it does not claim a novel scientific method
or parity with any upstream navigation stack. Source metadata and conceptual
material were checked during preparation in November 2026.

| Primary reference | How it informs this project | Boundary |
|---|---|---|
| Macenski et al., [The Marathon 2: A Navigation System](https://arxiv.org/abs/2003.00368), IROS 2020, [DOI](https://doi.org/10.1109/IROS45743.2020.9341207) | Separating global path planning, local tracking, and task state makes experiments easier to inspect. | This repository does not implement Nav2 behavior trees or reproduce the marathon study. |
| Macenski et al., [From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in the Robot Operating System 2](https://arxiv.org/abs/2307.15236), 2023, [DOI](https://doi.org/10.1016/j.robot.2023.104493) | Compare algorithm choices under shared assumptions and report tradeoffs. | Only two small path trackers are compared; this is not a survey-wide benchmark. |
| Macenski et al., [Regulated Pure Pursuit for Robot Path Tracking](https://arxiv.org/abs/2305.20026), 2023 | Adaptive lookahead, curvature/proximity slowing, and projected collision checks motivate the educational regulated variant. | Independently written heuristics differ from the paper and Nav2 implementation. Neither code nor figures are copied. |
| Macenski and Jambrecic, [SLAM Toolbox: SLAM for the dynamic world](https://joss.theoj.org/papers/10.21105/joss.02783), JOSS 2021, [DOI](https://doi.org/10.21105/joss.02783) | Keep map generation/localization separate from tracking and identify the coordinate frame of measurements. | Background only: maps here are hand-authored rectangles; there is no SLAM or localization implementation. |
| Macenski et al., [Robot Operating System 2: Design, Architecture, and Uses In The Wild](https://arxiv.org/abs/2211.07752), Science Robotics 2022, [DOI](https://doi.org/10.1126/scirobotics.abm6074) | A ROS-independent numerical core and a small message adapter keep experiments reusable. | No middleware performance study or production deployment is reproduced. The manuscript/figures are not bundled. |

## The actual controller experiment

The baseline selects a fixed 0.45 m lookahead along the A* path. The regulated
variant selects a speed-dependent 0.30–0.60 m lookahead, then reduces speed
using curvature and local disk clearance. Both derive turning curvature from
the lateral lookahead offset in the robot frame and compute angular velocity
from the chosen linear velocity. Both rotate in place for substantial heading
error and align to the goal heading after reaching the position region.

Both trackers use the same 0.22 m disk, 0.13 m extra planning margin, 0.10 m
grid, 0.05 s integration, 0.60 m/s and 1.5 rad/s limits, and acceleration
limits of 0.50 m/s² and 2.0 rad/s². A sampled constant-twist projection with a
spatial margin requests braking. This is deliberately simpler than a physical
dynamic model. The temporary obstacle is visible everywhere in the synthetic
world and appears at t=3 s, disappearing at t=7 s; no perception algorithm is
being evaluated.

There is no random seed because these six experiments contain no random
process. A repeat run should generate identical numeric output on the same
Python/platform; small cross-platform floating-point differences are possible.
The original [raw outputs](../examples/results/) permit independent plotting.

## Related upstream software

These are integration/research pointers only. None is vendored or required by
the offline simulator. Before reusing upstream code or media, inspect the
license of the exact revision and files involved.

- [Nav2](https://github.com/ros-navigation/navigation2)
- [ros2_control demos](https://github.com/ros-controls/ros2_control_demos)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Open Navigation Docking](https://github.com/open-navigation/opennav_docking)
- [Open-RMF](https://github.com/open-rmf/rmf)
- [TurtleBot 4 Simulator](https://github.com/turtlebot/turtlebot4_simulator)

Fleet coordination, SLAM, physical docking detection, contactor control, and
real motor interfaces are not implemented in this small research project.

## Official API references consulted

- [ROS 2 Humble publisher/package documentation source](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.rst)
- [Humble Odometry message definition](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)
- [Humble Marker message definition](https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg)

These references establish API contracts. The project's application logic,
visuals, maps, descriptions, and tests were authored separately.
