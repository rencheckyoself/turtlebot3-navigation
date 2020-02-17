# Homework 2 Answers

Values recorded as (th, x, y) in degrees and meters

## Rotaion Only
### CCW

- Frac Val: 0.55
- Actual Velocity: 1.562 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: 15.374, 0, 0
- Final Pose from Fake Odom: -7.775, 0, 0
- Final Pose from Groundtruth: -8, 0, 0,
- Drift: 1.1687, 0, 0


- Frac Val: 1
- Actual Velocity: 2.84 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: 157.471, 0, 0
- Final Pose from Fake Odom: -10.734, 0, 0
- Final Pose from Groundtruth: 135, 0, 0,
- Drift: 1.12355, 0, 0

### CW

- Frac Val: 0.55
- Actual Velocity: 1.562 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: -16.208, 0, 0
- Final Pose from Fake Odom: 6.148, 0, 0
- Final Pose from Groundtruth: -5, 0, 0,
- Drift: 0.5604, 0, 0


- Frac Val: 1
- Actual Velocity: 2.84 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: -160.7, 0, 0
- Final Pose from Fake Odom: 10.7, 0, 0
- Final Pose from Groundtruth: 177, 0, 0,
- Drift: 1.15, 0, 0

### FWD

- Frac Val: 0.55
- Actual Velocity: 0.121 m/s
- Expected Final Pose: 0, 2, 0
- Final Pose from Odom: 0, 2.008, 0.008
- Final Pose from Fake Odom: 0, 2.002, 0
- Final Pose from Groundtruth: 5, 1.949, 0.025
- Drift: 0.25, 0.0029, 0.00087


- Frac Val: 1
- Actual Velocity: 0.22 m/s
- Expected Final Pose: 0, 2, 0
- Final Pose from Odom: -0.163, 1.905, -0.004
- Final Pose from Fake Odom: 0, 2, 0
- Final Pose from Groundtruth: 5, 1.829, 0.013
- Drift: 0.258, 0.00381, 0.000835

### BKWD

- Frac Val: 0.55
- Actual Velocity: 0.121 m/s
- Expected Final Pose: 0, -2, 0
- Final Pose from Odom: 0, -2.011, 0.004
- Final Pose from Fake Odom: 0, -2.002, 0
- Final Pose from Groundtruth: 0, -1.9939, 0
- Drift: 0, 0.000855, 0.0002


- Frac Val: 1
- Actual Velocity: 0.22 m/s
- Expected Final Pose: 0, -2, 0
- Final Pose from Odom: 0, -1.991, 0
- Final Pose from Fake Odom: 0, -2.002, 0
- Final Pose from Groundtruth: 0, -1.8796, 0
- Drift: 0, 0.00557, 0

### Waypoint Following

The total path distance is 4.414m
Relative to the odom frame, the robot finished: 0.102, 0.140 off from the original starting position

The encoders improve the odometry because it is a direct measurement of how the wheels are actually spinning for a given command.

Moving slower helps increase the accuracy of the odometry calculation because the acceleration is smaller which minimizes slipping.
