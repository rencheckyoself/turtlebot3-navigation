# Homework 2 Answers

## Rotaion Only
### CCW

- Frac Val: 0.55
- Actual Velocity: 1.562 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: 15.374, 0, 0
- Final Pose from Fake Odom: -7.775, 0, 0
- Final Pose from Groundtruth: -8, 0, 0,
- Drift: , 0, 0


- Frac Val: 1
- Actual Velocity: 2.84 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: 157.471, 0, 0
- Final Pose from Fake Odom: -10.734, 0, 0
- Final Pose from Groundtruth: 135, 0, 0,
- Drift: , 0, 0

### CW

- Frac Val: 0.55
- Actual Velocity: 1.562 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: -16.208, 0, 0
- Final Pose from Fake Odom: 6.148, 0, 0
- Final Pose from Groundtruth: -5, 0, 0,
- Drift: , 0, 0


- Frac Val: 1
- Actual Velocity: 2.84 rad/s
- Expected Final Pose: 0, 0, 0
- Final Pose from Odom: -160.7, 0, 0
- Final Pose from Fake Odom: 10.7, 0, 0
- Final Pose from Groundtruth: 177, 0, 0,
- Drift: 0.85, 0, 0

### FWD

- Frac Val: 0.55
- Actual Velocity: 0.121 m/s
- Expected Final Pose: 0, 2, 0
- Final Pose from Odom: 0, 2.008, 0.008
- Final Pose from Fake Odom: 0, 2.002, 0
- Final Pose from Groundtruth: 5, 76.75", 1"
- Drift:


- Frac Val: 1
- Actual Velocity: 0.22 m/s
- Expected Final Pose: 0, 2, 0
- Final Pose from Odom: -0.163, 1.905, -0.004
- Final Pose from Fake Odom: 0, 2, 0

- Final Pose from Groundtruth: 5, 72", 0.5"
- Drift:

### BKWD

- Frac Val: 0.55
- Actual Velocity: 0.121 m/s
- Expected Final Pose: 0, -2, 0
- Final Pose from Odom: 0, 2.011, 0.004
- Final Pose from Fake Odom: 0, -2.002, 0
- Final Pose from Groundtruth: 0, 78.5", 0
- Drift:


- Frac Val: 1
- Actual Velocity: 0.22 m/s
- Expected Final Pose: 0, -2, 0
- Final Pose from Odom: 0, -1.991, 0
- Final Pose from Fake Odom: 0, -2.002, 0
- Final Pose from Groundtruth: 0, -74", 0
- Drift:

The encoders improve the odometry because it is a direct measurement of how the wheels are actually spinning for a given command.

Moving slower helps increase the accuracy of the odometry calculation because the acceleration is smaller which minimizes slipping.
