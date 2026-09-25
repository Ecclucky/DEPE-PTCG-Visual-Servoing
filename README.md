# Dual-Estimator Secure Predictive Visual Servoing for Quadrotors Under Hybrid Attacks

**Authors:** Enci Wang, Yang Yi, Guangdeng Zong, Jun Yang.

**Target journal:** IEEE Transactions on Industrial Electronics (TIE).

This ROS 1 workspace contains the first set of real-flight experimental code
accompanying the manuscript prepared for submission to TIE. It brings together
visual sensing, four control methods, the quadrotor flight interface, and
experiment recording within a common visual-servo platform.

The onboard RealSense camera observes an AprilTag target. The visual sensing
node provides relative-position measurements to the selected controller, which
produces commands for the PX4/MAVROS flight interface:

```text
RealSense / AprilTag -> Visual measurements -> Selected controller
                                             -> PX4 control interface -> MAVROS / PX4
```

Motion capture supports vehicle odometry and experiment evaluation; recording
utilities save controller telemetry and camera images for subsequent analysis.

The workspace contains four methods:

| Method | ROS package |
| --- | --- |
| DEPE–PTCG (proposed) | `depe_ptcg` |
| MRSFC | `MultiRate_Hybrid_attacks` |
| PHBSC | `PHBSC` |
| PID | `pid` |

The four controllers share the sensing and flight interfaces. Select one
controller for each experiment.
Vehicle calibration and flight-platform configuration should match the
experimental hardware.

This work was supported by the National Natural Science Foundation of China
under Grants 92371116, 62433005, and 62303400.
