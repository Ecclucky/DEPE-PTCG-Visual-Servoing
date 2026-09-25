# Third-party components

This workspace includes dependencies inherited with the experimental code.
Paper authorship does not imply authorship of these dependencies. The table
records the information supplied by the local source files; no upstream version
or commit has been inferred.

| Component | Attribution retained in supplied files | License declaration |
| --- | --- | --- |
| `src/px4ctrl` | Original maintainer `uav` (`iszhouxin@zju.edu.cn`) | GPLv3 in `package.xml`; full text in `LICENSE` |
| `src/vrpn_client_ros` | Paul Bovbel / Clearpath Robotics copyright and maintainer notices | BSD in package metadata and source headers |
| `src/utils/uav_utils` | LIU Tianbo | LGPLv3 in `package.xml` |
| `src/utils/cmake_utils` | William.Wu | LGPLv3 in `package.xml` |
| `src/utils/quadrotor_msgs` | Kartik Mohta | BSD in `package.xml` |

Individual files may carry additional notices. In particular, the bundled
`FindEigen.cmake` retains its Google/Ceres Solver BSD notice and Alex Stewart
attribution, and `FindNLopt.cmake` retains its DART BSD-style notice.

The original `quadrotor_msgs` maintainer email is an upstream placeholder and is
retained as provenance, not presented as a support contact. For this experimental
workspace, contact Enci Wang at `dx120220100@stu.yzu.edu.cn`.

Local preparation changes include removal of unused bundled packages, generated
files and stale backup files; cleanup of comments; dependency declarations and
build configuration corrections; explicit launch arguments; and Python 3 syntax
for optional thrust-calibration and utility scripts. Controller settings and numerical
flight logic are retained. Existing copyright headers and supplied license text
remain in place. The remaining shared message definitions and utility headers
are retained to preserve the dependency interface.

ROS, MAVROS, Eigen, OpenCV, ViSP, librealsense2, VRPN, and the PID controller's
IIR filter library are external dependencies
and are not relicensed by this repository. Several bundled packages provide only
a license declaration in their manifests; this list is not a substitute for
the corresponding license text or confirmation of their exact upstream revision.
