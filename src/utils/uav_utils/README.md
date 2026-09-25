# uav_utils

Vendored, header-only ROS utilities used by `px4ctrl` for coordinate conversion
and geometry operations. The original maintainer and LGPLv3 declaration are
retained in `package.xml`.

The local CMake configuration locates Eigen through the bundled `cmake_utils`
package and exports the `include` directory. Consumers must also provide Eigen
include directories. The existing geometry tests are built when
`CATKIN_ENABLE_TESTING` is enabled; no `uav_utils` library is required.

The remaining scripts are optional odometry/frame inspection helpers inherited
with this dependency. They are not controllers or experiment entry points.
