# cmu-16833-project
High-Speed Localization for Autonomous Race Cars in GNSS-Denied Scenarios


## Import Dependencies

```bash
vcs import . < iron.repos
```

## Install dependencies

```bash
source /opt/ros/iron/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

[Casadi source build instructions](https://github.com/casadi/casadi/wiki/InstallationLinux)

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```