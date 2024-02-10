# High-Speed Localization for Autonomous Race Cars in GNSS-Denied Scenarios

(Click on image to watch on YouTube)

[![video](https://img.youtube.com/vi/uOAymmNPiZs/maxresdefault.jpg)](https://www.youtube.com/watch?v=uOAymmNPiZs)

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
