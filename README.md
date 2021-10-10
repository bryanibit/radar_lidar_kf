## Intro

The project is useed for processing lidar and radar data to get fusion position results. Lidar data is processed to (x, y) in vehicle coordinate. And radar can produce (phi, theta, velocity) and it can be transformed to ($x$, $y$, $v_x$, $v_y$). And the whole filter can be achieved by linear kalman fliter after transforming radar data to Cartesian coordinate. Kalman filter use global coordinate of target detected by radar or lidar in kalman filter. `data` file is used in `main.cc`, which has real, measurement and predicted by KF data.

1. The kernel program is 'radar_lidar_kf.cc' and 'kalman_filter.cc', which are compiled to shared library called `radar_lidar_kf`.
2. For testing the library, we use main.cc and data/.txt to get results and difference.

## Use

```sh
mkdir build
cd script
./start.sh
```
