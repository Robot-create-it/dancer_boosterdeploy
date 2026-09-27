# K1 pass vision runtime

Migrated from `dancer-robocupdemo/src/vision` and
`src/robocup_ros2_interface/src/vision_interface`. The message definitions are
unchanged. Detector and ball projection retain the demo implementation.

Local adaptations: optional segmentation disabled by default; guard disabled
data logging; resolve model paths against the installed package; reject images
without a synchronized head pose/depth; remove unused robot SDK linkage;
calibration tools are excluded from the normal build. Their optional AprilTag
dependencies are not bundled. Models included: `k1_realsense_0120.engine` and
`best_digua_1223_10.3.engine`. TensorRT engines need a compatible Jetson/TensorRT
runtime; select the calibrated model named by the robot's configuration.

Build on the robot after loading ROS Humble and the Booster interface:

```bash
cd vision_ws
colcon build --symlink-install --packages-select vision_interface vision \
  --cmake-args -DBUILD_TESTING=OFF -DBUILD_CALIBRATION=OFF
source install/setup.bash
```

Requires CUDA/nvcc, TensorRT, OpenCV, PCL, yaml-cpp, ROS image_transport and tf2.
Do not overlay a second `vision_interface` version after loading this workspace.
Camera drivers and `/head_pose` are supplied by the robot platform. Startup
checks require live data; no uncalibrated head-pose approximation is published.

See `../docs/k1_pass_camera_head_testing.md` for the complete procedure.
