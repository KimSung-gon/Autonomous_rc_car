echo RUN ROS Viewer ===========================
roslaunch razor_imu_9dof display-imu.launch &
roslaunch rplidar_ros display_lidar.launch &
roslaunch zed_wrapper display_zed.launch &
roslaunch usb_cam display_cam.launch &
echo IMU / LiDAR / ZED / CAM - Background Running ===

