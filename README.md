# SPIDeR

## Info about sonar

* Power Supply :+5V DC
* Quiescent Current : <2mA
* Working Current: 15mA
* Effectual Angle: <15°
* Ranging Distance : 2cm – 400 cm/1″ – 13ft
* Resolution : 0.3 cm
* Measuring Angle: 30 degree
* Trigger Input Pulse width: 10uS TTL pulse
* Echo Output Signal: TTL pulse proportional to the distance range
* Dimension: 45mm x 20mm x 15mm

## To run our code

- Run: `pip install -r requirements`
- Run: `sh camera`
- Run: `colcon build --symlink-install`
- Run: `source src/source.sh`
- Run: `sh src/camera_setup.sh`
- Run: `sh start_nodes.sh` (requires tmux)

## Files to review

- Under artifacts
  - File called good_robit.mp4 demonstrates our robot moving in a square.
  - File called keyboard_movement.mp4 demonstrates the robot being controlled by the keyboard.
  - File called imu_plot.png demonstrates a visualization of the square manuvure


## Assignment 2

### Files to review

- ./assignment2/Assignment2Report.docx
- ./assigment2/Monocular-Video-Odometry/images/odometry.png
- ./assigment2/Monocular-Video-Odometry/videos/VO_video.mp4
- Follow the odometry readme for running the video through the package