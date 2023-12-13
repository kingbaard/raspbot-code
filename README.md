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
- Run: `colcon build --symlink-install`
- Follow this guide to set up camera [https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304)
- Source generated `setup.bash`
- Run: `sh src/camera_setup.sh`

### Run Nodes
- camera:`ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"`
- apriltag_detection: `ros2 run raspbot apriltag`
- sonar node: `ros2 run raspbot sonar`
- downward-facing IR sensors: `ros2 run raspbot ir`
- buzzer: `ros2 run raspbot buzzer`
- motors/state machine: `ros2 run raspbot motors`
- keyboard control node (e-stop): `ros2 run raspbot keyboard`

### Start State Machine Loop
`ros2 topic pub -r 100 /warehouse_control std_msgs/msg/Bool "{ data: True }"`

### Interesting topics to monitor:
- `/apriltags`: displays information about detected apriltags
- `/sonar`: displays the current sonar distance

## Files to review
- [apriltag.py](./src/raspbot/raspbot/apriltag.py) - apriltag detection node
- [keyboard.py](./src/raspbot/raspbot/keyboard.py) - keyboard control node (for e-stop/debugging)
- [motors.py](./src/raspbot/raspbot/motors.py) - state machine + motor control **The Brain**
- [sonar.py](./src/raspbot/raspbot/sonar.py) - the sonar node
- [sonar.py](./src/raspbot/raspbot/ir.py) - the IR node
- [sonar.py](./src/raspbot/raspbot/buzzer.py) - the buzzer node