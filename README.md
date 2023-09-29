# Using the Docker Container

## Placing the ros2_nodes folder as /root/ros2_ws/src

This will make it so your files don't get deleted when the container ends

```sh
docker run -it --rm --privileged -v ${PWD}/ros2_nodes:/root/ros2_ws/src humble
```


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

- cd into ```~/raspbot-code/```
- Build docker container
  - Run ```docker build .```
- Run docker container
  - ```docker run -it --rm --privileged -v ${PWD}/ros2_nodes:/root/ros2_ws/src <docker container id>```
- cd into ```/root/ros2_ws```
- Run: ```colcon build --symlink-install```
- Run: ```source src/source.sh```
- Run motors first. It must be running for any of the following to run.
  - ```ros2 run raspbot motors```
- To run keyboard controls (wasd for movement and any other key to stop moving):
  - ```ros2 run raspbot keyboard```
- To run square:
  - ```ros2 topic pub /drive_square_control std_msgs/msg/Bool "{ data: True }" --once```
- To run imu:
  - Make sure you are running motors
  - Then run:
    - ```ros2 run raspbot imu```
    - ```ros2 bag record position```
    - Then run the command to run the square.
  - You will now have the imu information in the same directory as you ran the record in.
  - You may export this data with docker cp and scp.
