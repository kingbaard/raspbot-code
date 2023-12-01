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
