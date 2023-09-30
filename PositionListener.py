import roslibpy
import time
import matplotlib.pyplot as plt

x_positions = []
y_positions = []

def add_positions(pose_stamped):
  global x_positions
  global y_positions
  print(pose_stamped['pose']['position']['x'])
  print(pose_stamped['pose']['position']['y'])
  x_positions.append(pose_stamped['pose']['position']['x'])
  y_positions.append(pose_stamped['pose']['position']['y'])

client = roslibpy.Ros(host='127.0.0.1', port=9090)
client.run()
listener = roslibpy.Topic(client, '/position', 'geometry_msgs/PoseStamped')
listener.subscribe(add_positions)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()

plt.scatter(x_positions, y_positions)
plt.title("Robot Positions Plotted Againt X and Y Axis")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.show()
