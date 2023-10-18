import matplotlib.pyplot as plt
import csv
import sys

x_positions = []
y_positions = []

with open(sys.argv[1], 'r') as file:
    csvreader = csv.reader(file)
    next(csvreader)

    for row in csvreader:
        x_positions.append(row[1])
        y_positions.append(row[2])

# plt.plot(x_positions, y_positions)
plt.scatter(x_positions, y_positions)
plt.title("Robot Positions Plotted Againt X and Y Axis")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.show()
