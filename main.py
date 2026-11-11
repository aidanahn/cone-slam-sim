import sys

FSDS_LIB_PATH = r"C:\Projects\Formula-Student-Driverless-Simulator\python"
sys.path.insert(0, FSDS_LIB_PATH)

import fsds
import matplotlib.pyplot as plt

CONE_COLORS = {
    0: 'yellow',
    1: 'blue',
    2: 'orange'
}
MARKER_SIZE = 10
PLOT_REFRESH_INTERVAL = 0.01

try:
    client = fsds.FSDSClient()
    client.enableApiControl(False)

    referee = client.getRefereeState()
    cones = referee.cones
    origin_x, origin_y = referee.initial_position.x, referee.initial_position.y

    x = [(cone['x'] - origin_x) / 100.0 for cone in cones]
    y = [-(cone['y'] - origin_y) / 100.0 for cone in cones]
    color = [CONE_COLORS[cone['color']] for cone in cones]
    
    plt.ion()
    vehicle = plt.scatter(0, 0, c='red', s=MARKER_SIZE)
    plt.scatter(x, y, c=color, s=MARKER_SIZE)
    plt.axis("equal")
    plt.show()

    while True:
        ground_truth = client.simGetGroundTruthKinematics()
        
        vehicle.set_offsets([[
            ground_truth.position.x_val,
            ground_truth.position.y_val
        ]])

        plt.pause(PLOT_REFRESH_INTERVAL)

except KeyboardInterrupt:
    client.reset()