import sys

FSDS_LIB_PATH = r'C:\Projects\Formula-Student-Driverless-Simulator\python'
sys.path.insert(0, FSDS_LIB_PATH)

import fsds
import numpy as np

CONE_DETECTION_RANGE = 15
RADIAL_GAUSSIAN_NOISE = 0.05
ANGULAR_GAUSSIAN_NOISE = 0.007


def map_to_car_frame(cone_x, cone_y, car_x, car_y, car_yaw):
    offset_x = cone_x - car_x
    offset_y = cone_y - car_y

    cos_yaw = np.cos(car_yaw)
    sin_yaw = np.sin(car_yaw)

    local_x = cos_yaw * offset_x + sin_yaw * offset_y
    local_y = -sin_yaw * offset_x + cos_yaw * offset_y

    return local_x, local_y


def quaternion_to_yaw_rad(quaternion):
    x = quaternion.x_val
    y = quaternion.y_val
    z = quaternion.z_val
    w = quaternion.w_val

    norm = np.sqrt(x * x + y * y + z * z + w * w)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sin_yaw = 2 * (w * z + x * y)
    cos_yaw = 1 - 2 * (y * y + z * z)

    return np.arctan2(sin_yaw, cos_yaw)

try:
    client = fsds.FSDSClient()
    client.enableApiControl(False)

    referee = client.getRefereeState()
    cones = referee.cones

    cone_xs = [(cone['x'] - referee.initial_position.x) / 100.0 for cone in cones]
    cone_ys = [-(cone['y'] - referee.initial_position.y) / 100.0 for cone in cones]

    while True:
        car_kinematics = client.simGetGroundTruthKinematics()

        car_x = car_kinematics.position.x_val
        car_y = car_kinematics.position.y_val
        car_yaw = quaternion_to_yaw_rad(car_kinematics.orientation)

        detected_cones = []

        for cone_x, cone_y in zip(cone_xs, cone_ys):
            local_x, local_y = map_to_car_frame(cone_x, cone_y, car_x, car_y, car_yaw)

            distance = np.hypot(local_x, local_y)

            if distance < CONE_DETECTION_RANGE and local_x >= -1:
                angle = np.arctan2(local_y, local_x)

                noisy_distance = distance + np.random.normal(0.0, scale=RADIAL_GAUSSIAN_NOISE)
                noisy_angle = angle + np.random.normal(0.0, scale=ANGULAR_GAUSSIAN_NOISE)

                noisy_x = noisy_distance * np.cos(noisy_angle)
                noisy_y = noisy_distance * np.sin(noisy_angle)

                detected_cones.append((noisy_x, noisy_y))

except KeyboardInterrupt:
    client.reset()