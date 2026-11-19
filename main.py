import math
import sys
import time
import numpy as np
import yaml
import gtsam
from gtsam.symbol_shorthand import X, L

FSDS_LIB_PATH = r'C:\Projects\Formula-Student-Driverless-Simulator\python'
sys.path.insert(0, FSDS_LIB_PATH)

import fsds

from cone_observation_model import ConeObservationModel
from odometry_model import OdometryModel
from pose2d import Pose2D
from graph_slam import GraphSLAM


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


def main():
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    client = fsds.FSDSClient()

    try:
        referee = client.getRefereeState()

        odometry_model = OdometryModel()
        cone_observation_model = ConeObservationModel(
            referee.cones,
            referee.initial_position.x,
            referee.initial_position.y
        )

        graph = GraphSLAM()

        previous_time = time.perf_counter()
        next_odometry_measurement_time = previous_time
        next_cone_observation_time = previous_time

        while True:
            kinematics = client.simGetGroundTruthKinematics()
            current_time = time.perf_counter()

            if current_time >= next_odometry_measurement_time:
                dt = current_time - previous_time
                previous_time = current_time

                pose_delta = odometry_model.update_pose(
                    kinematics.linear_velocity.x_val,
                    kinematics.linear_velocity.y_val,
                    kinematics.angular_velocity.z_val,
                    dt
                )

                graph.add_odometry(pose_delta, dt)

                next_odometry_measurement_time = (
                    current_time + (1.0 / config['odometry']['rate_hz'])
                )
            
            if current_time >= next_cone_observation_time:
                cone_observations = cone_observation_model.observe(
                    Pose2D(
                        kinematics.position.x_val,
                        kinematics.position.y_val,
                        quaternion_to_yaw_rad(kinematics.orientation)
                    )
                )

                next_cone_observation_time = (
                    current_time + (1.0 / config['cone_observation']['rate_hz'])
                )

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()