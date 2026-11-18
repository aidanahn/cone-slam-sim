import numpy as np
import yaml

from cone_observation import ConeObservation
from pose2d import Pose2D


class ConeObservationModel:
    def __init__(self, cones: list[dict], initial_x: int, initial_y: int, config_path: str = 'config.yaml') -> None:
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)['cone_observation']
    
        self.cones = [((cone['x'] - initial_x) / 100.0, -(cone['y'] - initial_y) / 100.0) for cone in cones]

    def observe(self, pose: Pose2D) -> list[ConeObservation]:
        observations = []

        for global_x, global_y in self.cones:
            local_x, local_y = self.world_to_local(global_x, global_y, pose)
            distance = np.hypot(local_x, local_y)

            if distance < self.config['observation_radius'] and local_x >= -self.config['max_observation_distance_behind']:
                bearing = np.arctan2(local_y, local_x)

                distance += np.random.normal(0.0, scale=self.config['gaussian_noise_range_sigma'])
                bearing += np.random.normal(0.0, scale=self.config['gaussian_noise_bearing_sigma'])

                observations.append(ConeObservation(distance * np.cos(bearing), distance * np.sin(bearing)))

        return observations

    @staticmethod
    def world_to_local(global_x: float, global_y: float, pose: Pose2D) -> tuple[float, float]:
        dx = global_x - pose.x
        dy = global_y - pose.y

        local_x = dx * np.cos(pose.theta) + dy * np.sin(pose.theta)
        local_y = -dx * np.sin(pose.theta) + dy * np.cos(pose.theta)

        return local_x, local_y
    
