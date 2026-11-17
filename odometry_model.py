import numpy as np
import yaml

from pose2d import Pose2D


class OdometryModel:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, config_path: str = 'config.yaml') -> None:
        self.pose = Pose2D(x, y, theta)

        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)['odometry']

    def update_pose(self, vx: float, vy: float, wz: float, dt: float) -> None:
        self.pose.x += (vx + np.random.normal(0.0, scale=self.config['gaussian_noise_vx_sigma'])) * dt
        self.pose.y += (vy + np.random.normal(0.0, scale=self.config['gaussian_noise_vy_sigma'])) * dt
        self.pose.theta += (wz + np.random.normal(0.0, scale=self.config['gaussian_noise_wz_sigma'])) * dt
        self.pose.theta = np.arctan2(np.sin(self.pose.theta), np.cos(self.pose.theta))

    def get_pose(self) -> Pose2D:
        return Pose2D(self.pose.x, self.pose.y, self.pose.theta)