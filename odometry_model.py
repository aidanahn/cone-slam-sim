import numpy as np
import yaml

from pose2d import Pose2D


class OdometryModel:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, config_path: str = 'config.yaml') -> None:
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)['odometry']
            
        self.pose = Pose2D(x, y, theta)

    def update_pose(self, vx: float, vy: float, wz: float, dt: float) -> Pose2D:
        dx = (vx + np.random.normal(0.0, scale=self.config['gaussian_noise_vx_sigma'])) * dt
        dy = (vy + np.random.normal(0.0, scale=self.config['gaussian_noise_vy_sigma'])) * dt
        dtheta = (wz + np.random.normal(0.0, scale=self.config['gaussian_noise_wz_sigma'])) * dt

        cos_theta = np.cos(self.pose.theta)
        sin_theta = np.sin(self.pose.theta)

        self.pose.x += dx * cos_theta - dy * sin_theta
        self.pose.y += dx * sin_theta + dy * cos_theta
        self.pose.theta = np.arctan2(np.sin(self.pose.theta + dtheta), np.cos(self.pose.theta + dtheta))

        return Pose2D(dx, dy, dtheta)

    def get_pose(self) -> Pose2D:
        return Pose2D(self.pose.x, self.pose.y, self.pose.theta)