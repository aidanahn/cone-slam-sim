import gtsam
from gtsam.symbol_shorthand import X, L
import yaml
import matplotlib.pyplot as plt
import io
import graphviz
import matplotlib.image as mpimg

from pose2d import Pose2D


class GraphSLAM:
    def __init__(self) -> None:
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

        self.graph = gtsam.NonlinearFactorGraph()
        self.pose_index = 0

        prior_noise = gtsam.noiseModel.Constrained.All(3)
        self.graph.add(gtsam.PriorFactorPose2(
            X(0),
            gtsam.Pose2(0.0, 0.0, 0.0),
            prior_noise
        ))

    def add_odometry(self, pose_delta: Pose2D, dt: float) -> None:
        odem_noise = gtsam.noiseModel.Diagonal.Sigmas([
            self.config['odometry']['gaussian_noise_vx_sigma'] * dt,
            self.config['odometry']['gaussian_noise_vy_sigma'] * dt,
            self.config['odometry']['gaussian_noise_wz_sigma'] * dt
        ])

        self.graph.add(gtsam.BetweenFactorPose2(
            X(self.pose_index),
            X(self.pose_index + 1),
            gtsam.Pose2(pose_delta.x, pose_delta.y, pose_delta.theta ),
            odem_noise
        ))

        self.pose_index += 1