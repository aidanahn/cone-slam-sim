from pose import Pose
from edge import Edge

class GraphSLAM:
    def __init__(self):
        self.poses = []
        self.odom_edges = []

    def add_pose(self, pose: Pose) -> int:
        node_id = len(self.poses)
        self.poses.append(pose)
        return node_id

    def add_edge(self, from_id: int, to_id: int, measurement) -> None:
        edge = Edge(from_id, to_id, measurement)
        self.odom_edges.append(edge)