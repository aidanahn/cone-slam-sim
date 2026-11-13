from pose import Pose

class GraphSLAM:
    def __init__(self):
        self.poses = []

    def add_pose(self, pose: Pose) -> int:
        node_id = len(self.poses)
        self.poses.append(pose)
        return node_id

    def add_edge(self):
        