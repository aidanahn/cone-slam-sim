from dataclasses import dataclass
from pose import Pose

@dataclass
class Edge:
    from_id: int
    to_id: int
    measurement: Pose