import os
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

class Environment:
    """Manages the QLabs simulation environment setup."""
    def __init__(self, qlabs):
        self.qlabs = qlabs
        self.x_offset = 0.13
        self.y_offset = 1.67

    def setup(self):
        """Sets up the QLabs environment with flooring, walls, crosswalk, and spline."""
        self.qlabs.destroy_all_spawned_actors()
        QLabsRealTime().terminate_all_real_time_models()

        # Setup flooring
        hFloor = QLabsQCarFlooring(self.qlabs)
        hFloor.spawn_degrees([self.x_offset, self.y_offset, 0.001], rotation=[0, 0, -90], configuration=0)

        # Setup walls
        hWall = QLabsWalls(self.qlabs)
        hWall.set_enable_dynamics(False)
        for y in range(5):
            hWall.spawn_degrees(location=[-2.4 + self.x_offset, (-y*1.0)+2.55 + self.y_offset, 0.001], rotation=[0, 0, 0])
        for x in range(5):
            hWall.spawn_degrees(location=[-1.9+x + self.x_offset, 3.05 + self.y_offset, 0.001], rotation=[0, 0, 90])
        for y in range(6):
            hWall.spawn_degrees(location=[2.4 + self.x_offset, (-y*1.0)+2.55 + self.y_offset, 0.001], rotation=[0, 0, 0])
        for x in range(4):
            hWall.spawn_degrees(location=[-0.9+x + self.x_offset, -3.05 + self.y_offset, 0.001], rotation=[0, 0, 90])
        hWall.spawn_degrees(location=[-2.03 + self.x_offset, -2.275 + self.y_offset, 0.001], rotation=[0, 0, 48])
        hWall.spawn_degrees(location=[-1.575 + self.x_offset, -2.7 + self.y_offset, 0.001], rotation=[0, 0, 48])

        # Setup crosswalk and spline
        myCrossWalk = QLabsCrosswalk(self.qlabs)
        myCrossWalk.spawn_degrees(location=[-2 + self.x_offset, -1.475 + self.y_offset, 0.01], rotation=[0, 0, 0], scale=[0.1, 0.1, 0.075], configuration=0)
        mySpline = QLabsBasicShape(self.qlabs)
        mySpline.spawn_degrees(location=[2.05 + self.x_offset, -1.5 + self.y_offset, 0.01], rotation=[0, 0, 0], scale=[0.27, 0.02, 0.001], waitForConfirmation=False)

    def spawn_vehicles(self):
        """Spawns the leader and follower vehicles."""
        leader = QLabsQCar2(self.qlabs)
        follower = QLabsQCar2(self.qlabs)
        leader.spawn_id(actorNumber=0, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
        follower.spawn_id(actorNumber=1, location=[-1.735, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
        return leader, follower

    def start_real_time(self):
        """Starts the real-time model for the leader."""
        rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
        QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)

    def cleanup(self):
        """Cleans up the QLabs environment."""
        self.qlabs.close()