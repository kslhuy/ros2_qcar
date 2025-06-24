import math
import numpy as np
import threading
import time
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

class Follower:
    def __init__(self, qcar, k_steering=0.5, max_steering=0.6, max_speed=1, safe_distance=1):
        self.qcar = qcar
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.min_speed = 0.5
        self.safe_distance = safe_distance
        
        self.K_p_obstacle=0.8
        
        self.running = False
        self.thread = None
        
        self.utils = Utils()

        lidarPlot = pg.plot(title="LIDAR")

        squareSize = 10

        lidarPlot.setXRange(-squareSize, squareSize)

        lidarPlot.setYRange(-squareSize, squareSize)

        self.lidarData = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
        self.detected = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)
        self.target = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolPen=None, symbolSize=10)
        self.target2 = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='y', symbolPen=None, symbolSize=10)
        # Occupancy grid (added as image item)
        self.occupancy_img = pg.ImageItem()
        self.occupancy_img.setZValue(-10)  # Draw under LiDAR points
        lidarPlot.addItem(self.occupancy_img)

        
        self.L = 3
        self.CELLS_PER_METER = 20
        self.grid_width_meters = 6
        self.grid_height = int(self.L * self.CELLS_PER_METER)
        self.grid_width = int(self.grid_width_meters * self.CELLS_PER_METER)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
       
        self.IS_OCCUPIED = 100
        self.IS_FREE = 50
        
    def local_to_grid(self, x, y):
        i = int(y * -self.CELLS_PER_METER + (self.grid_height - 1))
        j = int(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)

    def local_to_grid_parallel(self, x, y):
        i = np.round(y * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def grid_to_local(self, point):
        i, j = point[0], point[1]
        y = (i - (self.grid_height - 1)) / -self.CELLS_PER_METER
        x = (j - self.CELL_Y_OFFSET) / self.CELLS_PER_METER
        return (x, y)
        
    def populate_occupancy_grid(self, ranges, thetas):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid.

        Optimization performed to improve the speed at which we generate the occupancy grid.

        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        # reset empty occupacny grid (-1 = unknown)

        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=self.IS_FREE, dtype=int)

        ranges = np.array(ranges)
        valid = ranges > 0
        ranges = ranges[valid]
        thetas = thetas[valid]
        xs = ranges * np.sin(thetas)
        ys = ranges * np.cos(thetas)

        i, j = self.local_to_grid_parallel(xs, ys)

        occupied_indices = np.where((i > 0) & (i < self.grid_height) & (j > 0) & (j < self.grid_width))
        self.occupancy_grid[i[occupied_indices], j[occupied_indices]] = self.IS_OCCUPIED
        
        
    # === Occupancy Grid Plot Update Function ===
    def update_occupancy_image(self, cell_size=1/20):
        """
        Update the PyQtGraph image layer with new occupancy grid data.
        """
        # Flip vertically so image matches LiDAR frame orientation
        image = np.flipud(self.occupancy_grid).T  # transpose for (x,y) alignment
        self.occupancy_img.setImage(image, levels=(0, 100))

        # Scale image so each cell matches real-world meters
        shape = self.occupancy_grid.shape
        self.occupancy_img.setRect(
            pg.QtCore.QRectF(
                -shape[1] / 2.0 * cell_size,
                0,
                shape[1] * cell_size,
                shape[0] * cell_size
            )
        )

    def drive_to_target(self, point, K_p):
        """
        Using the pure pursuit derivation

        Improvement is that we make the point closer when the car is going at higher speeds

        """
        # calculate curvature/steering angle
        L = np.linalg.norm(point)
        y = point[1]
        angle = K_p * (2 * y) / (L**2)
        angle = angle -np.radians(90)
        angle = np.clip(angle, -self.max_steering, self.max_steering)

        # determine velocity
        if self.obstacle_detected:
        # if True:
            if np.degrees(angle) < 10.0:
                velocity = self.max_speed
            elif np.degrees(angle) < 20.0:
                velocity = (self.max_speed + self.min_speed) / 2
            else:
                velocity = self.min_speed

        else:
            # Set velocity to velocity of racing line
            velocity = self.max_speed

        self.qcar.set_velocity_and_request_state(
            forward=velocity,
            turn=angle,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )
        
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def check_collision(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        One of the issues is that if the starting cell is next to a wall, then it already considers there to be a collision.
        See check_collision_loose


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (cell_a[0], cell_a[1] + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    #print(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def check_collision_loose(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        This looser implementation only checks half way for meeting the margin requirement.


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (int((cell_a[0] + cell_b[0]) / 2), int((cell_a[1] + cell_b[1]) / 2) + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    #print(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def direction_to_leader(self, follower_pos, follower_yaw, leader_pos):
        """
        Get normalized direction vector from follower to leader in follower frame.

        Args:
            follower_pos (tuple): (x, y) in world frame
            follower_rot (float): yaw in radians
            leader_pos (tuple): (x, y) in world frame

        Returns:
            (dx, dy): unit direction vector in follower's local frame
        """
        dx = leader_pos[0] - follower_pos[0]
        dy = leader_pos[1] - follower_pos[1]

        # Normalize
        norm = math.hypot(dx, dy)
        if norm == 0:
            return 0.0, 0.0
        dx /= norm
        dy /= norm

        # Transform to follower's local frame (rotate by -yaw)
        rel_x = math.cos(follower_yaw) * dx - math.sin(follower_yaw) * dy
        rel_y = math.sin(follower_yaw) * dx + math.cos(follower_yaw) * dy

        return rel_x, rel_y

    def follow(self, point):
        
        # _, pos_leader, rot_leader, _ = leader.get_world_transform()    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        
        # Vector from follower to leader
        dx = point[0] - pos_follower[0]
        dy = point[1] - pos_follower[1]
        distance = math.hypot(dx, dy)

        # Follower heading and target direction
        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = target_angle - follower_heading
        heading_error = self.wrap_to_pi(heading_error)

        # Steering command
        steering_cmd = -self.k_steering * heading_error
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Speed control: Stop if too close
        if distance < self.safe_distance:
            speed_cmd = 0.0
        else:
            speed_cmd = min(self.max_speed, 0.5 * (distance - self.safe_distance))

        # Move follower
        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )
        
    def avoid_obstacle_lidar2(self, point, sample_points=400):
        
        success, angles, distances = self.qcar.get_lidar(samplePoints=sample_points)
        if not success:
            print("not success")
            return
        
        self.populate_occupancy_grid(distances, angles)
        self.update_occupancy_image()
        x = np.sin(angles)*distances

        y = np.cos(angles)*distances


        self.lidarData.setData(x,y)

        QtWidgets.QApplication.instance().processEvents()
        
        # Focus on front-facing region (±45°)
        fov_mask1 = np.abs(angles) < np.radians(45)
        fov_mask2 = np.abs(angles) > np.radians(315)
        fov_mask = np.logical_or(fov_mask1, fov_mask2)
        
        close_mask1 = distances < 1
        close_mask2 = distances > 0
        close_mask = np.logical_and(close_mask1, close_mask2)
        
        danger_mask = np.logical_and(fov_mask, close_mask)

        # Obstacle detected — steer away
        danger_angles = angles[danger_mask]
        
        #
        danger_distances = distances[danger_mask]
        x = np.sin(danger_angles)*danger_distances

        y = np.cos(danger_angles)*danger_distances
        self.detected.setData(x,y)
        
        
        current_pos = np.array(self.local_to_grid(0, 0))
        
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        dx, dy = self.direction_to_leader(pos_follower, -rot_follower[2], point)
        
        dx_rot = -dy
        dy_rot = dx
        self.target.setData([dx_rot], [dy_rot])
        self.target2.setData([], [])

        goal_pos = np.array(self.local_to_grid(dx_rot, dy_rot))
        target = None
        MARGIN = 5

        self.update_occupancy_image()
        if self.check_collision(current_pos, goal_pos, margin=MARGIN):
            self.obstacle_detected = True

            shifts = [i * (-1 if i % 2 else 1) for i in range(1, 21)]

            found = False
            for shift in shifts:
                # We consider various points to the left and right of the goal position
                new_goal = goal_pos + np.array([0, shift])

                # If we are currently super close to the wall, this logic doesn't work
                if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                    target = self.grid_to_local(new_goal)
                    found = True
                    print("Found condition 1")
                    break

            if not found:
                # This means that the obstacle is very close to us, we need even steeper turns
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                        target = self.grid_to_local(new_goal)
                        found = True
                        print("Found condition 2")
                        break

            if not found:
                # Try again with a looser collision checker, we are probably very close to the obstacle, so check only collision free in the second half
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision_loose(current_pos, new_goal, margin=MARGIN):
                        target = self.grid_to_local(new_goal)
                        found = True
                        print("Found condition 3")
                        break

        else:
            self.obstacle_detected = False
            target = self.grid_to_local(goal_pos)
            
        if target:
            if self.obstacle_detected:
                self.drive_to_target(target, self.K_p_obstacle)
                self.target2.setData([target[0]], [target[1]])
                return True
            else:
                return False
        else:
            self.qcar.set_velocity_and_request_state(
                forward=0, 
                turn=0,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
            return True
            
    def avoid_obstacle_lidar(self, min_distance=1, sample_points=400):
        """
        Reads LiDAR and applies basic obstacle avoidance by steering away.

        Args:
            qcar: the QCar object
            min_distance (float): distance to start avoiding
            max_turn (float): maximum steering rate
            sample_points (int): LiDAR resolution
        """
        success, angles, distances = self.qcar.get_lidar(samplePoints=sample_points)
        if not success:
            print("not success")
            return
        
        self.populate_occupancy_grid(distances, angles)
        self.update_occupancy_image()
        x = np.sin(angles)*distances

        y = np.cos(angles)*distances


        self.lidarData.setData(x,y)

        QtWidgets.QApplication.instance().processEvents()
        
        # Focus on front-facing region (±45°)
        fov_mask1 = np.abs(angles) < np.radians(45)
        fov_mask2 = np.abs(angles) > np.radians(315)
        fov_mask = np.logical_or(fov_mask1, fov_mask2)
        
        close_mask1 = distances < min_distance
        close_mask2 = distances > 0
        close_mask = np.logical_and(close_mask1, close_mask2)
        
        danger_mask = np.logical_and(fov_mask, close_mask)
    
        if not np.any(danger_mask):
            self.detected.setData([],[])
            print("not np.any(danger_mask)")
            return False

        print("detected")
        # Obstacle detected — steer away
        danger_angles = angles[danger_mask]
        
        #
        danger_distances = distances[danger_mask]
        x = np.sin(danger_angles)*danger_distances

        y = np.cos(danger_angles)*danger_distances
        self.detected.setData(x,y)
        #
        
        # Compute mean angle of obstacles
        mean_angle = self.wrap_to_pi(np.mean(danger_angles))

        # Turn away: if obstacle on left (angle > 0), turn right (negative)
        turn_rate = -np.clip(mean_angle, -np.radians(60), np.radians(60)) / np.radians(60) * self.max_steering

        # Slow down near obstacles
        self.qcar.set_velocity_and_request_state(
            forward=0.6, 
            turn=self.k_steering *turn_rate,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )
        
        return True
        
    def run(self, leader):
        """Start following leader in a loop."""
        self.running = True
        while self.running:        
            _, pos_leader, _, _ = leader.get_world_transform()    
            _, pos_follower, _, _ = self.qcar.get_world_transform()
        
            # Vector from follower to leader
            dx = pos_leader[0] - pos_follower[0]
            dy = pos_leader[1] - pos_follower[1]
            distance = math.hypot(dx, dy)
            if distance < self.safe_distance:
                continue
            
            if self.avoid_obstacle_lidar2(pos_leader):
                continue
            
            self.follow(pos_leader)
            time.sleep(0.1)  # Small delay to simulate a realistic loop, adjust as needed

    def start(self, leader):
        """Start the follower loop in a new thread."""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        """Stop the follower loop."""
        self.running = False
        if self.thread is not None:
            self.thread.join()
            

class Utils:
    def __init__(self):
        pass

    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points