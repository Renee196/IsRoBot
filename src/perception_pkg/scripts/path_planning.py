#!usr/bin/env python3

import rospy
from std_msgs.msg import Image
from nav_msgs.msg import Path
import cv2
import numpy as np
import heapq
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud
from pcl import PointXYZRGB
from pcl import PointXYZ
import pcl.pcl_visualization
import tf.transformations import
from custom_msgs.srv import PauseResume

class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def lidar_callback(msg):
    # Process Lidar data and extract obstacle location
    pc = pcl.PointCloud()
    pc.from_message(msg)

    # Example: Assuming obstacle_location is a tuple (obstacle_x, obstacle_y)
    obstacle_detected = True  # Set to True when obstacle is detected
    if obstacle_detected:
        # Assuming self is an instance of PathPlanningNode class
        self.update_obstacle_location(obstacle_location)

def astar(grid, start, end):
    open_set = []
    closed_set = set()
    heapq.heappush(open_set, (0, start))

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (end.x, end.y):
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node)

        for neighbor in get_neighbors(current_node, grid):
            if neighbor in closed_set:
                continue

            tentative_g = current_node.g + 1
            if neighbor not in open_set or tentative_g < neighbor.g:
                neighbor.parent = current_node
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, end)
                if neighbor not in open_set:
                    heapq.heappush(open_set, (neighbor.f(), neighbor))

    return None

def get_neighbors(node, grid):
  neighbors = []
    x, y = node.x, node.y
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if (0 <= nx < len(grid)) and (0 <= ny < len(grid[0])) and (grid[nx][ny] == 0):
            neighbors.append(Node(nx, ny))

    return neighbors

def heuristic(node, end):
    return abs(node.x - end.x) + abs(node.y - end.y)

    def follow_path(path):
    dt = 1
    velocities = []

    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]

        if abs(x1 - x0) < 1e-6:
            omega = 0
        else:
            omega = math.atan2((y1 - y0), (x1 - x0)) / dt

        l = 0.5
        v_avg = 5
        L = np.array([[1, l/2], [1, -l/2]])
        V = np.array([[v_avg], [omega]])
        v = np.dot(L, V)

        velocities.append(tuple(v))

    return np.asarray(velocities, np.float64)

def update_path_for_obstacle(path, obstacle_location):
    updated_path = path.copy()

    for i in range(len(path) - 1):
        line_start = np.array([path[i][0], path[i][1]])
        line_end = np.array([path[i + 1][0], path[i + 1][1]])

        # Check if the line segment intersects with the obstacle
        if is_intersect(line_start, line_end, obstacle_location):
            # Recalculate path by rerunning A* algorithm from the current position to the end
            updated_path = updated_path[:i+1] + calculate_new_path(path[i], end_node, grid, obstacle_location)
            break

    return updated_path


def is_intersect(line_start, line_end, obstacle_location):
    # Check if the line segment intersects with the circular obstacle
    obstacle_radius = 0.5  # Adjust the radius based on your obstacle size
    obstacle_center = np.array(obstacle_location)

    d = line_end - line_start
    f = line_start - obstacle_center

    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - obstacle_radius**2

    discriminant = b**2 - 4*a*c

    if discriminant < 0:
        return False  # No intersection
    else:
        return True  # Intersection

def calculate_new_path(start, end, grid, obstacle_location):
    # Run A* algorithm to calculate a new path
    new_start_node = Node(start[0], start[1])
    new_end_node = Node(end.x, end.y)

    # Mark the cell containing the obstacle as occupied in the grid
    obstacle_x, obstacle_y = obstacle_location
    grid[obstacle_x][obstacle_y] = 1

    new_path = astar(grid, new_start_node, new_end_node)

    # Reset the obstacle cell in the grid
    grid[obstacle_x][obstacle_y] = 0

    return new_path

class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planner_node')
        rospy.Subscriber('/nav_msgs/OccupancyGrid', OccupancyGrid , self.gridMap_callback)
        rospy.Subscriber('/depth_estimation/QRDistInfo', QRDistInfo, self.qrDistInfo_callback)
        rospy.Subscriber('/geometry_msgs/PoseWithCovarianceStamped', PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber('/sensor_msgs/LaserScan', LaserScan , self.lidar_callback)
        self.Path=None
        self.path_publisher = rospy.Publisher('nav_msgs/Path', self.Path, queue_size=10)
        self.map = None
        self.depth_map = None
        self.robot_pose = []
        self.qr_code_positions = []
        self.obstacle_location = None  # Initialize obstacle_location

    def pause_resume_client(self,pause):
        rospy.wait_for_service('pause_resume')
        try:
          pause_resume=rospy.ServiceProxy('pause_resume', PauseResume)
          response=pause_resume(pause)
          return response.success
        except rospy.ServiceException as e:
          rospy.logerr("Service call failed: %s"%e)
          return False

    def gridMap_callback(self,msg):
        self.map=msg

    def qrDistInfo_callback(self,msg):
        self.qr_code_positions=msg

    def pose_callback(self,msg):
        quaternion=[msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler= euler_from_quaternion(quaternion)
        self.robot_pose=[msg.position.x, msg.position.y, euler[2]]

    def lidar_callback(self, msg):
        # Process Lidar data and extract obstacle location
        pc = pcl.PointCloud()
        pc.from_message(msg)

        outlier_removed = self.remove_ground(pc)

        # Segment the scene using a planar model
        segmented_planes = self.segment_planes(outlier_removed)

        # Extract obstacles using Euclidean clustering
        obstacles = self.extract_obstacles(segmented_planes)

        if obstacles:
            # Extract the centroid of the largest obstacle
            largest_obstacle = max(obstacles, key=lambda x: len(x))
            obstacle_location = self.calculate_obstacle_centroid(largest_obstacle)

            # Update obstacle_location in the PathPlanningNode instance
            self.update_obstacle_location(obstacle_location)

    def remove_ground(self, pc):
        # Statistical outlier removal filter to remove ground points
        outlier_removed = pc.make_statistical_outlier_filter()
        outlier_removed.set_mean_k(50)
        outlier_removed.set_std_dev_mul_thresh(1.0)
        outlier_removed.filter()
        return outlier_removed

    def segment_planes(self, pc):
        # Segment planes in the scene
        seg = pc.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        inliers, _ = seg.segment()
        segmented_planes = pc.extract(inliers, negative=True)
        return segmented_planes

    def extract_obstacles(self, pc):
        # Euclidean clustering to extract obstacles
        tree = pc.make_kdtree()
        ec = pc.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.2)  # Adjust based on scene
        ec.set_MinClusterSize(10)     # Adjust based on scene
        ec.set_MaxClusterSize(5000)   # Adjust based on scene
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        obstacles = []
        for j, indices in enumerate(cluster_indices):
            cluster = pc.extract(indices)
            obstacles.append(cluster)

        return obstacles

    def calculate_obstacle_centroid(self, obstacle_cloud):
        # Calculate the centroid of the obstacle point cloud
        centroid = pcl.pcl_visualization.CloudViewing()
        centroid.ShowMonochromeCloud(obstacle_cloud)
        centroid.Spin()
        return centroid

    def update_obstacle_location(self, obstacle_location):
        # Update obstacle_location in the PathPlanningNode instance
        self.obstacle_location = obstacle_location

        # Assuming you have access to the necessary variables and methods
        if self.map is not None and self.robot_pose is not None:
            start_node = (self.robot_pose[0], self.robot_pose[1])
            goal_node = (self.obstacle_location[0], self.obstacle_location[1])

            # Using A* for path planning
            self.Path = self.AStar(start_node, goal_node, self.map)

            if self.Path:
                self.follow_path(self.Path)
                rospy.loginfo("Updated path to avoid obstacle")

    def AStar(self, start_pt, goal_pt, grid):
        return astar(grid, start_pt, goal_pt)

    def plan_path(self):
        current_location = self.robot_pose
        unvisited_qr_codes = self.qr_code_positions

        while not rospy.is_shutdown():
            if self.map is not None and self.depth_map is not None and self.qr_code_positions:
                if not unvisited_qr_codes:
                    rospy.loginfo("All QR Codes visited. Exiting Path Planning..")
                    break

                nearest_qr_code = unvisited_qr_codes[0]
                start_node = Node(current_location[0], current_location[1])
                goal_node = Node(nearest_qr_code[0], nearest_qr_code[1])

                # Using A* for path planning
                self.Path = self.AStar(start_node, goal_node, self.map)

                if self.Path:
                  self.follow_path(self.Path)
                  rospy.loginfo("A* Path Found:", self.Path)
                  velocities = follow_path(self.Path)
                  rospy.loginfo("Velocities:", velocities)

                if current_node==nearest_qr_code:
                  unvisited_qr_codes.pop(0)
                  rospy.loginfo("Reached QR code {}".format(nearest_qr_code))

                # Simulate obstacle avoidance
                obstacle_detected = True  # Set to True when obstacle is detected
                if obstacle_detected:
                    updated_path = update_path_for_obstacle(self.Path, obstacle_location)
                    rospy.loginfo("Updated Path:", updated_path)
                    velocities=follow_path(updated_path)
                    rospy.loginfo(velocities)

    def follow_path(self, self.Path):
        # Kinematics calculation to calculate and control speed of motors
        return follow_path(self.Path)

if __name__ == '__main__':
    try:
        planner_node = PathPlanningNode()
        planner_node.plan_path() #main fn for the node logic flow
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
