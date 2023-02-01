import numpy as np
from enum import Enum
from queue import PriorityQueue
from collections import deque
import matplotlib.pyplot as plt
import csv
from skimage.morphology import medial_axis
from skimage.util import invert
from sklearn.neighbors import KDTree

class PlanningProblem:
    def __init__(self, filename, safety=4):
        self.mapfile = filename
        self.mapdata = np.loadtxt(filename, delimiter=',', skiprows=2)
        self.elevation_map = np.array([[]])
        self.grids = {}
        self.medialaxis_grids = {}
        self.voronoi_graphs = {}
        self.prms = {}
        self.voxel_maps = {}
        self.potential_fields = {}
        self.rrts = {}
        self.mapdims = []
        self.map_nedboundary = []
        self.offsets = []
        self.geodetic = []
        self.waypoints_to_send = [[]]
        self.plans = []
        self.safety_distance = safety
        
        self.take_measurements()
        
        with open(filename) as f:
            reader = csv.reader(f)
            a = next(reader)
        self.lat0 = float(a[0].split('lat0 ')[1])
        self.lon0 = float(a[1].split(' lon0 ')[1])
        
    #########################################################################################################
    #### Build Maps #########################################################################################
    ##########################################################################################################
    def build_elevation_map(self, mapdata):
        #elevation_map = np.zeros(self.mapdata.shape)
        pass
    
    def take_measurements(self):
        # First, record the minimum and maximum NED coordinates of the obstacle array.
        r = [0,0,0,0,0,0]
        r[0] = int(np.floor(np.amin(self.mapdata[:,0] - self.mapdata[:,3]))) - self.safety_distance #North min
        r[1] = int(np.ceil(np.amax(self.mapdata[:,0] + self.mapdata[:,3]))) + self.safety_distance #North max
        r[2] = int(np.floor(np.amin(self.mapdata[:,1] - self.mapdata[:,4]))) - self.safety_distance #East min
        r[3] = int(np.ceil(np.amax(self.mapdata[:,1] + self.mapdata[:,4]))) + self.safety_distance #East max
        r[4] = int(np.floor(np.amin(self.mapdata[:,2] - self.mapdata[:,5]))) - self.safety_distance #Alt min
        r[5] = int(np.ceil(np.amax(self.mapdata[:,2] + self.mapdata[:,5]))) + self.safety_distance #Alt max
        
        # Determine NED offsets by taking the smallest north value and the smallest east value.
        offset = [0,0,0]
        offset[0] = int(np.amin(self.mapdata[:,0])) #North offset
        offset[1] = int(np.amin(self.mapdata[:,1])) #East offset
        offset[2] = 0

        # Calculate the size of the configuration space next.
        s = [0,0,0]
        s[0] = r[1] - r[0] 
        s[1] = r[3] - r[2]
        s[2] = r[5] - r[4]
        self.mapdims = s
        self.map_nedboundary = r
        self.offsets = offset
        
    def build_grid(self, elevation):
        """
        Updates the grids attribute dictionary with key: elevation and value: nxm numpy array, Float64, 0 = Free Space, 1 = Obstacle.
        """
        grid = np.zeros((self.mapdims[0], self.mapdims[1]), dtype=float)
        obstacle = [0,0,0,0]
        for i in range(self.mapdata.shape[0]):
            north, east, down, d_north, d_east, d_down = self.mapdata[i,:]          
            height = down + d_down
            if elevation < down + d_down:
                obstacle = [
                    int(north - self.offsets[0] - d_north - self.safety_distance),
                    int(north - self.offsets[0] + d_north + self.safety_distance),
                    int(east - self.offsets[1] - d_east - self.safety_distance),
                    int(east - self.offsets[1] + d_east + self.safety_distance)
                ]

                grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
        self.grids[elevation] = grid

    def build_medialaxis_grid(self, grid, elevation):
        skeleton = medial_axis(invert(grid))
        skeleton = invert(skeleton)
        self.medialaxis_grids[elevation] = skeleton

    def build_voronoi_graph(self, elevation):
        pass
    def build_prm_graph(self, elevation, num_samples):
        pass
    def build_potential_field_grid(self, elevation):
        pass
    def build_voxel_map(self, center_location, elevation, dx=10, dy=10, dz=10):
        pass
    def build_rrt(self, elevation, num_vertices):
        pass
    
    #########################################################################################################
    # Search Maps ###########################################################################################
    #########################################################################################################
    def search_grid(self, grid_elevation, heuristic, start, goal):
        grid = self.grids[grid_elevation]
        path, cost = self.astar_grid(grid, heuristic, start, goal)
        gridnodes = self.path_to_gridnodes(start, path)
        gridnodes = self.remove_collinear(gridnodes)
        self.plot_gridnodes(gridnodes, grid_elevation, 'Grid, A*, Diagonals allowed')
        plan = self.gridnodes_to_plan(gridnodes, grid_elevation, heading=0)
        self.plans.append(plan)
    def search_medialaxis_grid(self, grid_elevation, heuristic, start, goal):
        grid = self.medialaxis_grids[grid_elevation]
        freenodes = self.list_freegridnodes(grid, grid_elevation)
        start = self.find_kneighbors(start, freenodes, 1)
        goal = self.find_kneighbors(goal, freenodes, 1)
        path, cost = self.astar_grid(grid, heuristic, start, goal)
        gridnodes = self.path_to_gridnodes(start, path)
        gridnodes = self.remove_collinear(gridnodes)
        self.plot_gridnodes(gridnodes, grid_elevation, 'Grid, Medial-axis transformed, A*, Diagonals allowed')
        plan = self.gridnodes_to_plan(gridnodes, grid_elevation, heading=0)
        self.plans.append(plan)
    def search_graph(self, graph, start, goal):
        pass
    def send_waypoints(self, waypoints):
        self.waypoints = waypoints   
        
    #############################################################################################################
    # Planning utilities ########################################################################################
    #############################################################################################################
    def list_freegridnodes(self, grid, elevation):
        freenodes = []
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 0:
                    freenodes.append([i,j])
        return freenodes
    
    def find_kneighbors(self, node, nodes, k):
        N = KDTree(nodes)
        node = list(node)
        dist, ind = N.query([node], k=k)
        n = []
        for i in range(len(ind[0])):
            n.append(tuple(nodes[ind[0][i]]))
        if k == 1:
            return n[0]
        else:
            return n
        
    def plot_gridnodes(self, gridnodes, elevation, plan_type):
        X = []
        Y = []
        for node in gridnodes:
            X.append(node[1])
            Y.append(node[0])
        plt.figure(figsize=(8.0, 8.0))
        plt.imshow(self.grids[elevation], origin='lower', cmap='Greys')
        plt.plot(X, Y, 'bo-')
        plt.xlabel('Meters East of {0}m'.format(int(self.map_nedboundary[2])))
        plt.ylabel('Meters North of {0}m'.format(int(self.map_nedboundary[0])))
        plt.title('Motion Plan: {2}, Latitude: {0}, Longitude: {1}'.format(self.lat0, self.lon0, plan_type))
        
    def local_to_grid(self, local_position):
        """
        Converts an NED local position into a grid position. 
        """
        x = local_position[0] - self.map_nedboundary[0]
        y = local_position[1] - self.map_nedboundary[2]
        grid_position = [int(x), int(y)]
        
        return grid_position
    
    def remove_collinear(self, gridnodes):
        """
        This function is an algorithm that removes unneeded collinear waypoints. 
        Input: gridnodes, n x 3 numpy array
        Returns: culled_waypoints, n x 3 numpy array
        """
        i = 0
        while i+2 < len(gridnodes):
            
            x1 = gridnodes[i][0]
            y1 = gridnodes[i][1]
            x2 = gridnodes[i+1][0]
            y2 = gridnodes[i+1][1]
            x3 = gridnodes[i+2][0]
            y3 = gridnodes[i+2][1]
            
            #points = np.array([[x1, y1, 1],[x2, y2, 1],[x3, y3, 1]])
            #collinear = np.linalg.det(points) <= np.abs(1e-6)
            collinear = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2) == 0
            if collinear: 
                del(gridnodes[i+1])
            else:
                i += 1
        return gridnodes
    
    def euclidean_heuristic(self, p1, p2):
        return np.linalg.norm(np.array(p2) - np.array(p1))
    
    def manhattan_heuristic(self, p1, p2):
        return np.abs(p2[1]-p1[1]) + np.abs(p2[0]-p1[0])
    
    def path_to_gridnodes(self, start, path):
        current_node = list(start)
        gridnodes = [current_node]
        for action in path:
            dx = action.value[0]
            dy = action.value[1]
            next_node = [current_node[0] + dx, current_node[1] + dy]
            gridnodes.append(next_node)
            current_node = next_node
        return gridnodes
    
    def gridnodes_to_plan(self, gridnodes, elevation, heading=0):
        x = self.map_nedboundary[0]
        y = self.map_nedboundary[2]
        plan = []
        for gridnode in gridnodes:
            plan.append([x + gridnode[0], y + gridnode[1], elevation, heading])
        return plan
    
    def astar_grid(self, grid, h, start, goal):
        """
        Executes A Star Search given a grid, heuristic array, a start goal, and an end goal. 
        Returns the path if one is found and the associated cost. 
        """
        # First, initializes data structures.
        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)
        branch = {}
        found = False

        # Then, begin the A Star. The algorithm will expand the lowest queue-cost node, where queue-cost
        # is the sum of the heuristic cost and the path cost. 
        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            print(item[0])
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                for action in self.valid_actions(grid, current_node):
                    print(action)
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)

                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))

        # After, retraces steps, returning the step-by-step actions and total path-cost required to get there.
        if found:
            n = goal
            path_cost = branch[n][0]
            while branch[n][1] != start:
                path.append(branch[n][2])
                n = branch[n][1]
            path.append(branch[n][2])
        else:
            print('************')
            print('Failed to find a path.')
            print('************')

        # Return a step-by-step list of actions taken from start to goal, given one is found. Also return the total associated path cost. 
        return path[::-1], path_cost

    def valid_actions(self, grid, current_node):
        """
        Returns a list of valid actions given a grid and given a node. Input grid is a nxm bool numpy array, representing the configuration space.
        Input current_node is a grid cell tuple. This function returns a list of valid actions, from the potential actions defined in the action class,
        stemming from the given cell.
        """

        # This function defines some key parameters first.
        potential = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.NORTHEAST, Action.NORTHWEST, Action.SOUTHEAST, Action.SOUTHWEST]
        #potential = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
        valid = []
        n, m = grid.shape[0] - 1, grid.shape[1] - 1
        x, y = current_node

        # Next, it iterates through each potential action, only considering it as a valid action if the action results in a location that is both on-grid and
        # non-obstacle.
        for action in potential:
            dx, dy = action.value[0], action.value[1]
            rx, ry = x + dx, y + dy
            if rx <= n and rx >= 0 and ry <=m and ry >= 0 and grid[rx, ry] != True:
                valid.append(action)

        # Finally, this function returns a list of valid actions. 
        print(valid)    
        return valid

#######################################################################################################################
# Action class used in search #########################################################################################
#######################################################################################################################

class Action(Enum):
    """
    This class defines the action set. Each action is a 3-element tuple where the first two elements describe
    the vertical and horizontal grid motions or translations, while the final element of the tuple describes
    the action's cost. 
    
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    NORTHEAST = (-1, 1, np.sqrt(2))
    NORTHWEST = (-1, -1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.NORTHEAST:
            return '*'
        elif self == self.NORTHWEST:
            return '*'
        elif self == self.SOUTHEAST:
            return '*'
        elif self == self.SOUTHWEST:
            return '*'
        
    @property
    def cost(self):
        return self.value[2]
    @property
    def delta(self):
        return (self.value[0], self.value[1])
    

    


