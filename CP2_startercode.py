import pybullet as p
import pybullet_data
import random
import math
from typing import List

random.seed(1)

### VARIABLES ###

# You can modify these variables for testing, but the final solution needs to be implemented with this setup #

GOAL = [4.5, 4.5]
GOAL_RADIUS = 0.3
START_POS = [1.5, 1.5, 0]
BLOCKS = [[1.5, 2.5], [4, 2], [3,5]]


### PYBULLET SETUP ###

def setup_pybullet(goal):
    """
    This function setups the pybullet environment. You do not need to modify anything in this function.
    """

    # Setup physics client and turtlebot
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf") 
    startPos = START_POS
    turtleId = p.loadURDF("turtlebot.urdf", startPos, globalScaling=2)
    sphereId = p.loadURDF("sphere.urdf", [goal[0], goal[1], 0.01], globalScaling=0.25)
    wallId = p.loadURDF("arena_walls6.urdf") 
    obstacles = [wallId]
    blocks = BLOCKS

    for block in blocks:
        blockId = p.loadURDF("block.urdf", [block[0], block[1], 0]) 
        obstacles.append(blockId)

    # set camera to bird eye view
    # note: can't set camera pitch to 90 exact -> gimble lock
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-89.99, cameraTargetPosition=[4, 4, 0])
    return turtleId, obstacles

### TURTLEBOT SETUP ##

class Turtlebot():
    """
    This class sets up your robot (a turtlebot!). 
    Helper functions to help you interact with the pybullet environment have been provided.
    You are welcome to modify any functions if you'd like as part of your solution but it is not necessary.
    """

    def __init__(self, turtleId, obstacles):
        self.position = None
        self.turtleId = turtleId
        self.obstacles = obstacles
        self.old_goals = []

    def get_position(self):
        """
        Inputs: none
        Outputs: turtlebot position as [x,y,z]
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return pos
    
    def get_orientation(self):
        """
        Inputs: none
        Outputs: turtlebot orientation as a quaternion
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return orn
    
    def collision_check(self):
        """
        Inputs: none
        Outputs: True/False, whether or not the turtlebot is in collision with a wall or block
        """
        collision_check = False
        for obstacle in self.obstacles: # check walls and blocks
            contacts = p.getContactPoints(bodyA=turtlebot.turtleId, bodyB=obstacle)
            if contacts:
                collision_check = True
                print(f"Collision detected at pose: {turtlebot.get_position()}")
                break
        return collision_check
        
    def teleport(self, pos, orn):
        """
        Teleports turtlebot to desired position/orientation

        Inputs:
        -pos: desired turtlebot positon
        -orn: desired turtlebot orientation

        Outputs: None
        """
        p.resetBasePositionAndOrientation(self.turtleId, pos, orn)

    def set_velocities(self, leftWheelVelocity, rightWheelVelocity):
        """
        Set turtlebot wheel velocities

        Inputs:
        -leftWheelVelocity
        -rightWheelVelocity

        Outputs: None
        """
        # set velocities
        p.setJointMotorControl2(self.turtleId,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
        p.setJointMotorControl2(self.turtleId,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

    def plot_path(self, color_string: str, path_points: List[List[float]]) -> None:
        """
        Debug function for plotting paths in pybullet

        Inputs:
        -color_string: color of path line
        -path_points: path line points to plot List[[x,y,z]]

        Outputs: None
        """

        # Supported colors
        # lineWidth is ignored on Mac (always 1px), so thickness comes from
        # drawing side-by-side copies spaced `offsets` meters apart.
        # Green sits higher so the red tree under it doesn't cover it.
        if color_string == "red":
            color = [1, 0, 0]
            line_width = 3
            offsets = [-0.01, 0.0, 0.01]
            height = 0.05
        elif color_string == "green":
            color = [0, 1, 0]
            line_width = 10
            offsets = [-0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03, 0.04]
            height = 0.15
        else:
            print("That color does not exist to be viewed. Not plotting Path.")
            return

        # Loop through path points and add debug line
        for i in range(1, len(path_points)):
            x0, y0, z0 = path_points[i-1][0], path_points[i-1][1], path_points[i-1][2] + height
            x1, y1, z1 = path_points[i][0],   path_points[i][1],   path_points[i][2] + height

            # Unit vector perpendicular to the segment, for the side-by-side copies
            length = math.hypot(x1 - x0, y1 - y0)
            if length == 0:
                continue
            nx, ny = -(y1 - y0) / length, (x1 - x0) / length

            for off in offsets:
                p.addUserDebugLine(
                    [x0 + nx * off, y0 + ny * off, z0],
                    [x1 + nx * off, y1 + ny * off, z1],
                    color,
                    lineWidth=line_width
                )
    
    def show_intermediate_goal(self, intermediate_goal: List[float]) -> None:
        """
        Show randomly generated goals in pybullet

        Inputs:
        -intermediate_goal: path line points to plot [x,y,z]

        Outputs: None
        """

        # Loop through old random goals and change them to black to show they are no longer the active goal
        size = 0.15
        for goal, goal_cross_1, goal_cross_2 in self.old_goals:
            p.removeUserDebugItem(goal_cross_1)
            p.removeUserDebugItem(goal_cross_2)
            goal_cross_1 = p.addUserDebugLine([goal[0] - size, goal[1], 0.1], [goal[0] + size, goal[1], 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([goal[0], goal[1] - size, 0.1], [goal[0], goal[1] + size, 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)

        # If new random goal provided, plot blue cross marking it.
        if intermediate_goal != None:
            goal_cross_1 = p.addUserDebugLine([intermediate_goal[0] - size, intermediate_goal[1], 0.1], [intermediate_goal[0] + size, intermediate_goal[1], 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([intermediate_goal[0], intermediate_goal[1] - size, 0.1], [intermediate_goal[0], intermediate_goal[1] + size, 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            self.old_goals.append((intermediate_goal, goal_cross_1, goal_cross_2))

    def sample_state(self, bounds, goal_bias = 0.5):
        """Return [x, y]: GOAL with 50% probability, else uniform random in the arena."""
        #Does everything above but its bounded by x/y min and  x/y max
        if random.random() < goal_bias:
            return[GOAL[0], GOAL[1]]
        x = random.uniform(bounds[0][0], bounds[0][1])
        y = random.uniform(bounds[1][0], bounds[1][1])
        return [x, y]
    
    def nearest_node(self, nodes, sample):
        """Return the index of the tree node closest (Euclidean x,y) to the sample."""
        closest_idx = 0
        closest_dist = float("inf")

        #Keep closest node in tree
        for i in range(len(nodes)):
            dx = sample[0] - nodes[i]["x"]
            dy = sample[1] - nodes[i]["y"]

            dist = math.sqrt(dx**2 + dy**2)

            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        return closest_idx

    def in_collision(self, x, y, th, margin=0.1):
        """Teleport robot to (x, y, th), run p.performCollisionDetection(),
        return True if touching any obstacle."""
        #Angle to quaternion for pybullet
        quat = p.getQuaternionFromEuler([0, 0, th])

        #Move robo to spot
        self.teleport([x, y, 0], quat)

        #Have pybullet update contacts
        p.performCollisionDetection()

        #See if robo is within margin of a block or wall
        #(margin gives the real robot room to drift without scraping)
        for obstacle in self.obstacles:
            close = p.getClosestPoints(bodyA = self.turtleId, bodyB = obstacle, distance = margin)
            if len(close) > 0:
                return True
        return False


    def propagate(self, node, v, w, duration, dt=0.05):
        """Integrate the unicycle model from node for `duration` seconds.
        Check collision at every small step.
        Return (x, y, th, traj) if collision-free, else None."""
        #Get starting node position and angle 
        x = node["x"]
        y = node["y"]
        th = node["th"]
        path = [[x, y, 0]]

        #Find out how many steps we need to take 
        steps = math.floor(duration / dt)

        #move robot forward and turn robot 
        for i in range(steps):
            x = x + v * dt * math.cos(th)
            y = y + v * dt * math.sin(th)
            th = th + w * dt 

            #Check to see if we hit something 
            if self.in_collision(x, y, th):
                return None

            #Save our current loc/path
            path.append([x, y, 0])
        return x, y, th, path

    def extend(self, nodes, near_idx, sample, n_props=5):
        """Run 5 propagations from nodes[near_idx] with random v, w, duration.
        Discard collisions, keep the one ending closest to the sample.
        Return the new node dict, or None if all 5 collided."""
        #This should work like steer but its trying 5 random moves and keeping the best one

        near_node = nodes[near_idx]

        closest_node = None
        closest_dist = float("inf")

        #Pick a random v, w, and time to drive for
        #Drive for ^
        #Check to see if crashed and keep the ones that dont
        #Keep closest sampled one
        for i in range(n_props):

            v = random.uniform(.2, .7)
            w = random.uniform(-1.5, 1.5)
            time = random.uniform(.5, 1.5)

            drive = self.propagate(near_node, v, w, time)

            if drive is None:
                continue

            x, y, th, path = drive

            #Draw valid paths
            self.plot_path("red", path)

            dx = sample[0] - x
            dy = sample[1] - y

            dist = math.sqrt(dx**2 + dy**2)

            #Find the closest one and update it 
            if dist < closest_dist:
                closest_dist = dist
                #Set key
                closest_node = {"x": x, "y": y, "th": th,
                                "parent": near_idx, "path": path,
                                "action": [v, w, time]}
        return closest_node

    def reached_goal(self, node, goal, radius):
        """True if the node is within radius of the goal."""
        #Check to see how far node is too goal
        dx = goal[0] - node["x"]
        dy = goal[1] - node["y"]
        dist = math.sqrt(dx**2 + dy**2)

        #Check to see if we are within the goal circle
        if dist <= radius:
            return True
        return False


    def extract_path(self, nodes, goal_idx):
        """Follow parent pointers back to the root.
        Return the list of [x, y, z] points from start to goal."""
        #Set up array of tree pieces 
        puzzle = []
        idx = goal_idx

        #follow up tree till we hit the start 
        while nodes[idx]["parent"] is not None:
            puzzle.append(nodes[idx]["path"])
            idx = nodes[idx]["parent"]

        #Start to goal 
        puzzle = puzzle[::-1]

        #Put path together
        puzzle_path = []

        for piece in puzzle:
            for point in piece:
                puzzle_path.append(point)
        return puzzle_path

### MAIN - implement your solution here! ###

if __name__ == "__main__":
    goal = GOAL
    # Setup pybullet
    turtleId, obstacles = setup_pybullet(goal)
    # Setup turtlebot
    turtlebot = Turtlebot(turtleId, obstacles)

    print('starting at {}'.format(turtlebot.get_position()))
    # Turn off real-time simulation for manual step control
    p.setRealTimeSimulation(0)

    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./search.mp4")

    #Each call draws one frame into the video
    def record_frame(n=1):
        for _ in range(n):
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)

    #Plan the whole path first, then drive it
    bounds = [[0.3, 5.7], [0.3, 5.7]]
    nodes = [{"x": START_POS[0], "y": START_POS[1], "th": 0.0,
              "parent": None, "path": [], "action": None}]
    goal_idx = None

    #Grow the tree till a node lands in the goal circle
    for it in range(1000):
        sample = turtlebot.sample_state(bounds)

        #Blue cross on random samples only 
        #show_intermediate_goal turns the older crosses black
        if sample != [GOAL[0], GOAL[1]]:
            turtlebot.show_intermediate_goal([sample[0], sample[1], 0])

        near_idx = turtlebot.nearest_node(nodes, sample)
        new_node = turtlebot.extend(nodes, near_idx, sample)

        record_frame(15)
        if new_node is None:
            continue
        nodes.append(new_node)
        if turtlebot.reached_goal(new_node, GOAL, GOAL_RADIUS):
            goal_idx = len(nodes) - 1
            print(f"Found goal after {it} iterations, {len(nodes)} nodes")
            break

    #Turn the last blue cross black now that the search is over
    turtlebot.show_intermediate_goal(None)

    if goal_idx is None:
        print("No path found :(")
    else: #extra credit bonus part section below!
        #draws solution, resets turtle, declares robot dimensions(from turtlebot.urdf)
        #picks points to drive to, then steers toward each point and drives to them
        #also prints data: velocities,
        path = turtlebot.extract_path(nodes, goal_idx)
        turtlebot.plot_path("green", path)
        record_frame(60)

        #in_collision teleported robo all over the map so put it back
        turtlebot.teleport(START_POS, p.getQuaternionFromEuler([0, 0, 0]))

        #Wheel radius and separation from turtlebot.urdf times globalScaling=2
        R = 0.07
        L = 0.46

        #Playing back v, w, time open loop drifts into blocks,
        #so instead steer toward every few points on the green path
        waypoints = path[::5] + [path[-1], [GOAL[0], GOAL[1], 0]]
        step = 0
        for target in waypoints:
            for _ in range(2000):
                x, y, _ = turtlebot.get_position()
                yaw = p.getEulerFromQuaternion(turtlebot.get_orientation())[2]
                dx = target[0] - x
                dy = target[1] - y
                if math.sqrt(dx**2 + dy**2) < 0.1:
                    break

                #Heading error wrapped to [-pi, pi]
                err = math.atan2(dy, dx) - yaw
                err = math.atan2(math.sin(err), math.cos(err))

                #Slow down while turning a lot
                v = 0.4 * max(0.0, math.cos(err))
                w = 3.0 * err

                left = (v - w * L / 2) / R
                right = (v + w * L / 2) / R
                turtlebot.set_velocities(leftWheelVelocity=left, rightWheelVelocity=right)

                p.stepSimulation()
                turtlebot.collision_check()

                step += 1
                if step % 4 == 0:
                    record_frame()
        turtlebot.set_velocities(0, 0)
        print(f"Ended at {turtlebot.get_position()}")
        record_frame(60)

    # Stop the recorording 
    p.stopStateLogging(log_id)
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 0)

    #Keep the window open after recording stops, so we can look at it do its thing
    while p.isConnected():
        p.stepSimulation()

    p.disconnect()
