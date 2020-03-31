import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from lib.planning_utils import getOrigin, prunePath
from lib.voronoi_utils import a_star_graph, closest_node, create_grid_and_edges, create_graph, heuristic

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.global_goal = global_goal

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = int(np.ceil(self.global_goal[2]))
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE
        obstacle_file = 'worlds/colliders.csv'

        lat0, lon0 = getOrigin(obstacle_file)

        self.set_home_position(lon0, lat0, 0)
        local_home_ned = global_to_local(self.global_position, self.global_home)

        print('home position set to [N : {:2f}, E : {:2f} D : {:2f}]'.format(local_home_ned[0], local_home_ned[1], local_home_ned[2]))
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('worlds/colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        global_pos = (self._longitude, self._latitude, self._altitude)
        global_home = self.global_home

        local_pos = global_to_local(global_pos, global_home)
        grid_start = (int(local_pos[0]-north_offset), int(local_pos[1]-east_offset))
        
        # Set goal as some arbitrary position on the grid
        local_goal_ned = global_to_local(self.global_goal, self.global_home)
        grid_goal = (int(local_goal_ned[0]-north_offset), int(local_goal_ned[1]-east_offset))

        graph_ = create_graph(edges)

        graph_start = closest_node(graph_, grid_start)
        graph_goal = closest_node(graph_, grid_goal)

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star_graph(graph_, graph_start, graph_goal, heuristic)

        
        # Prune path 
        path = prunePath(path, epsilon=1e-2)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=float, default=-122.396591, help="Goal Longitude")
    parser.add_argument('--goal_lat', type=float, default=37.793405, help="Goal Latitude")
    parser.add_argument('--goal_alt', type=float, default=20.0, help="Goal Altitude")
    args = parser.parse_args()

    goal = (args.goal_lon, args.goal_lat, args.goal_alt)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    # MotionPlanning take connexion, goal coordinates and diagonal
    drone = MotionPlanning(conn, goal)
    time.sleep(1)

    drone.start()
