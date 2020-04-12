'''
Now fly trajectories without errors with a cascaded controller !
'''

import argparse
import time
from enum import Enum

import numpy as np
import matplotlib.pyplot as plt

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

from lib.unity_drone import UnityDrone
from lib.controller import NonlinearController


class Phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(UnityDrone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_phase = Phases.MANUAL

        # list of logs
        self.xlog = []
        self.ylog = []
        self.zlog = []

        self.vxlog = []
        self.vylog = []
        self.vzlog = []

        self.controller = NonlinearController()

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
        self.register_callback(MsgID.RAW_GYROSCOPE, self.gyro_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)

    def attitude_callback(self):
        if self.flight_phase == Phases.WAYPOINT:
            self.attitude_controller()

    def gyro_callback(self):
        if self.flight_phase == Phases.WAYPOINT:
            self.bodyrate_controller()

    def velocity_callback(self):
        self.vxlog.append(self.local_velocity[0])
        self.vylog.append(self.local_velocity[1])
        self.vzlog.append(-1 * self.local_velocity[2])

        if self.flight_phase == Phases.WAYPOINT:
            self.position_controller()

        if self.flight_phase == Phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1)) and abs(self.local_position[2] < 0.01):
                self.disarming_transition()

    def local_position_callback(self):
        self.xlog.append(self.local_position[0])
        self.ylog.append(self.local_position[1])
        self.zlog.append(-1 * self.local_position[2])

        if self.flight_phase == Phases.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                (self.position_trajectory, self.time_trajectory, self.yaw_trajectory) = self.load_test_trajectory(time_mult=0.5)
                self.all_waypoints = self.position_trajectory.copy()
                self.waypoint_number = -1
                self.waypoint_transition()

        elif self.flight_phase == Phases.WAYPOINT:
            if time.time() > self.time_trajectory[self.waypoint_number]:

                if (len(self.all_waypoints) > 0):
                    self.waypoint_transition()
                else:
                    if (self.local_velocity[0] < 1.0 and self.local_velocity[1] < 1.0):
                        print("local position " + str(self.local_position))
                        self.landing_transition()

    def state_callback(self):
        if not self.in_mission:
            return

        if self.flight_phase == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == Phases.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_phase == Phases.DISARMING:
            if not self.armed:
                self.manual_transition()

    def position_controller(self):
        """Sets the local acceleration target using the local position and local velocity"""

        (self.local_position_target, self.local_velocity_target, yaw_cmd) = self.controller.trajectory_control(
            self.position_trajectory, self.yaw_trajectory, self.time_trajectory, time.time())
        self.attitude_target = np.array((0.0, 0.0, yaw_cmd))

        acceleration_cmd = self.controller.lateral_position_control(
            self.local_position_target[0:2], self.local_velocity_target[0:2], self.local_position[0:2], self.local_velocity[0:2])
        self.local_acceleration_target = np.array([acceleration_cmd[0], acceleration_cmd[1], 0.0])

    def attitude_controller(self):
        """Sets the body rate target using the acceleration target and attitude"""
        self.thrust_cmd = self.controller.altitude_control(
            -self.local_position_target[2], -self.local_velocity_target[2], -self.local_position[2], -self.local_velocity[2], self.attitude, 9.81)
        roll_pitch_rate_cmd = self.controller.roll_pitch_controller(
            self.local_acceleration_target[0:2], self.attitude, self.thrust_cmd)
        yawrate_cmd = self.controller.yaw_control(self.attitude_target[2], self.attitude[2])
        self.body_rate_target = np.array([roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yawrate_cmd])

    def bodyrate_controller(self):  
        """Commands a moment to the vehicle using the body rate target and body rates"""
        moment_cmd = self.controller.body_rate_control(self.body_rate_target, self.gyro_raw)
        self.cmd_moment(moment_cmd[0], moment_cmd[1], moment_cmd[2], self.thrust_cmd)

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_phase = Phases.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_phase = Phases.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        print("target position " + str(self.target_position))
        self.local_position_target = np.array(
            (self.target_position[0], self.target_position[1], self.target_position[2]))
        self.waypoint_number += 1
        self.flight_phase = Phases.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_phase = Phases.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_phase = Phases.DISARMING

    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = Phases.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


def plot_positions(drone_object):
    fig, axs = plt.subplots(3, sharex=True, sharey=True)
    fig.suptitle("Position at each Callbacks")
    axs[0].plot(drone_object.xlog, 'tab:orange')
    axs[0].set_title("X Position")
    axs[1].plot(drone_object.ylog, 'tab:green')
    axs[1].set_title("Y Position")
    axs[2].plot(drone_object.zlog, 'tab:blue')
    axs[2].set_title("Z Position")


def plot_velocities(drone_object):
    fig, axs = plt.subplots(3, sharex=True)
    fig.suptitle("Velocities at each Callbacks")
    axs[0].plot(drone_object.vxlog, 'tab:orange')
    axs[0].set_title("X Velocity")
    axs[1].plot(drone_object.vylog, 'tab:green')
    axs[1].set_title("Y Velocity")
    axs[2].plot(drone_object.vzlog, 'tab:blue')
    axs[2].set_title("Z Velocity")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    drone.test_trajectory_file = './lib/trajectory_test.txt'
    time.sleep(2)
    drone.start()

    plot_positions(drone)
    plot_velocities(drone)

    plt.show()
