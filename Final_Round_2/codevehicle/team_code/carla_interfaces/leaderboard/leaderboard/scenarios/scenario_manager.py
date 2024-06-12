#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementations.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import signal
import sys
import time

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.traffic_events import TrafficEventType

from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider

PENALTY_COLLISION_PEDESTRIAN = 0.50
PENALTY_COLLISION_VEHICLE = 0.60
PENALTY_COLLISION_STATIC = 0.65
PENALTY_TRAFFIC_LIGHT = 0.70
PENALTY_STOP = 0.80


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, run and stop a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, timeout, debug_mode=False):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)

        # Detects if the simulation is down
        self._watchdog = Watchdog(self._timeout)
        # Stop the agent from freezing the simulation
        self._agent_watchdog = Watchdog(self._timeout)

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        # Register the scenario tick as callback for the CARLA world
        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)

        self.traffic_event = []

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError(
                "Agent took longer than {}s to send its command".format(self._timeout))
        elif self._watchdog and not self._watchdog.get_status():
            raise RuntimeError(
                "The simulation took longer than {}s to update".format(self._timeout))
        self._running = False

    def cleanup(self):
        """
        Reset all parameters
        """
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        self._spectator = None

    def load_scenario(self, scenario, agent, rep_number):
        """
        Load a new scenario
        """

        GameTime.restart()
        self._agent = AgentWrapper(agent)
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.repetition_number = rep_number

        self._spectator = CarlaDataProvider.get_world().get_spectator()

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._agent_watchdog.start()
        self._running = True

        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                # TODO: Remove this hack to be able to run leaderboard properly
                self._tick_scenario(timestamp, world)

    def _tick_scenario(self, timestamp, world):
        """
        Run next tick of scenario and the agent and tick the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            self._watchdog.pause()

            try:
                self._agent_watchdog.resume()
                self._agent_watchdog.update()
                # Hack to get access to ego.
                ego_action = self._agent(world, self.ego_vehicles[0])
                self._agent_watchdog.pause()

            # Special exception inside the agent that isn't caused by the agent
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)

            self._watchdog.resume()
            self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(
                    self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

            ego_trans = self.ego_vehicles[0].get_transform()
            self._spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=50),
                                                          carla.Rotation(pitch=-90)))

        if self._running and self.get_running_status():
            CarlaDataProvider.get_world().tick(self._timeout)

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        self._watchdog.stop()
        self._agent_watchdog.stop()

        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

        if self.get_running_status():
            if self.scenario is not None:
                self.scenario.terminate()

            if self._agent is not None:
                self._agent.cleanup()
                self._agent = None

            self.analyze_scenario()

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        global_result = '\033[92m'+'SUCCESS'+'\033[0m'

        for criterion in self.scenario.get_criteria():
            if criterion.test_status != "SUCCESS":
                global_result = '\033[91m'+'FAILURE'+'\033[0m'

            self.traffic_event.extend(criterion.list_traffic_events)

        if self.scenario.timeout_node.timeout:
            global_result = '\033[91m'+'FAILURE'+'\033[0m'

        ResultOutputProvider(self, global_result)

    def get_coderace_statistics(self):
        route_completion = None
        outside_lanes = None
        collisions = None
        lights_ran = None
        stops_ran = None
        duration = round(self.scenario_duration_game, 2)

        target_reached = False
        score_penalty = 1.0
        score_route = 0.0

        for criterion in self.scenario.get_criteria():
            actual_value = criterion.actual_value
            name = criterion.name

            if name == 'RouteCompletionTest':
                route_completion = float(actual_value)
            elif name == 'OutsideRouteLanesTest':
                outside_lanes = float(actual_value)
            elif name == 'CollisionTest':
                collisions = int(actual_value)
            elif name == 'RunningRedLightTest':
                lights_ran = int(actual_value)
            elif name == 'RunningStopTest':
                stops_ran = int(actual_value)

            if criterion.list_traffic_events:
                for event in criterion.list_traffic_events:
                    if event.get_type() == TrafficEventType.COLLISION_STATIC:
                        score_penalty *= PENALTY_COLLISION_STATIC

                    elif event.get_type() == TrafficEventType.COLLISION_PEDESTRIAN:
                        score_penalty *= PENALTY_COLLISION_PEDESTRIAN

                    elif event.get_type() == TrafficEventType.COLLISION_VEHICLE:
                        score_penalty *= PENALTY_COLLISION_VEHICLE

                    elif event.get_type() == TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION:
                        score_penalty *= (1 - event.get_dict()
                                          ['percentage'] / 100)

                    elif event.get_type() == TrafficEventType.ROUTE_COMPLETED:
                        score_route = 100.0
                        target_reached = True
                    elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                        if not target_reached:
                            if event.get_dict():
                                score_route = event.get_dict()[
                                    'route_completed']
                            else:
                                score_route = 0

        score = max(score_route*score_penalty, 0.0)

        return route_completion, \
            outside_lanes, \
            collisions, \
            lights_ran, \
            stops_ran, \
            duration, \
            score
