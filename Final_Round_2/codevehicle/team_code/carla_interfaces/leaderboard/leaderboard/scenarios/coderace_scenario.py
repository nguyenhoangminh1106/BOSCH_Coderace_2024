from __future__ import print_function

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorSpeedAboveThresholdTest)

from leaderboard.scenarios.route_scenario import convert_transform_to_location
from leaderboard.utils.route_manipulation import location_route_to_gps, _get_latlon_ref
from .route_scenario import RouteScenario

WEATHERS = [
    carla.WeatherParameters.ClearNoon,
    carla.WeatherParameters.ClearSunset,
]


class CodeRaceScenario(RouteScenario):
    category = "CodeRaceScenario"

    def __init__(self, world, agent, start_idx, target_idx, debug_mode=0, criteria_enable=True):

        # Overwrite
        self.list_scenarios = []

        self.town_name = world.get_map().name
        self.start_idx = start_idx
        self.target_idx = target_idx

        self.agent = agent

        # Set route
        self._set_route()

        ego_vehicle = self._update_ego_vehicle()

        BasicScenario.__init__(self, name=f'CodeRace_{self.town_name}_s{start_idx}_t{target_idx}',
                               ego_vehicles=[ego_vehicle],
                               config=None,
                               world=world,
                               debug_mode=debug_mode > 1,
                               terminate_on_failure=False,
                               criteria_enable=criteria_enable
                               )

        self.list_scenarios = []

    def _set_route(self, hop_resolution=1.0):

        world = CarlaDataProvider.get_world()
        dao = GlobalRoutePlannerDAO(world.get_map(), hop_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        spawn_points = CarlaDataProvider._spawn_points

        start = spawn_points[self.start_idx]
        target = spawn_points[self.target_idx]

        route = grp.trace_route(start.location, target.location)
        self.route = [(w.transform, c) for w, c in route]

        CarlaDataProvider.set_ego_vehicle_route(
            [(w.transform.location, c) for w, c in route])
        gps_route = location_route_to_gps(self.route, *_get_latlon_ref(world))
        self.agent.set_global_plan(gps_route, self.route)

        self.timeout = self._estimate_route_timeout()

    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """
        # Create the background activity of the route
        veh_amounts = {
            'Town01': 90,
            'Town02': 75,
            'Town03': 90,
            'Town04': 150,
            'Town05': 90,
        }

        veh_amount = veh_amounts[self.town_name]

        new_actors = CarlaDataProvider.request_new_batch_actors('vehicle.*',
                                                                veh_amount,
                                                                carla.Transform(),
                                                                autopilot=True,
                                                                random_location=True,
                                                                rolename='background')

        if new_actors is None:
            raise Exception(
                "Error: Unable to add the background activity, all spawn points were occupied")

    def _initialize_environment(self, world):
        world.set_weather(WEATHERS[self.start_idx % 2])

    def _setup_scenario_trigger(self, config):
        pass

    def _setup_scenario_end(self, config):
        """
        This function adds and additional behavior to the scenario, which is triggered
        after it has ended.

        The function can be overloaded by a user implementation inside the user-defined scenario class.
        """
        pass

    def _create_test_criteria(self):
        """
        """
        criteria = []
        route = convert_transform_to_location(self.route)

        collision_criterion = CollisionTest(
            self.ego_vehicles[0], terminate_on_failure=True)

        route_criterion = InRouteTest(self.ego_vehicles[0],
                                      route=route,
                                      offroad_max=30,
                                      terminate_on_failure=True)

        completion_criterion = RouteCompletionTest(
            self.ego_vehicles[0], route=route)

        outsidelane_criterion = OutsideRouteLanesTest(
            self.ego_vehicles[0], route=route)

        red_light_criterion = RunningRedLightTest(self.ego_vehicles[0])

        stop_criterion = RunningStopTest(self.ego_vehicles[0])

        blocked_criterion = ActorSpeedAboveThresholdTest(self.ego_vehicles[0],
                                                         speed_threshold=0.1,
                                                         below_threshold_max_time=180.0,
                                                         terminate_on_failure=True,
                                                         name="AgentBlockedTest")

        criteria.append(completion_criterion)
        criteria.append(outsidelane_criterion)
        criteria.append(collision_criterion)
        criteria.append(red_light_criterion)
        criteria.append(stop_criterion)
        criteria.append(route_criterion)
        criteria.append(blocked_criterion)

        return criteria
