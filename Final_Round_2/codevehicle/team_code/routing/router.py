import carla
import global_config
import numpy as np
import itertools
import math

from collections import deque

from common.geometry.segment2d import Segment2D

BASE_MIN_DISTANCE = 3.0


class Router():
    def __init__(self, config):
        self.config = config
        self.map = None

        self._last_timestamp = float("inf")
        self._last_velocity = 0.

        self._remaining_route = None
        self._remaining_count = 0

        self._global_wps = None
        self._global_idx = 0

        self._local_seg = Segment2D()
        self._lon_accum = []
        self._lon_idx = -1

    def set_map(self, map):
        if not self.map:
            self.map = map

    def is_initialized(self):
        return self.map is not None

    def norm_to_range(self, value, start=-np.pi, end=np.pi):
        width = end - start
        offset = value - start
        return (offset - (np.floor(offset / width) * width)) + start

    def smooth_route(self, route):
        # remove a waypoint if the angle formed by its
        # two adjacent waypoints is larger than pi/4
        w = end = len(route) - 2
        while 0 < w:
            wp0 = route[w - 1]
            wp1 = route[w]
            wp2 = route[w + 1]

            yaw01 = math.atan2(
                wp1.location.y - wp0.location.y,
                wp1.location.x - wp0.location.x)

            yaw12 = math.atan2(
                wp2.location.y - wp1.location.y,
                wp2.location.x - wp1.location.x)

            if (math.pi / 4) < abs(self.norm_to_range(yaw12 - yaw01)):
                del route[w]
                end -= 1
                if w < end:
                    continue

            w -= 1

    def skim_route(self, route, skip):
        return range(len(route) - 1, 0, -skip).__reversed__()

    def squared_distance(self, x0, y0, x1, y1):
        return (x0 - x1)**2 + (y0 - y1)**2

    def set_route(self, global_route):
        if self._remaining_route:
            return

        ids_to_sample = self.skim_route(
            global_route, skip=int(global_config.WAYPOINT_SPACING_DISTANCE))
        route = [global_route[x][0] for x in ids_to_sample]
        assert 1 < len(route)

        self.smooth_route(route)
        assert 1 < len(route)

        carla_wps = [
            self.map.get_waypoint(carla.Location(
                x=trans.location.x,
                y=trans.location.y,
                z=trans.location.z
            ))
            for trans in route
        ]
        self.remove_duplicates(carla_wps)
        assert 1 < len(carla_wps)

        self._remaining_route = deque(maxlen=len(carla_wps))
        for carla_wp in carla_wps:
            self._remaining_route.append(carla_wp)

        self._global_wps = np.asarray(carla_wps)
        self._global_idx = 0

        self._lon_accum = [0.]
        self._lon_accum.extend(np.cumsum([
            math.sqrt(self.squared_distance(
                carla_wps[i + 0].transform.location.x,
                carla_wps[i + 0].transform.location.y,
                carla_wps[i + 1].transform.location.x,
                carla_wps[i + 1].transform.location.y
            )) for i in range(len(self._global_wps) - 1)
        ]))
        self._lon_accum = np.asarray(self._lon_accum)

    def remove_duplicates(self, waypoints):
        num_segs = len(waypoints) - 1
        assert 0 < num_segs
        if 1 == num_segs:
            return

        for i in range(num_segs, 0, -1):
            if self.squared_distance(
                waypoints[i].transform.location.x,
                waypoints[i].transform.location.y,
                waypoints[i - 1].transform.location.x,
                waypoints[i - 1].transform.location.y
            ) < (global_config.WAYPOINT_SPACING_DISTANCE / 10)**2:
                del waypoints[i]

    def get_reference_path(self, ego_pose, ego_dynamics, timestamp, horizon):
        self._pop_passed_waypoints(ego_pose, ego_dynamics, timestamp)
        self._remaining_count = min(len(self._remaining_route), horizon + 1)
        return list(itertools.islice(
            self._remaining_route, 0, self._remaining_count
        ))

    def _pop_passed_waypoints(
        self, ego_pose, ego_dynamics, timestamp
    ):
        delta_t = (timestamp - self._last_timestamp) \
            if timestamp > self._last_timestamp else 0.
        velocity = max(self._last_velocity, ego_dynamics.speedometer)
        search_end = self._global_idx + 1 + max(0, math.ceil(
            velocity * delta_t / global_config.WAYPOINT_SPACING_DISTANCE))

        lon_offset = float("inf")
        min_squared_dist = float("inf")

        saved_lon_idx = self._lon_idx
        for i in range(
            max(0, self._lon_idx), min(search_end, len(self._global_wps) - 1)
        ):
            wp0 = self._global_wps[i]
            wp1 = self._global_wps[i + 1]

            self._local_seg.p1.x = wp0.transform.location.x
            self._local_seg.p1.y = wp0.transform.location.y
            self._local_seg.p2.x = wp1.transform.location.x
            self._local_seg.p2.y = wp1.transform.location.y

            ratio, num, det = \
                self._local_seg.ratio_from_coord(
                    ego_pose.location.x, ego_pose.location.y)

            if 0. <= ratio and ratio <= 1.:
                squared_dist = (num ** 2) / det
                if min_squared_dist > squared_dist:
                    min_squared_dist = squared_dist

                    if self._lon_idx < i:
                        self._lon_idx = i

                    lon_offset = ratio * \
                        (self._lon_accum[i + 1] - self._lon_accum[i])

        if not (lon_offset < float("inf")):
            if self._lon_idx < 0:
                # haven't started on track
                i = max(self._lon_idx, 0)

                wp0 = self._global_wps[i]
                wp1 = self._global_wps[i + 1]

                self._local_seg.p1.x = wp0.transform.location.x
                self._local_seg.p1.y = wp0.transform.location.y
                self._local_seg.p2.x = wp1.transform.location.x
                self._local_seg.p2.y = wp1.transform.location.y

            elif (self._lon_idx + 2) == len(self._global_wps):
                # going beyond track
                wp0 = self._global_wps[-2]
                wp1 = self._global_wps[-1]

                self._local_seg.p1.x = wp0.transform.location.x
                self._local_seg.p1.y = wp0.transform.location.y
                self._local_seg.p2.x = wp1.transform.location.x
                self._local_seg.p2.y = wp1.transform.location.y

            else:
                # still on track, relax condition (0. <= ratio and ratio <= 1.)
                min_squared_dist_pt = float("inf")
                for i in range(
                    self._lon_idx, min(search_end, len(self._global_wps) - 1)
                ):
                    wp0 = self._global_wps[i]
                    wp1 = self._global_wps[i + 1]

                    squared_dist_pt = self.squared_distance(
                        wp0.transform.location.x, wp0.transform.location.y,
                        ego_pose.location.x, ego_pose.location.y)

                    if min_squared_dist_pt > squared_dist_pt:
                        min_squared_dist_pt = squared_dist_pt

                        self._local_seg.p1.x = wp0.transform.location.x
                        self._local_seg.p1.y = wp0.transform.location.y
                        self._local_seg.p2.x = wp1.transform.location.x
                        self._local_seg.p2.y = wp1.transform.location.y

                        if self._lon_idx < i:
                            self._lon_idx = i

        if (saved_lon_idx < self._lon_idx) and (2 < len(self._remaining_route)):
            num_waypoint_removed = 0
            for wpi in range(self._lon_idx, len(self._global_wps)):
                dist = self._lon_accum[wpi] - self._lon_accum[self._lon_idx]
                if dist < BASE_MIN_DISTANCE:
                    num_waypoint_removed += 1
                else:
                    break

            self._global_idx = min(
                self._lon_idx + num_waypoint_removed, len(self._global_wps) - 1)
            n = num_waypoint_removed if saved_lon_idx < 0 else (
                self._lon_idx - saved_lon_idx)
            for _ in range(min(n, len(self._remaining_route) - 1)):
                self._remaining_route.popleft()

        self._last_velocity = ego_dynamics.speedometer
        self._last_timestamp = timestamp
