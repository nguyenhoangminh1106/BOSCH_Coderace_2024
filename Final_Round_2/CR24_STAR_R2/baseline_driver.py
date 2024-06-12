from carla import VehicleControl

from .controllers.pid_lat_controller import PidLatController
from .controllers.pid_lon_controller import PidLonController
from .controllers.aeb_controller import *
from .planners.cubic_spline_planner import Spline2D

import numpy as np


def get_driver():
    return "BaselineDriver"


class BaselineDriver():
    def __init__(self, router):
        self.router = router
        self.pid_lat = PidLatController()
        self.pid_lon = PidLonController()
        self.aeb_state = 0
        self.throt=0
        self.brake=0
        self.prev=0
        self.w_vel_brake=0
    def run(
        self, ego_pose, ego_dimension, ego_dynamics,
        npcs, timestamp, horizon=30
    ):
        ref_path = self.router.get_reference_path(
            ego_pose, ego_dynamics, timestamp, horizon)
        seed_trajectory = self.plan_trajectory(ref_path)
        target_waypoint = seed_trajectory[0]
        steering = self.pid_lat.pid_control(ego_pose, target_waypoint)
        ego_speed = ego_dynamics.get_speed()
        if len(ref_path)<3:
            lead_vehicle_dist = self.get_lead_vehicle(ego_pose, npcs,ref_path[-1],ref_path,ego_speed)
        else:
            lead_vehicle_dist = self.get_lead_vehicle(ego_pose, npcs,ref_path[2],ref_path,ego_speed)
        # print("Current vel:{} and Previous vel: {}".format(lead_vehicle_dist,self.getPrevVel()))
        self.setPrevVel(lead_vehicle_dist)
        if lead_vehicle_dist!=None:
            self.w_vel_brake=ego_speed/10
            self.brake=min(-lead_vehicle_dist/10+7/5+0.7*self.w_vel_brake,1)
            if lead_vehicle_dist<10:
                self.brake=1
            self.throt=0
            # if 14<lead_vehicle_dist<=15:
            #     self.brake = 0.85
            # elif  9.5<lead_vehicle_dist<=14:
            #     self.brake = 0.95
            #     # relative_velocity = ego_speed - lead_vehicle/timestamp
            #     # relative_distance = lead_vehicle

            #     # t_fcw = calculate_stopping_time(ego_speed, 3.8) + calculate_stopping_time(ego_speed, 9.8)
            #     # t_pb1 = calculate_stopping_time(ego_speed, 3.8)
            #     # t_pb2 = calculate_stopping_time(ego_speed, 5.8)
            #     # t_fb = calculate_stopping_time(ego_speed, 9.8)
            #     # ttc = calculate_ttc(relative_velocity, relative_distance)
                
            #     # self.aeb_state = state_transition_logic(ttc, t_fcw, t_pb1, t_pb2, t_fb)
            #     # deceleration = aeb_activation_logic(self.aeb_state, ego_speed)
                
            #     # throt, brake = self.calculate_throttle_brake(deceleration)
            # elif lead_vehicle_dist<=9.5:

            #     self.brake = 1
        else:
            self.throt = self.pid_lon.pid_control(
                target_speed=10,
                current_speed=ego_speed
            )
            self.brake = 0.0
        return VehicleControl(steer=steering, throttle=self.throt, brake=self.brake)
    def setPrevVel(self,vel):
        self.prev=vel
    def getPrevVel(self):
        return self.prev
    def plan_trajectory(self, ref_path, ds=1.):
        # NOTE: This is NOT a real planner
        traj = []
        sp = Spline2D(
            [wp.transform.location.x for wp in ref_path],
            [wp.transform.location.y for wp in ref_path]
        )
        s = np.arange(0, sp.s[-1], ds)
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            iyaw = sp.calc_yaw(i_s)
            traj.append([ix, iy, iyaw])

        return traj
    def get_lead_vehicle(self, ego_pose, npcs,ref_point,ref_path,ego_speed):
        """
        Get the nearest lead vehicle in front of the ego vehicle.
        """
        # Simplified example to get the closest vehicle ahead
        # obstacle=[]
        # for npc in npcs:
        #     obstacle.append(self.calculate_distance(ego_pose, npc))

        for npc in npcs:
            distance = self.calculate_distance(ego_pose, npc)
            # obstacle.append(distance)
            if (self.is_ahead(ego_pose, npc,ref_point)[0] and distance<8.5+ 0.95*ego_speed):
                # if len(ref_path)>=2:
                #     if self.lead_vehicle_onway(npc,ref_path):
                print("Hello world : {}".format(self.is_ahead(ego_pose, npc,ref_point)[1]))
                return distance
            elif ((not self.is_ahead(ego_pose, npc,ref_point)[0] )and distance<6 and 0.3<self.is_ahead(ego_pose, npc,ref_point)[1]<0.9):
                if len(ref_path)>=4:
                    if self.lead_vehicle_onway(npc,ref_path):
                        return distance
        # print("Min: {}".format(min(obstacle)))
        return None

    def calculate_distance(self, ego_pose, npc):
        """
        Calculate the distance between the ego vehicle and an NPC vehicle.
        """
        dx = npc.state.location.x - ego_pose.location.x
        dy = npc.state.location.y - ego_pose.location.y
        return np.sqrt(dx**2 + dy**2)
    def lead_vehicle_onway(self,npc,ref_path):
        mark_point1=ref_path[1]
        mark_point2=ref_path[3]
        vector_ref=[]
        vector_ref.append(mark_point1.transform.location.x-npc.state.location.x)
        vector_ref.append(mark_point1.transform.location.y-npc.state.location.y)
        vector_ref1=[]
        vector_ref1.append(mark_point2.transform.location.x-npc.state.location.x)
        vector_ref1.append(mark_point2.transform.location.y-npc.state.location.y)
        dot_product = vector_ref[0] * vector_ref1[0] + vector_ref[1] * vector_ref1[1]
        # Calculate magnitudes (norms) of the vectors
        magnitude_obs = math.sqrt(vector_ref1[0]**2 + vector_ref1[1]**2)
        magnitude_ref = math.sqrt(vector_ref[0]**2 + vector_ref[1]**2)

        # Calculate cosine of the angle
        cosine_angle = dot_product / (magnitude_obs * magnitude_ref)

        angle_radians = math.acos(cosine_angle)
        if 0<=angle_radians<0.1:
            return True
        return False
    def check_straight_waypoints(self,ref_pointsn):
        ref_point=ref_pointsn[0]
        mark_point1=ref_pointsn[1]
        mark_point2=ref_pointsn[5]
        vector_ref=[]
        vector_ref.append(ref_point.transform.location.x-mark_point1.transform.location.x)
        vector_ref.append(ref_point.transform.location.y-mark_point1.transform.location.y)
        vector_ref1=[]
        vector_ref1.append(ref_point.transform.location.x-mark_point2.transform.location.x)
        vector_ref1.append(ref_point.transform.location.y-mark_point2.transform.location.y)
        dot_product = vector_ref[0] * vector_ref1[0] + vector_ref[1] * vector_ref1[1]
        # Calculate magnitudes (norms) of the vectors
        magnitude_obs = math.sqrt(vector_ref1[0]**2 + vector_ref1[1]**2)
        magnitude_ref = math.sqrt(vector_ref[0]**2 + vector_ref[1]**2)

        # Calculate cosine of the angle
        cosine_angle = dot_product / (magnitude_obs * magnitude_ref)
        # cosine_angle = abs(max(min(cosine_angle, 1.0), -1.0))
        angle_radians = math.acos(cosine_angle)
        if 0<=angle_radians<0.1:
            return True
        return False
    def is_ahead(self, ego_pose, npc,ref_point):
        """
        Determine if an NPC vehicle is ahead of the ego vehicle.
        """
        vector_obs=[]
        vector_obs.append(npc.state.location.x-ego_pose.location.x)
        vector_obs.append(npc.state.location.y-ego_pose.location.y)
        vector_ref=[]
        vector_ref.append(ref_point.transform.location.x-ego_pose.location.x)
        vector_ref.append(ref_point.transform.location.y-ego_pose.location.y)
        dot_product = vector_obs[0] * vector_ref[0] + vector_obs[1] * vector_ref[1]
        # Calculate magnitudes (norms) of the vectors
        magnitude_obs = math.sqrt(vector_obs[0]**2 + vector_obs[1]**2)
        magnitude_ref = math.sqrt(vector_ref[0]**2 + vector_ref[1]**2)

        # Calculate cosine of the angle
        cosine_angle = dot_product / (magnitude_obs * magnitude_ref)
        # cosine_angle = abs(max(min(cosine_angle, 1.0), -1.0))
        angle_radians = math.acos(cosine_angle)
        if 0<=angle_radians<0.3:
            return [True,angle_radians]
        return [False,angle_radians]

    def calculate_throttle_brake(self, deceleration):
        """
        Calculate the throttle and brake values based on the required deceleration.
        """
        if deceleration == 0:
            return 5, 0.0  # Maintain speed
        elif deceleration > 0:
            return 0.0, deceleration / 9.8  # Apply brake proportional to max deceleration
        # else:
        #     return -deceleration / 9.8, 0.0  # Apply throttle proportional to negative deceleration
