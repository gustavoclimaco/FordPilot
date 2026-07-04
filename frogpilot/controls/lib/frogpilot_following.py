#!/usr/bin/env python3
import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance, get_jerk_factor, get_T_FOLLOW

from openpilot.frogpilot.common.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED, MAX_T_FOLLOW

TRAFFIC_JERK_BP = [0.0, CITY_SPEED_LIMIT]
TRAFFIC_ACCELERATION_JERK = [0.50, 0.50]
TRAFFIC_DANGER_JERK =       [0.50, 0.50]
TRAFFIC_SPEED_JERK =        [0.50, 0.50]

TRAFFIC_FOLLOW_BP = [0.0,  CRUISING_SPEED]
TRAFFIC_FOLLOW =    [0.50, 1.00]

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.desired_follow_distance = 0
    self.speed_jerk = 0
    self.t_follow = 0

  def update(self, v_ego, sm, frogpilot_toggles):
    if sm["controlsState"].enabled and sm["frogpilotCarState"].trafficModeEnabled:
      self.acceleration_jerk = float(np.interp(v_ego, TRAFFIC_JERK_BP, TRAFFIC_ACCELERATION_JERK))
      self.danger_jerk = float(np.interp(v_ego, TRAFFIC_JERK_BP, TRAFFIC_DANGER_JERK))
      self.speed_jerk = float(np.interp(v_ego, TRAFFIC_JERK_BP, TRAFFIC_SPEED_JERK))
      self.t_follow = float(np.interp(v_ego, TRAFFIC_FOLLOW_BP, TRAFFIC_FOLLOW))
    elif sm["controlsState"].enabled:
      if sm["carState"].aEgo >= 0:
        self.acceleration_jerk, self.danger_jerk, self.speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, sm["controlsState"].personality
        )
      else:
        self.acceleration_jerk, self.danger_jerk, self.speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, sm["controlsState"].personality
        )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, sm["controlsState"].personality
      )
    else:
      self.acceleration_jerk = 0
      self.danger_jerk = 0
      self.speed_jerk = 0
      self.t_follow = 0

    self.following_lead = self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.dRel < (self.t_follow * 2) * v_ego

    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      self.t_follow = min(self.t_follow + self.frogpilot_planner.frogpilot_weather.increase_following_distance, MAX_T_FOLLOW)

    if sm["controlsState"].enabled and self.frogpilot_planner.tracking_lead:
      self.desired_follow_distance = int(desired_follow_distance(v_ego, self.frogpilot_planner.lead_one.vLead, self.t_follow))
    else:
      self.desired_follow_distance = 0
