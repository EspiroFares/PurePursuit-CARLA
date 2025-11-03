"""
agent.py

Implements a Pure Pursuit controller for the CARLA simulator.
This controller computes throttle, brake, and steering commands
to make a vehicle follow a list of waypoints.
"""

import math
import logging
from typing import List, Optional
import carla


class PurePursuitAgent:
    """
    A Pure Pursuit path tracking controller for CARLA.

    Attributes:
        vehicle (carla.Vehicle): The CARLA vehicle actor being controlled.
        waypoints (List[carla.Waypoint]): List of route waypoints to follow.
        lookahead_distance (float): Distance ahead to target the next waypoint [m].
        target_speed (float): Desired vehicle speed [km/h].
        wheelbase (float): Vehicle wheelbase [m].
        kp_throttle (float): Proportional gain for throttle control.
        max_steer_angle (float): Maximum steering angle [radians].
    """

    def __init__(
        self,
        vehicle: carla.Vehicle,
        waypoints: List[carla.Waypoint],
        lookahead_distance: float = 6.0,
        target_speed: float = 50.0,
        wheelbase: float = 2.8,
        kp_throttle: float = 0.2,
        max_steer_angle: float = 1.22,
    ):
        """
        Initialize the Pure Pursuit agent.
        """
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.target_speed = target_speed
        self.wheelbase = wheelbase
        self.kp_throttle = kp_throttle
        self.max_steer_angle = max_steer_angle
        self.current_index = 0

        logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(message)s")

    # -------------------------------------------------------------------------
    # Main control method
    # -------------------------------------------------------------------------
    def run_step(self) -> carla.VehicleControl:
        """
        Compute and return the next control command for the vehicle.

        Returns:
            carla.VehicleControl: Steering, throttle, and brake commands.
        """
        return self._compute_control()

    # -------------------------------------------------------------------------
    # Core control logic
    # -------------------------------------------------------------------------
    def _compute_control(self) -> carla.VehicleControl:
        """
        Compute steering, throttle, and brake based on Pure Pursuit control law.
        """
        transform = self.vehicle.get_transform()
        location = transform.location
        yaw = math.radians(transform.rotation.yaw)

        velocity = self.vehicle.get_velocity()
        v = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # [m/s]

        # Dynamic lookahead distance
        self.lookahead_distance = 2.0 + 0.3 * v

        lookahead_point = self._find_lookahead_point(location)
        if lookahead_point is None:
            logging.warning("No valid lookahead point found. Braking.")
            return self._emergency_stop()

        # Transform the lookahead point into the vehicle's coordinate frame
        dx = lookahead_point.x - location.x
        dy = lookahead_point.y - location.y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        alpha = math.atan2(local_y, local_x)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)

        # Steering normalization
        speed_factor = max(0.3, 1.0 - 0.01 * v)
        steer_cmd = max(-1.0, min(1.0, (delta / self.max_steer_angle) * speed_factor))

        # Throttle control
        target_speed_ms = self.target_speed / 3.6
        speed_error = target_speed_ms - v
        throttle = max(0.0, min(1.0, self.kp_throttle * speed_error))
        brake = 0.0 if throttle > 0.1 else 0.1

        control = carla.VehicleControl(throttle=throttle, steer=steer_cmd, brake=brake)

        logging.info(
            f"Speed: {v * 3.6:.1f} km/h | Steer: {steer_cmd:.2f} | Throttle: {throttle:.2f}"
        )
        return control

    # -------------------------------------------------------------------------
    # Helper functions
    # -------------------------------------------------------------------------
    def _find_lookahead_point(self, vehicle_location: carla.Location) -> Optional[carla.Location]:
        """
        Find the next waypoint at least `lookahead_distance` ahead of the vehicle.
        """
        for i in range(self.current_index, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = wp.transform.location.distance(vehicle_location)
            if dist > self.lookahead_distance:
                self.current_index = i
                return wp.transform.location
        return None

    def _emergency_stop(self) -> carla.VehicleControl:
        """
        Stop the vehicle safely when no valid waypoint is found.
        """
        return carla.VehicleControl(throttle=0.0, brake=1.0)
