import carla
import math


class PurePursuitAgent:

    def __init__(self, vehicle, waypoints, lookahead_distance=6.0, target_speed=50.0, wheelbase=2.8):
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.target_speed = target_speed
        self.wheelbase = wheelbase
        self.current_index = 0

    def run_step(self):
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)

        vel = self.vehicle.get_velocity()
        v = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.lookahead_distance = 2.0 + 0.3 * v  # 2m + 0.3 * velocity

        #lookahead point
        lookahead_point = self._find_lookahead_point(vehicle_location)
        if lookahead_point is None:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            return control

        # Transform to vehicle coordinates
        dx = lookahead_point.x - vehicle_location.x
        dy = lookahead_point.y - vehicle_location.y
        local_x = math.cos(-vehicle_yaw) * dx - math.sin(-vehicle_yaw) * dy
        local_y = math.sin(-vehicle_yaw) * dx + math.cos(-vehicle_yaw) * dy

        alpha = math.atan2(local_y, local_x)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)

       


        max_steer = 1.22  # 70 degrees ich
        speed_factor = max(0.3,1.0 -0.01 * v)
        steer_cmd = max(-1.0, min(1.0, (delta / max_steer) * speed_factor))

        control = carla.VehicleControl()
        control.steer = steer_cmd

        print(v*3.6)

        #throttle control for target speed
        target_speed = self.target_speed/3.6 #km/h

        speed_error = target_speed - v
        throttle = 0.2 * speed_error

        throttle = max(0.0, min(1.0, throttle))

        control.throttle = throttle
        if throttle >  0.1:
            control.brake = 0.0 
        else:
            control.brake = 0.1
        return control

    def _find_lookahead_point(self, vehicle_location):
        for i in range(self.current_index, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = wp.transform.location.distance(vehicle_location)
            if dist > self.lookahead_distance:
                self.current_index = i
                return wp.transform.location
        return None
