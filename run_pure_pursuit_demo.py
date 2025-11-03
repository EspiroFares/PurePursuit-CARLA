import sys
import os
import time
import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from pure_pursuit_agent import PurePursuitAgent


client = carla.Client("localhost", 2000)
client.set_timeout(5.0)
world = client.get_world()
carla_map = world.get_map()

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter("model3")[0]
spawn_point = carla_map.get_spawn_points()[0]
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

if not vehicle:
    print("Could not spawn vehicle.")
    sys.exit(1)

print("Vehicle spawned")



sampling_resolution = 2.0
grp = GlobalRoutePlanner(carla_map, sampling_resolution)

start = spawn_point
end = carla_map.get_spawn_points()[11]
route = grp.trace_route(start.location, end.location)

# Draw route in blue
T = 120  # seconds visible
for pi, pj in zip(route[:-1], route[1:]):
    pi_loc = pi[0].transform.location
    pj_loc = pj[0].transform.location
    pi_loc.z = 0.5
    pj_loc.z = 0.5
    world.debug.draw_line(
        pi_loc, pj_loc,
        thickness=0.2,
        color=carla.Color(b=255),
        life_time=T
    )

waypoints = [wp[0] for wp in route]


agent = PurePursuitAgent(vehicle, waypoints, lookahead_distance=6.0)
print("Agent following route")


try:
    while True:
        control = agent.run_step()
        vehicle.apply_control(control)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Simulation stopped manually.")

finally:
    if vehicle is not None:
        vehicle.destroy()
        print("Vehicle destroyed. Simulation ended.")
