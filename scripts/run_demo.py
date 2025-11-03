#!/usr/bin/env python3
"""
run_demo.py

Run a Pure Pursuit path tracking demo in the CARLA simulator.

Expected repo structure:
    carla_pure_pursuit/
    ├── agents/
    ├── pure_pursuit/
    ├── scripts/run_demo.py

Usage:
    python scripts/run_demo.py --speed 80
"""

import os
import sys
import time
import argparse
import logging
import carla


# -----------------------------------------------------------------------------
# Ensure both the local CARLA "agents" package and our "pure_pursuit" package
# can be imported when running from the /scripts/ directory.
# -----------------------------------------------------------------------------
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from agents.navigation.global_route_planner import GlobalRoutePlanner
from pure_pursuit.agent import PurePursuitAgent


# -----------------------------------------------------------------------------
# Main simulation function
# -----------------------------------------------------------------------------
def main(target_speed: float = 90.0) -> None:
    """Launch the CARLA simulation and run the Pure Pursuit agent."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
    )

    logging.info("Connecting to CARLA server...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    carla_map = world.get_map()

    # -------------------------------------------------------------------------
    # Vehicle setup
    # -------------------------------------------------------------------------
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.filter("model3")[0]
    spawn_point = carla_map.get_spawn_points()[0]
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    if not vehicle:
        logging.error("Could not spawn vehicle. Exiting.")
        sys.exit(1)

    logging.info("Vehicle spawned successfully.")

    # -------------------------------------------------------------------------
    # Route planning
    # -------------------------------------------------------------------------
    grp = GlobalRoutePlanner(carla_map, sampling_resolution=2.0)
    start = spawn_point
    end = carla_map.get_spawn_points()[11]
    route = grp.trace_route(start.location, end.location)

    # Draw the route (blue line)
    for wp1, wp2 in zip(route[:-1], route[1:]):
        loc1, loc2 = wp1[0].transform.location, wp2[0].transform.location
        loc1.z = loc2.z = 0.5
        world.debug.draw_line(
            loc1,
            loc2,
            thickness=0.2,
            color=carla.Color(b=255),
            life_time=120.0,
        )

    waypoints = [wp[0] for wp in route]
    agent = PurePursuitAgent(vehicle, waypoints, target_speed=target_speed)
    logging.info(f"Pure Pursuit Agent started (target speed = {target_speed:.1f} km/h)")

    # -------------------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------------------
    try:
        while True:
            control = agent.run_step()
            vehicle.apply_control(control)
            time.sleep(0.05)

    except KeyboardInterrupt:
        logging.info("Simulation interrupted by user.")

    finally:
        if vehicle.is_alive:
            vehicle.destroy()
            logging.info("Vehicle destroyed. Simulation ended.")


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the Pure Pursuit demo in CARLA.")
    parser.add_argument(
        "--speed",
        type=float,
        default=80.0,
        help="Target speed in km/h (default: 50).",
    )
    args = parser.parse_args()

    main(target_speed=args.speed)
