# Trajectory Planner Package

The trajectory planner subscribes to detected lane center points and uses pure-pursuit control to generate command velocities.

The published linear speed is controlled by the `speed` parameter (default: `1.5`) and can be set at launch with the `speed` launch argument or changed at runtime.
