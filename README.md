# Hybrid A* Isaac Sim Path Planning

An Ackermann-vehicle path planning and visualization script for NVIDIA Isaac Sim 5.1. It uses a forward-only Hybrid A* search over a baked NavMesh, draws the resulting route in the USD stage, publishes it as a durable ROS 2 `nav_msgs/Path`, and can kinematically move the vehicle along the route.

## Features

- Forward-only Hybrid A* search with an Ackermann bicycle model
- Steering-angle and steering-rate constraints
- NavMesh collision checks with configurable vehicle clearance
- Border-clearance cost for safer routes away from road edges
- Optional waypoint routing before the final destination
- USD `BasisCurves` route visualization
- ROS 2 `nav_msgs/Path` publishing with transient-local durability
- Pure-pursuit-style kinematic route following
- Automatic retries while the NavMesh is being prepared

## Requirements

- NVIDIA Isaac Sim 5.1
- A USD stage with a baked NavMesh
- Isaac Sim navigation extension (`omni.anim.navigation.core`)
- Isaac Sim core Python APIs
- NumPy
- Optional: ROS 2 and the Isaac Sim ROS 2 bridge

The script is intended to run inside Isaac Sim's Script Editor, not as a standalone Python program.

## Scene setup

The default configuration expects these USD prims:

| Purpose | Prim path or name |
| --- | --- |
| Vehicle | `/root/white_vehicle_v2` |
| Destination | `/root/_731/destination` |
| Optional waypoint | Unique prim named `move_ball` |
| Route visualization | `/root/Debug_Navigation_0816/PlannedAckermann` |

Before running the script:

1. Open the map stage in Isaac Sim.
2. Bake a NavMesh for the drivable area.
3. Confirm that the vehicle, destination, and optional waypoint prims exist.
4. Update the configuration constants near the top of `0816.py` to match your scene and vehicle.

## Usage

1. Open **Window > Script Editor** in Isaac Sim.
2. Open or paste `0816.py` into the editor.
3. Run the script after the stage and NavMesh are ready.
4. Watch the Script Editor output for `[INFO]`, `[WARN]`, and `[ERROR]` diagnostics.

On success, the script:

1. Projects the vehicle and target positions onto the NavMesh.
2. Plans each route leg with Hybrid A*.
3. Validates curvature, NavMesh occupancy, and border clearance.
4. Draws an orange route curve in the stage.
5. Publishes the complete route to ROS 2 when enabled.
6. Moves the vehicle along the route when auto-follow is enabled.

Running the script again safely closes the previous application instance before starting a new one.

## Configuration

The main settings are grouped near the top of `0816.py`:

- **Scene:** vehicle, goal, waypoint, and debug-curve prim paths
- **Vehicle:** wheelbase, track width, maximum steering angle, and safety margin
- **Planner:** step size, grid resolution, yaw bins, steering levels, search bounds, and expansion limit
- **Clearance:** minimum and desired NavMesh border clearance
- **Follower:** speed, lookahead distance, arrival tolerance, and timeline behavior
- **ROS 2:** node name, topic, and frame ID

The included vehicle defaults are:

| Parameter | Value |
| --- | ---: |
| Wheelbase | 1.28139 m |
| Track width | 0.94942 m |
| Maximum steering | 30 deg |
| NavMesh agent radius | 0.50 m |
| Safety margin | 0.05 m |

## ROS 2 output

When `ENABLE_ROS2 = True`, the route is published as:

- Topic: `/sim/planned_path/full`
- Message type: `nav_msgs/msg/Path`
- Frame: `map`
- QoS: reliable, keep last 1, transient local

Positions are converted from USD stage units to meters before publishing.

## Limitations

- Planning is forward-only; reverse maneuvers are not generated.
- The kinematic follower directly updates the vehicle transform and is not a physics-based controller.
- Scene prim paths and vehicle geometry are project-specific and must be adjusted for other stages.
- Planner behavior depends on a correctly baked and connected NavMesh.

## File

- `0816.py` — planner, validation, visualization, ROS 2 publisher, and route follower

