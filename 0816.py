"""
ASCII-only Ackermann NavMesh route planner for Isaac Sim 5.1.

Run this file from the Isaac Sim Script Editor after opening the map stage and
baking its NavMesh. The script does not move the vehicle. It plans and draws a
kinematically feasible route, then publishes it as a durable ROS 2 Path.
"""

import builtins
import heapq
import itertools
import math
import time
import traceback

import carb
import carb.eventdispatcher
import numpy as np
import omni.anim.navigation.core as nav
import omni.kit.app
import omni.timeline
import omni.usd
from omni.isaac.core.prims import XFormPrim
from pxr import Gf, Usd, UsdGeom


# -----------------------------------------------------------------------------
# Scene configuration
# -----------------------------------------------------------------------------

CAR_PATH = "/root/white_vehicle_v2"
GOAL_PATH = "/root/_731/destination"
WAYPOINT_PATH = ""
WAYPOINT_NAME = "move_ball"
USE_WAYPOINT = True

CURVE_PATH = "/root/Debug_Navigation_0816/PlannedAckermann"
CURVE_COLOR = (1.0, 0.35, 0.0)
CURVE_WIDTH_M = 0.08
CURVE_LIFT_M = 0.08


# -----------------------------------------------------------------------------
# Vehicle and planner configuration
# Values below match white_vehicle_v2.usda.
# -----------------------------------------------------------------------------

WHEELBASE_M = 1.28139
TRACK_WIDTH_M = 0.94942
MAX_STEERING_DEG = 30.0
SAFETY_MARGIN_M = 0.05
NAV_AGENT_RADIUS_M = 0.50

STEP_M = 0.30
COLLISION_SAMPLES = 6
GRID_RESOLUTION_M = 0.20
YAW_BINS = 72
STEERING_LEVELS = 9
MAX_STEERING_INDEX_CHANGE = 1
GOAL_CONNECT_DISTANCE_M = 2.50
GOAL_STEERING_TRANSITION_DEG = 10.0
SEARCH_MARGIN_M = 5.0
MAX_EXPANSIONS = 80000
NAVMESH_SNAP_TOLERANCE_M = 0.08
ENDPOINT_MAX_PROJECTION_M = 2.0
START_NAVMESH_INSET_M = 0.75
MIN_BORDER_CLEARANCE_M = 0.55
DESIRED_BORDER_CLEARANCE_M = 0.85
BORDER_COST_WEIGHT = 0.30
BORDER_QUERY_CAP_M = 2.0
BORDER_HASH_CELL_M = 1.0

# A value of 1.0 keeps the A* heuristic admissible. Length remains the primary
# objective; steering terms only break ties in favor of smoother routes.
HEURISTIC_WEIGHT = 1.0
STEERING_COST_WEIGHT = 0.05
STEERING_CHANGE_COST_WEIGHT = 0.15

PLAN_RETRY_INTERVAL_S = 1.0
PLAN_MAX_RETRIES = 30
PRINT_TRACEBACK = False

ENABLE_AUTO_FOLLOW = True
PAUSE_TIMELINE_FOR_KINEMATIC_FOLLOW = True
FOLLOW_SPEED_M_S = 1.5
FOLLOW_MIN_SPEED_M_S = 0.45
FOLLOW_LOOKAHEAD_M = 1.50
FOLLOW_SLOW_DISTANCE_M = 3.0
FOLLOW_ARRIVAL_TOLERANCE_M = 0.35
FOLLOW_MAX_DT_S = 0.10
FOLLOW_LOG_INTERVAL_S = 1.0


# -----------------------------------------------------------------------------
# ROS 2 configuration
# -----------------------------------------------------------------------------

ENABLE_ROS2 = True
ROS_NODE_NAME = "isaac_ackermann_route_publisher"
ROS_TOPIC_PATH = "/sim/planned_path/full"
ROS_FRAME_ID = "map"


def log(level, code, message):
    """Print one compact ASCII-only diagnostic line."""
    print(f"[{level}][{code}] {message}")


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def quaternion_wxyz_to_yaw(q):
    w, x, y, z = [float(v) for v in q]
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_to_quaternion_xyzw(yaw):
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def get_world_pose(prim):
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    position = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    quaternion_wxyz = (
        float(rotation.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    )
    return (
        np.array(position, dtype=np.float64),
        quaternion_wxyz,
    )


def find_unique_prim_by_name(stage, name):
    matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if not matches:
        raise RuntimeError(f"Prim name not found: {name}")
    if len(matches) > 1:
        paths = ", ".join(str(prim.GetPath()) for prim in matches[:8])
        raise RuntimeError(
            f"Prim name is not unique: {name}; matches={paths}"
        )
    return matches[0]


class BorderClearanceIndex:
    """Small spatial index for distance queries against NavMesh border lines."""

    def __init__(self, segments, cell_size, cap_distance):
        self.segments = list(segments)
        self.cell_size = max(float(cell_size), 1e-6)
        self.cap_distance = max(float(cap_distance), self.cell_size)
        self.cells = {}
        for index, segment in enumerate(self.segments):
            ax, ay, bx, by = segment
            min_ix = math.floor(min(ax, bx) / self.cell_size)
            max_ix = math.floor(max(ax, bx) / self.cell_size)
            min_iy = math.floor(min(ay, by) / self.cell_size)
            max_iy = math.floor(max(ay, by) / self.cell_size)
            for ix in range(min_ix, max_ix + 1):
                for iy in range(min_iy, max_iy + 1):
                    self.cells.setdefault((ix, iy), []).append(index)

    @staticmethod
    def _point_segment_distance(x, y, ax, ay, bx, by):
        vx = bx - ax
        vy = by - ay
        denominator = vx * vx + vy * vy
        if denominator <= 1e-12:
            return math.hypot(x - ax, y - ay)
        ratio = ((x - ax) * vx + (y - ay) * vy) / denominator
        ratio = max(0.0, min(1.0, ratio))
        px = ax + ratio * vx
        py = ay + ratio * vy
        return math.hypot(x - px, y - py)

    def clearance(self, x, y):
        if not self.segments:
            return self.cap_distance
        center_x = math.floor(float(x) / self.cell_size)
        center_y = math.floor(float(y) / self.cell_size)
        radius = int(math.ceil(self.cap_distance / self.cell_size))
        candidates = set()
        for ix in range(center_x - radius, center_x + radius + 1):
            for iy in range(center_y - radius, center_y + radius + 1):
                candidates.update(self.cells.get((ix, iy), ()))
        best = self.cap_distance
        for index in candidates:
            best = min(
                best,
                self._point_segment_distance(
                    float(x),
                    float(y),
                    *self.segments[index],
                ),
            )
        return best


class AckermannNavMeshPlanner:
    """Forward-only Hybrid A* with an Ackermann bicycle motion model."""

    def __init__(self, navmesh, meters_per_unit):
        self.navmesh = navmesh
        self.meters_per_unit = float(meters_per_unit)
        self.wheelbase = self.to_stage(WHEELBASE_M)
        self.track_width = self.to_stage(TRACK_WIDTH_M)
        self.step = self.to_stage(STEP_M)
        self.grid_resolution = self.to_stage(GRID_RESOLUTION_M)
        self.search_margin = self.to_stage(SEARCH_MARGIN_M)
        self.goal_connect_distance = self.to_stage(GOAL_CONNECT_DISTANCE_M)
        self.snap_tolerance = self.to_stage(NAVMESH_SNAP_TOLERANCE_M)
        self.endpoint_max_projection = self.to_stage(ENDPOINT_MAX_PROJECTION_M)
        self.minimum_border_clearance = self.to_stage(MIN_BORDER_CLEARANCE_M)
        self.desired_border_clearance = self.to_stage(
            DESIRED_BORDER_CLEARANCE_M
        )
        self.max_steering = math.radians(MAX_STEERING_DEG)
        self.minimum_turn_radius = self.wheelbase / math.tan(self.max_steering)
        self.steering_values = np.linspace(
            -self.max_steering,
            self.max_steering,
            max(3, int(STEERING_LEVELS)),
        ).tolist()
        self.zero_steering_index = min(
            range(len(self.steering_values)),
            key=lambda idx: abs(self.steering_values[idx]),
        )
        self.agent = self._make_agent()
        self.border_index = self._make_border_index()
        self.navmesh_cache = {}

    def to_stage(self, meters):
        return float(meters) / self.meters_per_unit

    def to_meters(self, stage_units):
        return float(stage_units) * self.meters_per_unit

    def _make_agent(self):
        desc_class = getattr(nav, "NavAgentDesc", None)
        if desc_class is None:
            log("WARN", "NO_AGENT_DESC", "Using the baked NavMesh agent settings")
            return None

        try:
            desc = desc_class()
            # The current map was baked for a 0.50 m maximum agent radius.
            # The vehicle half width is 0.47471 m, leaving 0.02529 m per side.
            desc.radius = self.to_stage(NAV_AGENT_RADIUS_M)
            desc.height = self.to_stage(1.0)
            desc.collision_gap = self.to_stage(SAFETY_MARGIN_M)
            return desc
        except Exception as exc:
            log("WARN", "AGENT_DESC_FAILED", f"{type(exc).__name__}: {exc}")
            return None

    def _make_border_index(self):
        segments = []
        try:
            points = list(self.navmesh.get_draw_lines(border_only=True) or [])
            for index in range(0, len(points) - 1, 2):
                first = points[index]
                second = points[index + 1]
                segments.append(
                    (
                        float(first[0]),
                        float(first[1]),
                        float(second[0]),
                        float(second[1]),
                    )
                )
        except Exception as exc:
            log("WARN", "BORDER_READ_FAILED", f"{type(exc).__name__}: {exc}")
        log("INFO", "BORDER_READY", f"segments={len(segments)}")
        return BorderClearanceIndex(
            segments,
            self.to_stage(BORDER_HASH_CELL_M),
            self.to_stage(BORDER_QUERY_CAP_M),
        )

    def _query_closest(self, x, y, z):
        target = carb.Float3(float(x), float(y), float(z))
        try:
            if self.agent is not None:
                point, island = self.navmesh.query_closest_point(
                    target=target,
                    agent=self.agent,
                )
            else:
                point, island = self.navmesh.query_closest_point(target=target)
            if point is None:
                return None
            return np.array(
                [float(point[0]), float(point[1]), float(point[2])],
                dtype=np.float64,
            )
        except Exception:
            try:
                point, island = self.navmesh.query_closest_point(target)
                if point is None:
                    return None
                return np.array(
                    [float(point[0]), float(point[1]), float(point[2])],
                    dtype=np.float64,
                )
            except Exception:
                return None

    def project_to_navmesh(self, point):
        return self._query_closest(point[0], point[1], point[2])

    def _is_on_navmesh(self, x, y, z):
        key = (
            round(float(x) / max(self.grid_resolution * 0.25, 1e-9)),
            round(float(y) / max(self.grid_resolution * 0.25, 1e-9)),
        )
        cached = self.navmesh_cache.get(key)
        if cached is not None:
            return cached

        closest = self._query_closest(x, y, z)
        valid = False
        if closest is not None:
            distance = math.hypot(
                float(closest[0]) - float(x),
                float(closest[1]) - float(y),
            )
            valid = distance <= self.snap_tolerance
        self.navmesh_cache[key] = bool(valid)
        return bool(valid)

    def _query_coarse_path(self, start, goal):
        start_point = carb.Float3(*[float(v) for v in start[:3]])
        goal_point = carb.Float3(*[float(v) for v in goal[:3]])
        kwargs = {
            "start_pos": start_point,
            "end_pos": goal_point,
            "straighten": True,
        }
        if self.agent is not None:
            kwargs["agent"] = self.agent

        try:
            result = self.navmesh.query_shortest_path(**kwargs)
        except Exception:
            result = self.navmesh.query_shortest_path(start_point, goal_point)

        if result is None:
            return []
        return [
            np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float64)
            for p in result.get_points()
        ]

    def _state_key(self, x, y, yaw, steering_index):
        ix = int(round(float(x) / self.grid_resolution))
        iy = int(round(float(y) / self.grid_resolution))
        yaw_unit = 2.0 * math.pi / float(YAW_BINS)
        iyaw = int(round(normalize_angle(yaw) / yaw_unit)) % int(YAW_BINS)
        return ix, iy, iyaw, int(steering_index)

    def _integrate(self, x, y, yaw, steering, distance):
        curvature = math.tan(float(steering)) / self.wheelbase
        if abs(curvature) < 1e-12:
            next_x = float(x) + float(distance) * math.cos(float(yaw))
            next_y = float(y) + float(distance) * math.sin(float(yaw))
            next_yaw = float(yaw)
        else:
            next_yaw = float(yaw) + float(distance) * curvature
            next_x = float(x) + (
                math.sin(next_yaw) - math.sin(float(yaw))
            ) / curvature
            next_y = float(y) - (
                math.cos(next_yaw) - math.cos(float(yaw))
            ) / curvature
        return next_x, next_y, normalize_angle(next_yaw)

    def _sample_motion(self, x, y, yaw, steering, distance, z):
        samples = []
        count = max(2, int(COLLISION_SAMPLES))
        for sample_index in range(1, count + 1):
            sample_distance = float(distance) * sample_index / count
            sx, sy, syaw = self._integrate(
                x,
                y,
                yaw,
                steering,
                sample_distance,
            )
            samples.append(
                np.array([sx, sy, float(z), syaw, float(steering)], dtype=np.float64)
            )
        return samples

    def _motion_is_valid(self, samples, bounds):
        min_x, max_x, min_y, max_y = bounds
        for sample in samples:
            x, y, z = sample[:3]
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return False
            if not self._is_on_navmesh(x, y, z):
                return False
            if (
                self.border_index.clearance(x, y)
                < self.minimum_border_clearance
            ):
                return False
        return True

    def _build_bounds(self, coarse_path, start, goal):
        points = list(coarse_path) + [start, goal]
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        return (
            min(xs) - self.search_margin,
            max(xs) + self.search_margin,
            min(ys) - self.search_margin,
            max(ys) + self.search_margin,
        )

    def _point_along_polyline(self, points, distance):
        """Return an interior point and tangent at a distance along a polyline."""
        if len(points) < 2:
            return None, None

        remaining = max(0.0, float(distance))
        for first, second in zip(points[:-1], points[1:]):
            dx = float(second[0]) - float(first[0])
            dy = float(second[1]) - float(first[1])
            segment_length = math.hypot(dx, dy)
            if segment_length < 1e-9:
                continue
            if remaining <= segment_length:
                ratio = remaining / segment_length
                point = np.array(
                    [
                        float(first[0]) + ratio * dx,
                        float(first[1]) + ratio * dy,
                        float(first[2])
                        + ratio * (float(second[2]) - float(first[2])),
                    ],
                    dtype=np.float64,
                )
                return point, math.atan2(dy, dx)
            remaining -= segment_length

        last = np.array(points[-1][:3], dtype=np.float64)
        previous = np.array(points[-2][:3], dtype=np.float64)
        return last, math.atan2(last[1] - previous[1], last[0] - previous[0])

    def _recover_interior_start(self, coarse_path):
        """Find a start point away from both the vehicle hole and road borders."""
        best_point = None
        best_yaw = None
        best_clearance = -1.0
        progress_values = np.arange(
            START_NAVMESH_INSET_M,
            START_NAVMESH_INSET_M + 4.01,
            0.25,
        )
        lateral_values = [
            0.0,
            0.25,
            -0.25,
            0.50,
            -0.50,
            0.75,
            -0.75,
            1.00,
            -1.00,
            1.25,
            -1.25,
            1.50,
            -1.50,
        ]

        for progress_m in progress_values:
            center, yaw = self._point_along_polyline(
                coarse_path,
                self.to_stage(progress_m),
            )
            if center is None:
                continue
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            for lateral_m in lateral_values:
                lateral = self.to_stage(lateral_m)
                candidate = np.array(
                    [
                        float(center[0]) + normal_x * lateral,
                        float(center[1]) + normal_y * lateral,
                        float(center[2]),
                    ],
                    dtype=np.float64,
                )
                if not self._is_on_navmesh(
                    candidate[0], candidate[1], candidate[2]
                ):
                    continue
                clearance = self.border_index.clearance(
                    candidate[0], candidate[1]
                )
                forward_samples = self._sample_motion(
                    candidate[0],
                    candidate[1],
                    yaw,
                    0.0,
                    self.step,
                    candidate[2],
                )
                forward_is_safe = all(
                    self._is_on_navmesh(sample[0], sample[1], sample[2])
                    and self.border_index.clearance(sample[0], sample[1])
                    >= self.minimum_border_clearance
                    for sample in forward_samples
                )
                if not forward_is_safe:
                    continue
                if clearance > best_clearance:
                    best_point = candidate
                    best_yaw = float(yaw)
                    best_clearance = float(clearance)
                if clearance >= self.desired_border_clearance:
                    return candidate, float(yaw), float(clearance)

        return best_point, best_yaw, best_clearance

    def _goal_arc(self, record, goal, bounds):
        dx = float(goal[0]) - float(record["x"])
        dy = float(goal[1]) - float(record["y"])
        chord = math.hypot(dx, dy)
        if chord < 1e-9:
            return []
        if chord > self.goal_connect_distance:
            return None

        bearing = math.atan2(dy, dx)
        alpha = normalize_angle(bearing - float(record["yaw"]))
        if math.cos(alpha) <= 0.0:
            return None

        curvature = 2.0 * math.sin(alpha) / chord
        max_curvature = math.tan(self.max_steering) / self.wheelbase
        if abs(curvature) > max_curvature + 1e-9:
            return None

        steering = math.atan(curvature * self.wheelbase)
        previous_steering = self.steering_values[int(record["steering_index"])]
        transition_limit = math.radians(GOAL_STEERING_TRANSITION_DEG)
        if abs(steering - previous_steering) > transition_limit:
            return None

        if abs(curvature) < 1e-12:
            arc_length = chord
        else:
            arc_length = 2.0 * alpha / curvature
        if arc_length <= 0.0:
            return None

        sample_count = max(
            int(COLLISION_SAMPLES),
            int(math.ceil(arc_length / max(self.step / COLLISION_SAMPLES, 1e-9))),
        )
        samples = []
        for sample_index in range(1, sample_count + 1):
            distance = arc_length * sample_index / sample_count
            sx, sy, syaw = self._integrate(
                record["x"],
                record["y"],
                record["yaw"],
                steering,
                distance,
            )
            if sample_index == sample_count:
                sx = float(goal[0])
                sy = float(goal[1])
            sample = np.array(
                [sx, sy, float(goal[2]), syaw, steering],
                dtype=np.float64,
            )
            samples.append(sample)

        if not self._motion_is_valid(samples, bounds):
            return None
        return samples

    def _reconstruct(self, records, final_key, z):
        keys = []
        key = final_key
        while key is not None:
            keys.append(key)
            key = records[key]["parent"]
        keys.reverse()

        first = records[keys[0]]
        route = [
            np.array(
                [
                    first["x"],
                    first["y"],
                    float(z),
                    first["yaw"],
                    self.steering_values[first["steering_index"]],
                ],
                dtype=np.float64,
            )
        ]

        for parent_key, child_key in zip(keys[:-1], keys[1:]):
            parent = records[parent_key]
            child = records[child_key]
            steering = self.steering_values[child["steering_index"]]
            route.extend(
                self._sample_motion(
                    parent["x"],
                    parent["y"],
                    parent["yaw"],
                    steering,
                    self.step,
                    z,
                )
            )
        return route

    def plan_leg(self, start_pose, goal_point, leg_name):
        start = np.array(start_pose[:3], dtype=np.float64)
        goal = np.array(goal_point[:3], dtype=np.float64)
        start_projection = self.project_to_navmesh(start)
        goal_projection = self.project_to_navmesh(goal)
        if start_projection is None:
            raise RuntimeError(f"{leg_name}: start is not near the NavMesh")
        if goal_projection is None:
            raise RuntimeError(f"{leg_name}: goal is not near the NavMesh")

        start_offset = math.hypot(*(start_projection[:2] - start[:2]))
        goal_offset = math.hypot(*(goal_projection[:2] - goal[:2]))
        if start_offset > self.endpoint_max_projection:
            raise RuntimeError(
                f"{leg_name}: start is too far from NavMesh; "
                f"offset_m={self.to_meters(start_offset):.3f}"
            )
        if goal_offset > self.endpoint_max_projection:
            raise RuntimeError(
                f"{leg_name}: goal is too far from NavMesh; "
                f"offset_m={self.to_meters(goal_offset):.3f}"
            )
        if start_offset > self.snap_tolerance:
            log(
                "WARN",
                "START_PROJECTED",
                f"leg={leg_name} offset_m={self.to_meters(start_offset):.3f}",
            )
        if goal_offset > self.snap_tolerance:
            log(
                "WARN",
                "GOAL_PROJECTED",
                f"leg={leg_name} offset_m={self.to_meters(goal_offset):.3f}",
            )

        planning_z = float(start_projection[2])
        start[:] = start_projection
        goal[:] = goal_projection
        coarse_path = self._query_coarse_path(start, goal)
        if len(coarse_path) < 2:
            raise RuntimeError(f"{leg_name}: NavMesh shortest path was not found")

        start_yaw = normalize_angle(start_pose[3])
        if start_offset > self.snap_tolerance:
            recovered_start, recovered_yaw, recovered_clearance = (
                self._recover_interior_start(coarse_path)
            )
            if recovered_start is None:
                raise RuntimeError(f"{leg_name}: start recovery failed")
            if recovered_clearance < self.minimum_border_clearance:
                raise RuntimeError(
                    f"{leg_name}: no safe recovery point; best_clearance_m="
                    f"{self.to_meters(recovered_clearance):.3f}"
                )
            start[:] = recovered_start
            planning_z = float(recovered_start[2])
            start_yaw = float(recovered_yaw)
            log(
                "WARN",
                "START_RECOVERED",
                "leg={} projection_m={:.3f} clearance_m={:.3f} "
                "yaw_deg={:.1f}".format(
                    leg_name,
                    self.to_meters(start_offset),
                    self.to_meters(recovered_clearance),
                    math.degrees(start_yaw),
                ),
            )

        bounds = self._build_bounds(coarse_path, start, goal)
        start_key = self._state_key(
            start[0],
            start[1],
            start_yaw,
            self.zero_steering_index,
        )
        records = {
            start_key: {
                "x": float(start[0]),
                "y": float(start[1]),
                "yaw": start_yaw,
                "steering_index": self.zero_steering_index,
                "g": 0.0,
                "parent": None,
            }
        }
        queue = []
        counter = itertools.count()
        initial_h = math.hypot(goal[0] - start[0], goal[1] - start[1])
        heapq.heappush(queue, (initial_h, next(counter), start_key))
        closed = set()
        best_goal_cost = float("inf")
        best_goal_key = None
        best_goal_samples = None

        log(
            "INFO",
            "LEG_START",
            f"name={leg_name} coarse_points={len(coarse_path)}",
        )

        expansions = 0
        while queue and expansions < int(MAX_EXPANSIONS):
            priority, sequence, key = heapq.heappop(queue)
            if key in closed:
                continue
            if best_goal_key is not None and priority >= best_goal_cost:
                route = self._reconstruct(records, best_goal_key, planning_z)
                route.extend(best_goal_samples)
                log(
                    "INFO",
                    "LEG_OK",
                    f"name={leg_name} expansions={expansions} points={len(route)}",
                )
                return route
            record = records[key]
            closed.add(key)
            expansions += 1

            goal_samples = self._goal_arc(record, goal, bounds)
            if goal_samples is not None:
                previous_xy = np.array([record["x"], record["y"]])
                arc_length = 0.0
                for sample in goal_samples:
                    current_xy = np.array([sample[0], sample[1]])
                    arc_length += float(np.linalg.norm(current_xy - previous_xy))
                    previous_xy = current_xy
                goal_steering = (
                    float(goal_samples[-1][4]) if goal_samples else 0.0
                )
                previous_steering = self.steering_values[
                    int(record["steering_index"])
                ]
                steering_ratio = abs(goal_steering) / max(
                    self.max_steering, 1e-9
                )
                steering_change_ratio = abs(
                    goal_steering - previous_steering
                ) / max(self.max_steering, 1e-9)
                goal_cost = float(record["g"]) + arc_length * (
                    1.0
                    + STEERING_COST_WEIGHT * steering_ratio
                    + STEERING_CHANGE_COST_WEIGHT * steering_change_ratio
                )
                if goal_cost < best_goal_cost:
                    best_goal_cost = goal_cost
                    best_goal_key = key
                    best_goal_samples = goal_samples
                continue

            previous_index = int(record["steering_index"])
            first_index = max(0, previous_index - MAX_STEERING_INDEX_CHANGE)
            last_index = min(
                len(self.steering_values) - 1,
                previous_index + MAX_STEERING_INDEX_CHANGE,
            )

            for steering_index in range(first_index, last_index + 1):
                steering = self.steering_values[steering_index]
                samples = self._sample_motion(
                    record["x"],
                    record["y"],
                    record["yaw"],
                    steering,
                    self.step,
                    planning_z,
                )
                if not self._motion_is_valid(samples, bounds):
                    continue

                next_state = samples[-1]
                next_key = self._state_key(
                    next_state[0],
                    next_state[1],
                    next_state[3],
                    steering_index,
                )
                if next_key in closed:
                    continue

                steering_ratio = abs(steering) / max(self.max_steering, 1e-9)
                steering_change_ratio = abs(
                    steering - self.steering_values[previous_index]
                ) / max(self.max_steering, 1e-9)
                clearance = self.border_index.clearance(
                    next_state[0], next_state[1]
                )
                clearance_ratio = max(
                    0.0,
                    (self.desired_border_clearance - clearance)
                    / max(self.desired_border_clearance, 1e-9),
                )
                step_cost = self.step * (
                    1.0
                    + STEERING_COST_WEIGHT * steering_ratio
                    + STEERING_CHANGE_COST_WEIGHT * steering_change_ratio
                    + BORDER_COST_WEIGHT * clearance_ratio * clearance_ratio
                )
                candidate_g = float(record["g"]) + step_cost
                previous = records.get(next_key)
                if previous is not None and candidate_g >= float(previous["g"]):
                    continue

                records[next_key] = {
                    "x": float(next_state[0]),
                    "y": float(next_state[1]),
                    "yaw": float(next_state[3]),
                    "steering_index": int(steering_index),
                    "g": candidate_g,
                    "parent": key,
                }
                heuristic = math.hypot(
                    float(goal[0]) - float(next_state[0]),
                    float(goal[1]) - float(next_state[1]),
                )
                total_cost = candidate_g + HEURISTIC_WEIGHT * heuristic
                heapq.heappush(
                    queue,
                    (total_cost, next(counter), next_key),
                )

        if best_goal_key is not None:
            route = self._reconstruct(records, best_goal_key, planning_z)
            route.extend(best_goal_samples)
            log(
                "WARN",
                "LEG_BEST_AVAILABLE",
                f"name={leg_name} expansions={expansions} points={len(route)}",
            )
            return route

        raise RuntimeError(
            f"{leg_name}: search failed; expansions={expansions} open={len(queue)}"
        )

    def validate_route(self, route):
        if len(route) < 2:
            raise RuntimeError("Route contains fewer than two points")

        max_curvature = math.tan(self.max_steering) / self.wheelbase
        max_seen = 0.0
        minimum_clearance = float("inf")
        for index, point in enumerate(route):
            if not np.all(np.isfinite(point)):
                raise RuntimeError(f"Route point is not finite: index={index}")
            steering = float(point[4])
            curvature = abs(math.tan(steering) / self.wheelbase)
            max_seen = max(max_seen, curvature)
            if curvature > max_curvature + 1e-9:
                raise RuntimeError(f"Curvature limit exceeded: index={index}")
            if not self._is_on_navmesh(point[0], point[1], point[2]):
                raise RuntimeError(f"Route left the NavMesh: index={index}")
            minimum_clearance = min(
                minimum_clearance,
                self.border_index.clearance(point[0], point[1]),
            )

        log(
            "INFO",
            "ROUTE_VALID",
            "points={} min_turn_radius_m={:.3f} max_curvature_1_m={:.4f} "
            "min_border_clearance_m={:.3f}".format(
                len(route),
                self.to_meters(self.minimum_turn_radius),
                max_seen / self.meters_per_unit,
                self.to_meters(minimum_clearance),
            ),
        )


class RouteDisplay:
    def __init__(self, stage, navmesh, meters_per_unit):
        self.stage = stage
        self.navmesh = navmesh
        self.meters_per_unit = float(meters_per_unit)
        self.curve = UsdGeom.BasisCurves.Define(stage, CURVE_PATH)
        self.curve.CreateTypeAttr("linear")
        self.curve.CreateWrapAttr("nonperiodic")
        self.curve.GetWidthsAttr().Set(
            [float(CURVE_WIDTH_M / self.meters_per_unit)]
        )
        self.curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        geometry = UsdGeom.Gprim(self.curve.GetPrim())
        geometry.CreateDisplayColorAttr().Set([Gf.Vec3f(*CURVE_COLOR)])

        xformable = UsdGeom.Xformable(self.curve.GetPrim())
        xformable.ClearXformOpOrder()
        xformable.SetResetXformStack(True)

    def _surface_z(self, point):
        target = carb.Float3(float(point[0]), float(point[1]), float(point[2]))
        try:
            closest, island = self.navmesh.query_closest_point(target=target)
            if closest is not None:
                return float(closest[2])
        except Exception:
            pass
        return float(point[2])

    def draw(self, route):
        lift = CURVE_LIFT_M / self.meters_per_unit
        points = [
            Gf.Vec3f(
                float(point[0]),
                float(point[1]),
                self._surface_z(point) + lift,
            )
            for point in route
        ]
        self.curve.GetPointsAttr().Set(points)
        self.curve.GetCurveVertexCountsAttr().Set([len(points)])
        log("INFO", "ROUTE_DRAWN", f"curve={CURVE_PATH} points={len(points)}")


class RosRoutePublisher:
    def __init__(self, meters_per_unit):
        self.meters_per_unit = float(meters_per_unit)
        self.ok = False
        self.node = None
        self.publisher = None
        self.PathMessage = None
        self.PoseStampedMessage = None

        if not ENABLE_ROS2:
            log("INFO", "ROS2_DISABLED", "ROS 2 route publishing is disabled")
            return

        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )

            if not rclpy.ok():
                rclpy.init(args=None)

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.node = rclpy.create_node(ROS_NODE_NAME)
            self.publisher = self.node.create_publisher(Path, ROS_TOPIC_PATH, qos)
            self.PathMessage = Path
            self.PoseStampedMessage = PoseStamped
            self.ok = True
            log(
                "INFO",
                "ROS2_READY",
                f"topic={ROS_TOPIC_PATH} durability=transient_local",
            )
        except Exception as exc:
            log("ERROR", "ROS2_INIT_FAILED", f"{type(exc).__name__}: {exc}")

    def close(self):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.ok = False

    def publish(self, route):
        if not self.ok:
            return

        try:
            stamp = self.node.get_clock().now().to_msg()
            message = self.PathMessage()
            message.header.stamp = stamp
            message.header.frame_id = ROS_FRAME_ID

            for point in route:
                pose = self.PoseStampedMessage()
                pose.header.stamp = stamp
                pose.header.frame_id = ROS_FRAME_ID
                pose.pose.position.x = float(point[0]) * self.meters_per_unit
                pose.pose.position.y = float(point[1]) * self.meters_per_unit
                pose.pose.position.z = float(point[2]) * self.meters_per_unit
                qx, qy, qz, qw = yaw_to_quaternion_xyzw(point[3])
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                message.poses.append(pose)

            self.publisher.publish(message)
            log(
                "INFO",
                "ROS2_PATH_PUBLISHED",
                f"topic={ROS_TOPIC_PATH} points={len(message.poses)}",
            )
        except Exception as exc:
            log("ERROR", "ROS2_PUBLISH_FAILED", f"{type(exc).__name__}: {exc}")


class RouteApplication:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("No USD stage is open")

        self.meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(self.stage))
        if self.meters_per_unit <= 0.0:
            raise RuntimeError("Invalid stage metersPerUnit")

        self.nav_interface = nav.acquire_interface()
        self.ros_publisher = RosRoutePublisher(self.meters_per_unit)
        self.route = []
        self.done = False
        self.follow_active = False
        self.follow_complete = False
        self.follow_route_index = 0
        self.follow_last_time = time.monotonic()
        self.follow_last_log_time = self.follow_last_time
        self.car_controller = None
        self.car_world_z = None
        self.retry_count = 0
        self.last_attempt_time = time.monotonic()
        self.event_dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self.update_subscription = self.event_dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            observer_name="RouteApplication0816.Update",
            on_event=self._on_update,
        )

        log(
            "INFO",
            "START",
            "meters_per_unit={:.6f} wheelbase_m={:.5f} track_width_m={:.5f} "
            "max_steering_deg={:.1f} min_turn_radius_m={:.3f}".format(
                self.meters_per_unit,
                WHEELBASE_M,
                TRACK_WIDTH_M,
                MAX_STEERING_DEG,
                WHEELBASE_M / math.tan(math.radians(MAX_STEERING_DEG)),
            ),
        )
        self.try_plan()

    def close(self):
        self.update_subscription = None
        self.event_dispatcher = None
        self.ros_publisher.close()

    def _required_prims(self):
        car = self.stage.GetPrimAtPath(CAR_PATH)
        goal = self.stage.GetPrimAtPath(GOAL_PATH)
        if not car.IsValid():
            raise RuntimeError(f"Car prim not found: {CAR_PATH}")
        if not goal.IsValid():
            raise RuntimeError(f"Goal prim not found: {GOAL_PATH}")

        waypoint = None
        if USE_WAYPOINT:
            if WAYPOINT_PATH:
                waypoint = self.stage.GetPrimAtPath(WAYPOINT_PATH)
                if not waypoint.IsValid():
                    raise RuntimeError(f"Waypoint prim not found: {WAYPOINT_PATH}")
            else:
                waypoint = find_unique_prim_by_name(self.stage, WAYPOINT_NAME)
        return car, goal, waypoint

    def try_plan(self):
        self.retry_count += 1
        try:
            navmesh = self.nav_interface.get_navmesh()
            if navmesh is None:
                log(
                    "WARN",
                    "NAVMESH_NOT_READY",
                    f"attempt={self.retry_count}/{PLAN_MAX_RETRIES}",
                )
                return False

            car, goal, waypoint = self._required_prims()
            car_position, car_rotation = get_world_pose(car)
            goal_position, goal_rotation = get_world_pose(goal)
            start_yaw = quaternion_wxyz_to_yaw(car_rotation)

            planner = AckermannNavMeshPlanner(navmesh, self.meters_per_unit)
            complete_route = []
            current_pose = np.array(
                [car_position[0], car_position[1], car_position[2], start_yaw],
                dtype=np.float64,
            )

            targets = []
            if waypoint is not None:
                waypoint_position, waypoint_rotation = get_world_pose(waypoint)
                targets.append(("car_to_waypoint", waypoint_position))
            targets.append(("to_destination", goal_position))

            for leg_name, target in targets:
                leg = planner.plan_leg(current_pose, target, leg_name)
                if complete_route:
                    complete_route.extend(leg[1:])
                else:
                    complete_route.extend(leg)
                final = complete_route[-1]
                current_pose = np.array(
                    [final[0], final[1], final[2], final[3]],
                    dtype=np.float64,
                )

            planner.validate_route(complete_route)
            RouteDisplay(
                self.stage,
                navmesh,
                self.meters_per_unit,
            ).draw(complete_route)
            self.ros_publisher.publish(complete_route)
            self.route = complete_route
            self.done = True
            if ENABLE_AUTO_FOLLOW:
                timeline = omni.timeline.get_timeline_interface()
                if (
                    PAUSE_TIMELINE_FOR_KINEMATIC_FOLLOW
                    and timeline.is_playing()
                ):
                    timeline.pause()
                    log(
                        "WARN",
                        "TIMELINE_PAUSED",
                        "Kinematic follower owns the vehicle transform",
                    )
                self.car_controller = XFormPrim(CAR_PATH)
                current_position, current_orientation = (
                    self.car_controller.get_world_pose()
                )
                self.car_world_z = float(current_position[2])
                self.follow_route_index = 0
                self.follow_last_time = time.monotonic()
                self.follow_last_log_time = self.follow_last_time
                self.follow_active = True
                log(
                    "INFO",
                    "FOLLOW_READY",
                    "speed_m_s={:.2f} lookahead_m={:.2f} max_steering_deg={:.1f}".format(
                        FOLLOW_SPEED_M_S,
                        FOLLOW_LOOKAHEAD_M,
                        MAX_STEERING_DEG,
                    ),
                )
            log("INFO", "PLAN_COMPLETE", f"points={len(self.route)}")
            return True

        except Exception as exc:
            log("ERROR", "PLAN_FAILED", f"{type(exc).__name__}: {exc}")
            if PRINT_TRACEBACK:
                log(
                    "ERROR",
                    "TRACEBACK",
                    traceback.format_exc().replace("\n", " | "),
                )
            self.done = True
            return False

    def _follow_step(self):
        if (
            not self.follow_active
            or self.follow_complete
            or self.car_controller is None
            or len(self.route) < 2
        ):
            return

        current_time = time.monotonic()
        dt = min(
            max(current_time - self.follow_last_time, 0.0),
            FOLLOW_MAX_DT_S,
        )
        self.follow_last_time = current_time
        if dt <= 1e-5:
            return

        position, orientation = self.car_controller.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_wxyz_to_yaw(orientation)
        goal = self.route[-1]
        goal_distance = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
        arrival_tolerance = FOLLOW_ARRIVAL_TOLERANCE_M / self.meters_per_unit

        if goal_distance <= arrival_tolerance:
            final_position = np.array(
                [float(goal[0]), float(goal[1]), float(self.car_world_z)],
                dtype=np.float32,
            )
            final_yaw = float(goal[3])
            final_orientation = np.array(
                [
                    math.cos(0.5 * final_yaw),
                    0.0,
                    0.0,
                    math.sin(0.5 * final_yaw),
                ],
                dtype=np.float32,
            )
            self.car_controller.set_world_pose(
                position=final_position,
                orientation=final_orientation,
            )
            self.follow_complete = True
            self.follow_active = False
            log("INFO", "GOAL_REACHED", f"route_index={self.follow_route_index}")
            return

        search_start = max(0, self.follow_route_index - 10)
        search_end = min(len(self.route), self.follow_route_index + 500)
        nearest_index = min(
            range(search_start, search_end),
            key=lambda index: math.hypot(
                float(self.route[index][0]) - x,
                float(self.route[index][1]) - y,
            ),
        )
        self.follow_route_index = max(self.follow_route_index, nearest_index)

        lookahead = FOLLOW_LOOKAHEAD_M / self.meters_per_unit
        target_index = self.follow_route_index
        accumulated = 0.0
        while target_index < len(self.route) - 1 and accumulated < lookahead:
            first = self.route[target_index]
            second = self.route[target_index + 1]
            accumulated += math.hypot(
                float(second[0]) - float(first[0]),
                float(second[1]) - float(first[1]),
            )
            target_index += 1

        target = self.route[target_index]
        dx = float(target[0]) - x
        dy = float(target[1]) - y
        target_distance = max(math.hypot(dx, dy), 1e-6)
        alpha = normalize_angle(math.atan2(dy, dx) - yaw)
        wheelbase = WHEELBASE_M / self.meters_per_unit
        steering = math.atan2(
            2.0 * wheelbase * math.sin(alpha),
            target_distance,
        )
        maximum_steering = math.radians(MAX_STEERING_DEG)
        steering = max(-maximum_steering, min(maximum_steering, steering))

        steering_ratio = abs(steering) / max(maximum_steering, 1e-9)
        speed_m_s = max(
            FOLLOW_MIN_SPEED_M_S,
            FOLLOW_SPEED_M_S * (1.0 - 0.55 * steering_ratio),
        )
        slow_distance = FOLLOW_SLOW_DISTANCE_M / self.meters_per_unit
        if goal_distance < slow_distance:
            speed_m_s = max(
                FOLLOW_MIN_SPEED_M_S,
                speed_m_s * goal_distance / max(slow_distance, 1e-9),
            )

        speed = speed_m_s / self.meters_per_unit
        next_x = x + speed * math.cos(yaw) * dt
        next_y = y + speed * math.sin(yaw) * dt
        next_yaw = normalize_angle(
            yaw + speed / wheelbase * math.tan(steering) * dt
        )
        next_position = np.array(
            [next_x, next_y, float(self.car_world_z)],
            dtype=np.float32,
        )
        next_orientation = np.array(
            [
                math.cos(0.5 * next_yaw),
                0.0,
                0.0,
                math.sin(0.5 * next_yaw),
            ],
            dtype=np.float32,
        )
        self.car_controller.set_world_pose(
            position=next_position,
            orientation=next_orientation,
        )

        if current_time - self.follow_last_log_time >= FOLLOW_LOG_INTERVAL_S:
            self.follow_last_log_time = current_time
            log(
                "INFO",
                "FOLLOW",
                "index={} target={} speed_m_s={:.2f} steering_deg={:.1f} "
                "goal_distance_m={:.2f}".format(
                    self.follow_route_index,
                    target_index,
                    speed_m_s,
                    math.degrees(steering),
                    goal_distance * self.meters_per_unit,
                ),
            )

    def _on_update(self, event):
        if self.done:
            self._follow_step()
            return
        if self.retry_count >= int(PLAN_MAX_RETRIES):
            return
        current_time = time.monotonic()
        if current_time - self.last_attempt_time >= float(PLAN_RETRY_INTERVAL_S):
            self.last_attempt_time = current_time
            self.try_plan()


if hasattr(builtins, "_ROUTE_APPLICATION_0816"):
    old_application = builtins._ROUTE_APPLICATION_0816
    if old_application is not None:
        try:
            old_application.close()
        except Exception:
            pass

builtins._ROUTE_APPLICATION_0816 = RouteApplication()
