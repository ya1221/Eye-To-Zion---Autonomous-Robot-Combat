import json
import math
import os
import time
from collections import deque

from tactical_brain import world_model
from tactical_brain import A_planner
from tactical_brain import ballistics_helper
from tactical_brain import aim_controller
from tactical_brain import geometry_utils
from tactical_brain.localization_bridge import LocalizationBridge
from tactical_brain.team_comms import TeamComms
from tactical_brain.shooting_control import ShootingControl
from tactical_brain.path_executor import PathExecutor

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import py_trees

# attack_branch's engagement range - tied directly to the gun's actual
# firing envelope (ballistics_helper.MAX_FIRING_DISTANCE = 3.0m) instead
# of a separate hardcoded 5.0. With 5.0 the robot would commit to an
# engagement (stop patrolling, creep, align, log "Engaging enemy") from
# 3-5m, where trigger_controller.evaluate can never actually fire
# (dist > MAX_FIRING_DISTANCE) - it just looked like "aims but never
# shoots". Single source of truth so the two can't drift apart again.
MAX_COMBAT_DISTANCE = ballistics_helper.MAX_FIRING_DISTANCE

# A raw /ai/detections reading older than this is treated as "no current
# heading fix" rather than trusted stale data - two missed tree ticks
# (tree runs every 0.5s) worth of margin for a dropped frame or two.
DETECTION_STALE_SEC = 1.0

# How far RunFromEnemyAction retreats along the away-from-threat bearing
# on each replan (see PathExecutor.REPLAN_COOLDOWN_SEC for how often
# that's re-evaluated).
RETREAT_DISTANCE_METERS = 1.5

# AreTeammatesComing's threshold: a teammate within this radius is close
# enough to be treated as "arriving to help" rather than "far away".
TEAMMATE_HELP_ARRIVAL_RADIUS = 2.5

# IsAtFinalGoal's threshold - placeholder pending a real measurement of
# the physical capture zone, same caveat as ARENA_SIZE_METERS.
CAPTURE_ZONE_RADIUS_METERS = 0.5

# hide_x/y's "retreat along my own trail" memory (see safe_waypoints):
# only record a new safe waypoint once we've moved at least this far from
# the last one, so it doesn't fill up with near-duplicates while sitting
# still or moving slowly.
MIN_SAFE_WAYPOINT_SPACING_METERS = 0.5
# Bounded history size - old waypoints age out once this many newer ones
# have been recorded (deque(maxlen=...) below), so this doesn't grow
# unbounded over a long match.
MAX_SAFE_WAYPOINTS = 20

#######################
# attack branch
class IsEnemyClose(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_enemy_close"):
        super(IsEnemyClose, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        dist= self.blackboard.dist_to_closest_enemy
        if dist is not None and dist <= MAX_COMBAT_DISTANCE:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class HasLineOfSightToEnemy(py_trees.behaviour.Behaviour):
    def __init__(self, name = "has_line_of_sight_to_enemy"):
        super(HasLineOfSightToEnemy, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="has_line_of_sight_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        has_line_of_sight_to_enemy = self.blackboard.has_line_of_sight_to_closest_enemy
        if has_line_of_sight_to_enemy is not None and has_line_of_sight_to_enemy:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AlignToEnemyAction(py_trees.behaviour.Behaviour):
    """Creeps/steers toward the tracked enemy's bearing (see aim_controller.py).

    Publishes directly onto /cmd_vel - the same topic Nav2's controller
    normally writes to. This works without any new arbitration because
    attack_branch preempts patrol_branch in the root Selector's priority
    order: the instant this branch is selected, patrol_branch's
    MapsToGoalAction simply isn't ticked, and its PathExecutor.terminate()
    already cancels any in-flight FollowPath on invalidation, so Nav2
    stops writing to cmd_vel on its own.

    Always returns SUCCESS (never RUNNING) so AttackAction still
    evaluates the trigger every tick regardless of how converged
    alignment is yet - alignment and firing run concurrently, neither
    gates the other.
    """
    def __init__(self, name="align_to_enemy_action", ros_node=None):
        super(AlignToEnemyAction, self).__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_heading_error", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="dist_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        heading_error = self.blackboard.current_heading_error
        cmd = Twist()
        if heading_error is not None:
            # Pass dist_to_closest_enemy so compute_alignment_twist can hold
            # at aim_controller.STANDOFF_DISTANCE_METERS instead of creeping
            # all the way to point-blank/collision range against the target
            # while firing.
            cmd.linear.x, cmd.angular.z = aim_controller.compute_alignment_twist(
                heading_error, self.blackboard.dist_to_closest_enemy
            )
        # else: no fresh detection - publish the zero Twist() default
        # rather than creep blindly on a stale/missing heading error.
        self.ros_node.cmd_vel_pub.publish(cmd)
        return py_trees.common.Status.SUCCESS

class AttackAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "attack_action", ros_node=None):
        super(AttackAction, self).__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="dist_to_closest_enemy", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_heading_error", access=py_trees.common.Access.READ)

    def update(self):
        self.blackboard.robot_command = "ATTACK"
        self.ros_node.get_logger().info(f"[{self.name}] Engaging enemy at ({self.blackboard.current_x:.2f}, {self.blackboard.current_y:.2f})")

        # Ballistics/trigger decision - see ballistics_helper.py and
        # shooting_control.py. ShootingControl lives on ros_node (not on
        # this behaviour) because the safety net in
        # update_blackboard_from_world_state needs to reset/cease-fire the
        # same instance on ticks where this action never runs at all - a
        # single-tick, always-SUCCESS leaf like this one can't rely on
        # terminate()/initialise() to detect "I was skipped this tick".
        mode = self.ros_node.get_parameter('shooting_mode').value
        self.ros_node.shooting_control.evaluate_and_fire(
            self.blackboard.dist_to_closest_enemy, self.blackboard.current_heading_error, mode
        )
        return py_trees.common.Status.SUCCESS

#######################
# survival branch
class IsLowHealthOrOutnumbered(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_low_health_or_outnumbered"):
        super(IsLowHealthOrOutnumbered, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        # am_i_in_danger is computed once in update_blackboard_from_world_
        # state (the exact same health/outnumbered check this class used
        # to duplicate) - shared with _publish_team_status's distress-call
        # broadcast so the "what counts as danger" threshold lives in
        # exactly one place.
        self.blackboard.register_key(key="am_i_in_danger", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.am_i_in_danger:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AreTeammatesComing(py_trees.behaviour.Behaviour):
    def __init__(self, name="are_teammates_coming"):
        super(AreTeammatesComing, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_nearest_teammate", access=py_trees.common.Access.READ)

    def update(self):
        dist = self.blackboard.dist_to_nearest_teammate
        if dist is not None and dist <= TEAMMATE_HELP_ARRIVAL_RADIUS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class HideAndWaitAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="hide_and_wait_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        # Composition, not inheritance: this leaf owns its goal-selection
        # policy (below) and hands it to a PathExecutor as a callable, the
        # same shape the old get_tactical_path() override had - see
        # path_executor.py's module docstring.
        self.path_executor = PathExecutor(ros_node, name) if ros_node is not None else None
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_y", access=py_trees.common.Access.READ)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        hx, hy = self.blackboard.hide_x, self.blackboard.hide_y

        if hx is not None and hy is not None and self.ros_node is not None:
            self.blackboard.robot_command = "HIDING (A* PATH)"

            start_pos = (cx, cy, cyaw)
            # calc_hybrid_a_star's stop condition only checks x/y distance
            # (A_planner.py:100-101) - goal yaw is currently inert either
            # way - but compute it from the vector to goal anyway instead
            # of a hardcoded value, since "face east" has no more tactical
            # meaning here than "face the way you're driving."
            goal_pos = (hx, hy, math.atan2(hy - cy, hx - cx))

            self.ros_node.get_logger().info(f"[{self.name}] Calculating A* evasion route...")

            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )

            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] No safe evasion path found!")
                return None

            path_obj, _ = result
            x_path, y_path, theta_path, directions = path_obj

            tactical_waypoints = [(x_path[i], y_path[i], theta_path[i], directions[i]) for i in range(len(x_path))]
            return tactical_waypoints

        return None

    def update(self):
        if self.ros_node is None:
            return py_trees.common.Status.SUCCESS
        return self.path_executor.tick(self.get_tactical_path)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.path_executor is not None:
            self.path_executor.terminate()

class RunFromEnemyAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="run_from_enemy_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.path_executor = PathExecutor(ros_node, name) if ros_node is not None else None
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="closest_enemy_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="closest_enemy_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_y", access=py_trees.common.Access.READ)

    def get_tactical_path(self):
        if self.ros_node is None:
            return None

        self.blackboard.robot_command = "RUNNING_FROM_ENEMY"
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        ex, ey = self.blackboard.closest_enemy_x, self.blackboard.closest_enemy_y

        if ex is not None and ey is not None:
            # Straight-line retreat directly away from the threat's bearing.
            away_angle = math.atan2(cy - ey, cx - ex)
            gx = cx + RETREAT_DISTANCE_METERS * math.cos(away_angle)
            gy = cy + RETREAT_DISTANCE_METERS * math.sin(away_angle)
        else:
            # In danger (e.g. low health/outnumbered) with no specific
            # threat position known - fall back to the designated rally
            # point rather than fleeing in an undefined direction.
            gx, gy = self.blackboard.hide_x, self.blackboard.hide_y
            if gx is None or gy is None:
                return None

        start_pos = (cx, cy, cyaw)
        goal_pos = (gx, gy, math.atan2(gy - cy, gx - cx))

        result = A_planner.calc_hybrid_a_star(
            start=start_pos,
            goal=goal_pos,
            obstacle_set=self.ros_node.static_obstacles,
            xy_resolution=A_planner.XY_RESOLUTION,
            yaw_resolution=A_planner.YAW_RESOLUTION,
            danger_dict=self.ros_node.danger_dict,
            teammates_aura_set=self.ros_node.teammates_aura_set
        )
        if result is None:
            self.ros_node.get_logger().warn(f"[{self.name}] No escape path found!")
            return None

        path_obj, _ = result
        x_path, y_path, theta_path, directions = path_obj
        return [(x_path[i], y_path[i], theta_path[i], directions[i]) for i in range(len(x_path))]

    def update(self):
        if self.ros_node is None:
            return py_trees.common.Status.SUCCESS
        return self.path_executor.tick(self.get_tactical_path)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.path_executor is not None:
            self.path_executor.terminate()

#######################
# help teammate branch
class IsTeammateInDanger(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_teammate_in_danger"):
        super(IsTeammateInDanger, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key = "teammate_requested_help", access = py_trees.common.Access.READ)

    def update(self):
        teammate_requested_help = self.blackboard.teammate_requested_help
        if teammate_requested_help is not None and teammate_requested_help:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsDistanceToTeammateAcceptable(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_distance_to_teammate_acceptable"):
        super(IsDistanceToTeammateAcceptable, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_help_teammate", access=py_trees.common.Access.READ)

    def update(self):
        dist_to_help_teammate = self.blackboard.dist_to_help_teammate
        if dist_to_help_teammate is not None and dist_to_help_teammate < 5.0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class DriveToTeammateAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="drive_to_teammate_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.path_executor = PathExecutor(ros_node, name) if ros_node is not None else None
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        tx, ty = self.blackboard.teammate_x, self.blackboard.teammate_y

        if tx is not None and ty is not None and self.ros_node is not None:
            self.blackboard.robot_command = "DRIVE_TO_TEAMMATE (A* PATH)"

            start_pos = (cx, cy, cyaw)
            goal_pos = (tx, ty, math.atan2(ty - cy, tx - cx))

            self.ros_node.get_logger().info(f"[{self.name}] Calculating A* rescue route...")

            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )

            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] No path to teammate found!")
                return None

            path_obj, _ = result
            x_path, y_path, theta_path, directions = path_obj

            tactical_waypoints = [(x_path[i], y_path[i], theta_path[i], directions[i]) for i in range(len(x_path))]
            return tactical_waypoints

        return None

    def update(self):
        if self.ros_node is None:
            return py_trees.common.Status.SUCCESS
        return self.path_executor.tick(self.get_tactical_path)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.path_executor is not None:
            self.path_executor.terminate()

#########################
# patrol branch
class MapsToGoalAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="maps_to_goal_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.path_executor = PathExecutor(ros_node, name) if ros_node is not None else None
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        gx, gy = self.blackboard.final_goal_x, self.blackboard.final_goal_y

        if gx is not None and gy is not None and self.ros_node is not None:
            self.blackboard.robot_command = "NAVIGATING (A* PATH)"

            start_pos = (cx, cy, cyaw)
            goal_pos = (gx, gy, math.atan2(gy - cy, gx - cx))

            self.ros_node.get_logger().info(f"[{self.name}] Running Hybrid A* planner...")

            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )

            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] A* Planner failed to find a path!")
                return None

            path_obj, cost = result
            x_path, y_path, theta_path, directions = path_obj

            self.ros_node.get_logger().info(f"[{self.name}] A* found path with {len(x_path)} steps. Cost: {cost:.2f}")

            tactical_waypoints = []
            for i in range(len(x_path)):
                tactical_waypoints.append((x_path[i], y_path[i], theta_path[i], directions[i]))

            return tactical_waypoints

        return None

    def update(self):
        # Before the overhead camera has sighted the capture-zone marker
        # even once, final_goal_x/y are still None. PathExecutor.tick()
        # treats get_tactical_path() returning nothing as an A*-failed-to-
        # find-a-path signal and starts counting toward a recovery
        # backup-nudge after CONSECUTIVE_PLAN_FAILURE_THRESHOLD ticks - the
        # robot isn't stuck here, it's just waiting on the camera, so skip
        # straight to RUNNING instead of letting that bookkeeping misfire.
        if self.blackboard.final_goal_x is None or self.blackboard.final_goal_y is None:
            return py_trees.common.Status.RUNNING
        if self.ros_node is None:
            return py_trees.common.Status.SUCCESS
        return self.path_executor.tick(self.get_tactical_path)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.path_executor is not None:
            self.path_executor.terminate()

#######################
# squad roles - deterministic, negotiation-free PUSHER/SCREEN split.
# Every robot runs the exact same comparison independently from already-
# shared state (own position, teammate's broadcast position, the locked
# final_goal) - no "I'm pushing" message is ever exchanged, so there's no
# leader/negotiation protocol to fail if a teammate drops out or dies.
class IsCloserToGoal(py_trees.behaviour.Behaviour):
    """SUCCESS => this robot takes the PUSHER role this tick."""
    def __init__(self, name="is_closer_to_goal"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_y", access=py_trees.common.Access.READ)

    def update(self):
        gx, gy = self.blackboard.final_goal_x, self.blackboard.final_goal_y
        if gx is None or gy is None:
            return py_trees.common.Status.FAILURE

        current_pos = (self.blackboard.current_x, self.blackboard.current_y)
        my_dist = distance(current_pos, (gx, gy))

        tx, ty = self.blackboard.teammate_x, self.blackboard.teammate_y
        if tx is None or ty is None:
            # No teammate known at all - default to pushing rather than
            # leaving the zone uncontested.
            return py_trees.common.Status.SUCCESS

        teammate_dist = distance((tx, ty), (gx, gy))
        return (
            py_trees.common.Status.SUCCESS if my_dist <= teammate_dist
            else py_trees.common.Status.FAILURE
        )

class IsAtFinalGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="is_at_final_goal"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_y", access=py_trees.common.Access.READ)

    def update(self):
        gx, gy = self.blackboard.final_goal_x, self.blackboard.final_goal_y
        if gx is None or gy is None:
            return py_trees.common.Status.FAILURE
        current_pos = (self.blackboard.current_x, self.blackboard.current_y)
        d = distance(current_pos, (gx, gy))
        return (
            py_trees.common.Status.SUCCESS if d <= CAPTURE_ZONE_RADIUS_METERS
            else py_trees.common.Status.FAILURE
        )

class HoldZoneAction(py_trees.behaviour.Behaviour):
    """Stop and hold once inside the capture zone, instead of letting
    MapsToGoalAction keep re-triggering trivial/zero-length A* replans.
    survival_branch/attack_branch still preempt this every tick via the
    root Selector's priority order (same mechanism AlignToEnemyAction's
    docstring describes for attack_branch preempting patrol_branch), and
    switching away from this back to MapsToGoalAction (e.g. pushed off
    the zone) is handled for free by PathExecutor.terminate() canceling
    whatever FollowPath goal was in flight.
    """
    def __init__(self, name="hold_zone_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.robot_command = "HOLDING_ZONE"
        if self.ros_node is not None:
            self.ros_node.cmd_vel_pub.publish(Twist())
        return py_trees.common.Status.SUCCESS

class ScreenAction(py_trees.behaviour.Behaviour):
    """The SCREEN role with no enemy currently in sight (attack_branch
    already outranks this branch whenever one is, so this only ever runs
    when there's nothing to engage) - patrol the midpoint between the
    capture zone and the arena center, a generic "stay between the
    objective and the likely enemy approach" position, so the PUSHER
    doesn't have to cross open ground alone.
    """
    def __init__(self, name="screen_action", ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.path_executor = PathExecutor(ros_node, name) if ros_node is not None else None
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="final_goal_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        if self.ros_node is None:
            return None

        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        gx, gy = self.blackboard.final_goal_x, self.blackboard.final_goal_y
        if gx is None or gy is None:
            return None

        arena_center = self.ros_node.arena_size_meters / 2.0
        target_x = (gx + arena_center) / 2.0
        target_y = (gy + arena_center) / 2.0

        self.blackboard.robot_command = "SCREENING"
        start_pos = (cx, cy, cyaw)
        goal_pos = (target_x, target_y, math.atan2(target_y - cy, target_x - cx))

        result = A_planner.calc_hybrid_a_star(
            start=start_pos,
            goal=goal_pos,
            obstacle_set=self.ros_node.static_obstacles,
            xy_resolution=A_planner.XY_RESOLUTION,
            yaw_resolution=A_planner.YAW_RESOLUTION,
            danger_dict=self.ros_node.danger_dict,
            teammates_aura_set=self.ros_node.teammates_aura_set
        )
        if result is None:
            return None

        path_obj, _ = result
        x_path, y_path, theta_path, directions = path_obj
        return [(x_path[i], y_path[i], theta_path[i], directions[i]) for i in range(len(x_path))]

    def update(self):
        # Same reasoning as MapsToGoalAction's guard: before final_goal is
        # locked, get_tactical_path() has nothing to compute a midpoint
        # from. Unlike MapsToGoalAction (the tree's last resort), FAILURE
        # is correct here, not RUNNING - patrol_branch is the real
        # fallback for this case, and it's role_branch's sibling in the
        # root Selector, so role_branch must actually fail (both
        # pusher_branch and this) for the Selector to reach it.
        if self.blackboard.final_goal_x is None or self.blackboard.final_goal_y is None:
            return py_trees.common.Status.FAILURE
        if self.ros_node is None:
            return py_trees.common.Status.SUCCESS
        return self.path_executor.tick(self.get_tactical_path)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.path_executor is not None:
            self.path_executor.terminate()

# ==========================================
# פונקציית יצירת עץ ההתנהגות
# ==========================================
def create_tree(ros_node=None):
    survival_branch = py_trees.composites.Sequence(name="survival_branch", memory=False)
    tactical_reaction_selector = py_trees.composites.Selector(name="tactical_reaction_selector", memory=False)

    wait_for_teammates_branch = py_trees.composites.Sequence(name="wait_for_teammates_branch", memory=False)
    wait_for_teammates_branch.add_children([
        AreTeammatesComing(name="teammates coming?"),
        HideAndWaitAction(name="hide and wait action", ros_node=ros_node)
    ])

    tactical_reaction_selector.add_children([
        wait_for_teammates_branch,
        RunFromEnemyAction(name="run from enemy action", ros_node=ros_node),
    ])

    survival_branch.add_children([
        IsLowHealthOrOutnumbered(name="in danger?"),
        tactical_reaction_selector
    ])

    attack_branch = py_trees.composites.Sequence(name="attack_branch", memory = False)
    attack_branch.add_children(
        [IsEnemyClose(name = "is enemy close"),
         HasLineOfSightToEnemy(name = "has line of sight to enemy"),
         AlignToEnemyAction(name = "align to enemy action", ros_node=ros_node),
         AttackAction(name = "attack action", ros_node=ros_node)])

    help_teammate_branch = py_trees.composites.Sequence(name="help_teammate_branch", memory = False)
    help_teammate_branch.add_children(
        [IsTeammateInDanger(name = "is teammate in danger"),
         IsDistanceToTeammateAcceptable(name = "is distance to teammate acceptable"),
         DriveToTeammateAction(name = "drive to teammate action", ros_node=ros_node)])

    # Squad roles: PUSHER drives to final_goal and holds once there, SCREEN
    # roams to intercept. Sits below help_teammate_branch (a dying
    # teammate outranks your assigned role) and above patrol_branch, which
    # remains as the fallback for the one case role_branch can't resolve -
    # final_goal not locked in yet (see IsCloserToGoal/ScreenAction, both
    # of which FAILURE in that case, same as MapsToGoalAction's own guard).
    hold_zone_sequence = py_trees.composites.Sequence(name="hold_zone_sequence", memory=False)
    hold_zone_sequence.add_children([
        IsAtFinalGoal(name="is at final goal"),
        HoldZoneAction(name="hold zone action", ros_node=ros_node)
    ])

    pusher_action_selector = py_trees.composites.Selector(name="pusher_action_selector", memory=False)
    pusher_action_selector.add_children([
        hold_zone_sequence,
        MapsToGoalAction(name="maps to goal action (pusher)", ros_node=ros_node)
    ])

    pusher_branch = py_trees.composites.Sequence(name="pusher_branch", memory=False)
    pusher_branch.add_children([
        IsCloserToGoal(name="is closer to goal"),
        pusher_action_selector
    ])

    role_branch = py_trees.composites.Selector(name="role_branch", memory=False)
    role_branch.add_children([
        pusher_branch,
        ScreenAction(name="screen action", ros_node=ros_node)
    ])

    patrol_branch = py_trees.composites.Sequence(name="patrol_branch", memory = False)
    patrol_branch.add_children(
        [MapsToGoalAction(name = "maps to goal action (fallback)", ros_node=ros_node)])

    root = py_trees.composites.Selector(name="root", memory = False)
    root.add_children([survival_branch, attack_branch, help_teammate_branch, role_branch, patrol_branch])

    return py_trees.trees.BehaviourTree(root)


# ==========================================
# קוד ה-ROS2 Node (המעטפת שמריצה ובודקת את העץ)
# ==========================================
class TacticalBrainNode(Node):
    def __init__(self):
        super().__init__('tactical_brain_node')
        self.get_logger().info("Tactical Brain is waking up and planting the tree...")

        # Our identity on the overhead-camera tracker (PureModuloTrackerNode,
        # relayed to us via zenoh-bridge-ros2dds): which ArUco-marker id is
        # "us" within our own team's positions array, which numeric team_idx
        # we are (camera assigns team = marker_id % CNT_TEAM), and the
        # grid_n/arena_size_meters scale factor to convert the camera's
        # calibrated 0..grid_n perspective-grid units into real meters.
        self.declare_parameter('robot_id', int(os.environ.get('ROBOT_ID', 3)))
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.declare_parameter('my_team_idx', int(os.environ.get('MY_TEAM_IDX', 0)))
        self.my_team_idx = self.get_parameter('my_team_idx').get_parameter_value().integer_value
        self.declare_parameter('arena_size_meters', float(os.environ.get('ARENA_SIZE_METERS', 2.0)))
        self.declare_parameter('grid_n', int(os.environ.get('GRID_N', 2000)))
        self.arena_size_meters = self.get_parameter('arena_size_meters').get_parameter_value().double_value
        self.grid_to_meters = (
            self.arena_size_meters
            / self.get_parameter('grid_n').get_parameter_value().integer_value
        )

        # A*'s plannable-region geofence (A_planner.check_collision). This
        # is the real DRIVABLE arena extent, deliberately SEPARATE from
        # arena_size_meters above (which is the camera perspective-grid
        # scale, not a drivable size - conflating them would strangle the
        # 5m sim maze down to a 2m box). Default 5.0 reproduces the old
        # hardcoded 0.1/4.9 geofence exactly, so sim is unchanged; on the
        # real robot set PLANNING_ARENA_SIZE_METERS to the true arena side
        # length so A* won't plan a node off the actual boundary.
        self.declare_parameter(
            'planning_arena_size_meters',
            float(os.environ.get('PLANNING_ARENA_SIZE_METERS', 5.0))
        )
        A_planner.set_arena_bounds(
            self.get_parameter('planning_arena_size_meters').get_parameter_value().double_value
        )

        # Real pose source is /odometry/global, robot_localization's
        # ekf_global node (map->odom, ArUco-corrected - see
        # localization/launch.py on the rpi5 branch) - NOT /amcl_pose,
        # since this stack runs slam_toolbox rather than amcl. hardware's
        # pid_controller.cpp subscribes to this exact same topic, so this
        # and the low-level controller are guaranteed to agree on pose.
        #
        # Dedicated callback group + MultiThreadedExecutor (see main()):
        # the default single-threaded executor processes every callback
        # of a node strictly one at a time, on one thread. sense_and_think
        # (the 0.5s tree timer, which runs A*'s search) and map_callback
        # (looping over the full occupancy grid) are both expensive enough
        # to block that thread for a while - during which pose_callback
        # never got a turn to run, leaving current_x/current_y frozen at
        # whatever they were when the blocking started. Confirmed directly:
        # logged A* start position stayed bit-for-bit identical across
        # multiple planning cycles spanning ~24s while the real robot had
        # already driven over a meter away. Every path sent this session
        # was potentially planned from a stale position as a result.
        self.pose_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.pose_callback,
            10,
            callback_group=self.pose_callback_group
        )

        # zenoh-bridge-ros2dds (a separate container, not our code) mirrors
        # the overhead camera's real ROS2 topics across Zenoh transparently -
        # we just subscribe to the plain ROS2 topics like any other sensor,
        # no custom Zenoh client needed on our side anymore. Same callback
        # group as pose_callback so these small JSON updates don't queue
        # behind the tree timer's A* search.
        self.my_team_sub = self.create_subscription(
            String,
            f'teams/team_{self.my_team_idx}/positions',
            self.my_team_positions_callback,
            10,
            callback_group=self.pose_callback_group
        )

        # Capture-zone marker (physically static, one per team) tracked by
        # the same overhead-camera node as teams/team_X/positions, mirrored
        # across Zenoh the same way. Locked onto the first sighting in
        # final_goal_callback rather than kept live, since re-reading a
        # static marker's per-frame ArUco jitter would retarget MapsToGoal's
        # A* goal for no reason.
        self.final_goal_locked = False
        self.final_goal_sub = self.create_subscription(
            String,
            f'teams/team_{self.my_team_idx}/target_position',
            self.final_goal_callback,
            10,
            callback_group=self.pose_callback_group
        )
        # אויבים מגיעים מהזיהוי המקומי (YOLO+לידאר) של sensor_fusion_node,
        # לא מטופיק הקבוצה היריבה - teams/team_X/positions הוא הברודקאסט
        # העצמי של הקבוצה ההיא, לא משהו שאנחנו אמורים להאזין לו בכלל.
        self.local_enemy_sub = self.create_subscription(
            PoseStamped,
            '/sensor_fusion_node/local_enemy_position',
            self.local_enemy_position_callback,
            10,
            callback_group=self.pose_callback_group
        )

        # Raw YOLO relative bearing (same /ai/detections + 'angle' (deg)
        # field sensor_fusion_node.py itself parses to fuse against lidar)
        # read directly here for the attack branch's heading error - this
        # is our own camera's live relative angle to a target, not the
        # fused/team-broadcast global (x,y) position, which is exactly
        # what's needed for "is the gun pointed at what I'm actually
        # seeing right now" rather than a team-shared, possibly
        # teammate-detected position.
        self._latest_heading_error_deg = None
        self._latest_detection_time = None
        self.ai_detections_sub = self.create_subscription(
            String,
            '/ai/detections',
            self.ai_detections_callback,
            10,
            callback_group=self.pose_callback_group
        )

        # Team-shared enemy sightings (both broadcasting my own and
        # receiving teammates') and teammate position tracking - see
        # team_comms.py for the protocol/locking details.
        self.team_comms = TeamComms(
            self, self.my_team_idx, self.robot_id, callback_group=self.pose_callback_group
        )

        # Map<->world TF reconciliation and ArUco->EKF publishing - see
        # localization_bridge.py.
        self.localization_bridge = LocalizationBridge(self)

        # Attack branch shooting outputs - see shooting_control.py and
        # ballistics_helper.py.
        self.shooting_control = ShootingControl(self)

        self.declare_parameter('shooting_mode', 'single')   # 'single' or 'auto'
        self.declare_parameter('auto_fire_rate_hz', 2.0)
        self.declare_parameter('max_fire_angle_deg', 30.0)

        # AlignToEnemyAction's output (see aim_controller.py). Same topic
        # Nav2's controller normally publishes to - safe because
        # attack_branch preempts patrol_branch (see AlignToEnemyAction's
        # docstring), so only one of the two is ever actively driving.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. אתחול משתני מצב מקומיים (world-model state that only we hold -
        # enemies/teammates now live in team_comms, see above)
        self.static_obstacles = set()
        self.enemies_list = []
        self.danger_dict = {}
        self.teammates_aura_set = set()

        # hide_x/y's "retreat along my own trail" fallback (see
        # update_blackboard_from_world_state) - starts empty and gets
        # seeded with the real starting position on the first pose_
        # callback firing (not the current_x/y placeholder default), since
        # that's the actual point the robot begins from.
        self.safe_waypoints = deque(maxlen=MAX_SAFE_WAYPOINTS)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        # 1. רישום כל המשתנים שהעץ צריך ב-Blackboard
        self.blackboard = py_trees.blackboard.Client(name="global")
        keys = [
            "current_x", "current_y", "current_yaw", "hide_x", "hide_y", "final_goal_x", "final_goal_y", "teammate_x", "teammate_y",
            "dist_to_closest_enemy", "has_line_of_sight_to_closest_enemy", "current_heading_error",
            "closest_enemy_x", "closest_enemy_y", "dist_to_nearest_teammate", "am_i_in_danger", "am_i_fighting",
            "health", "num_enemies", "num_teammates",
            "teammate_requested_help", "dist_to_help_teammate", "robot_command"
        ]
        for key in keys:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

        # מצב התחלתי שקט
        self.reset_blackboard_to_safe_state()

        # 2. הקמת העץ
        self.tree = create_tree(ros_node=self)
        self.tree.setup(timeout=15)

        # 3. טיימרים
        self.tree_timer = self.create_timer(0.5, self.sense_and_think)


    def reset_blackboard_to_safe_state(self):
        self.blackboard.dist_to_closest_enemy = 100.0
        self.blackboard.has_line_of_sight_to_closest_enemy = False
        self.blackboard.current_heading_error = None
        self.blackboard.closest_enemy_x = None
        self.blackboard.closest_enemy_y = None
        self.blackboard.dist_to_nearest_teammate = 100.0
        self.blackboard.am_i_in_danger = False
        self.blackboard.am_i_fighting = False
        self.blackboard.health = 1.0
        self.blackboard.num_enemies = 0
        self.blackboard.num_teammates = 1
        self.blackboard.teammate_requested_help = False
        self.blackboard.dist_to_help_teammate = 100.0
        self.blackboard.robot_command = "WAIT"

        self.blackboard.current_x = 2.0
        self.blackboard.current_y = 2.0
        self.blackboard.current_yaw = 0.0
        # Recomputed every tick in update_blackboard_from_world_state
        # (nearest safe teammate, else last known-safe waypoint) - these
        # are just the pre-first-tick placeholders.
        self.blackboard.hide_x = 2.0
        self.blackboard.hide_y = 2.0
        # Not known until the overhead camera's capture-zone marker is
        # sighted at least once (see final_goal_callback) - None until
        # then. MapsToGoalAction.update() explicitly treats this as
        # "waiting", not a failed plan.
        self.blackboard.final_goal_x = None
        self.blackboard.final_goal_y = None
        # None (not a placeholder point) until teammates_dict actually has
        # an entry - IsCloserToGoal reads a None teammate position as
        # "no teammate known, default to PUSHER", which a fake stale
        # coordinate would silently defeat.
        self.blackboard.teammate_x = None
        self.blackboard.teammate_y = None

    def sense_and_think(self):
        """
        זוהי לולאת הליבה של הרובוט: Sense -> Think -> Act
        """
        # שלב א': Sense. enemies_by_detector/teammates_dict כבר מתעדכנים
        # ישירות ע"י team_comms (מוזנים מ-local_enemy_position_callback/
        # team_enemy_callback/my_team_positions_callback) - כאן רק שואבים
        # תמונת מצב מנוקה (snapshot) ומחשבים ממנה את הרשתות הטקטיות,
        # בעזרת static_obstacles שרק אנחנו מחזיקים.
        self.enemies_list = self.team_comms.get_enemies_snapshot()
        self.team_comms.prune_stale_help_status()
        self.danger_dict = world_model.get_danger_dict(self.danger_dict, self.enemies_list, self.static_obstacles)
        self.danger_dict = world_model.delete_old_danger_squares(self.danger_dict)
        self.teammates_aura_set = world_model.get_teammates_aura_set(self.team_comms.get_teammates())

        # שלב ב': Translation
        self.update_blackboard_from_world_state()
        # Distress-call broadcast - see team_comms.publish_status. Reads
        # am_i_in_danger/am_i_fighting rather than recomputing either, so
        # this and IsLowHealthOrOutnumbered/attack_branch can never
        # disagree on what those mean.
        self.team_comms.publish_status(self.blackboard.am_i_in_danger, self.blackboard.am_i_fighting)

        # שלב ג': Think
        self.tree.tick()
        self.get_logger().info(f"Tree Output Command ---> {self.blackboard.robot_command}")

    def my_team_positions_callback(self, msg):
        try:
            robots = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        my_entry = next((r for r in robots if r.get('id') == self.robot_id), None)
        if my_entry is None:
            self.get_logger().warn(
                f"Robot id {self.robot_id} not visible in teams/team_{self.my_team_idx}/positions this frame."
            )
        else:
            aruco_x = my_entry['x'] * self.grid_to_meters
            aruco_y = my_entry['y'] * self.grid_to_meters
            aruco_yaw = math.radians(my_entry['angle'])
            self.localization_bridge.publish_aruco_odom(aruco_x, aruco_y, aruco_yaw)
            self.localization_bridge.check_drift(
                aruco_x, aruco_y, aruco_yaw,
                self.blackboard.current_x, self.blackboard.current_y, self.blackboard.current_yaw
            )

        self.team_comms.set_teammates({
            r['id']: {'x': r['x'] * self.grid_to_meters, 'y': r['y'] * self.grid_to_meters}
            for r in robots if r.get('id') != self.robot_id
        })

    def final_goal_callback(self, msg):
        if self.final_goal_locked:
            return
        try:
            target = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.blackboard.final_goal_x = target['x'] * self.grid_to_meters
        self.blackboard.final_goal_y = target['y'] * self.grid_to_meters
        self.final_goal_locked = True
        self.get_logger().info(
            f"Final goal (capture zone) locked at "
            f"({self.blackboard.final_goal_x:.2f}, {self.blackboard.final_goal_y:.2f})"
        )

    def ai_detections_callback(self, msg):
        # Same raw JSON sensor_fusion_node.py parses ('angle' in degrees,
        # first detection in the list - no multi-target selection exists
        # anywhere yet, see that file's fusion_callback). Only the angle is
        # used here; distance/range for firing purposes comes from the
        # already-fused dist_to_closest_enemy on the blackboard, not lidar
        # directly, so the two stay consistent with what IsEnemyClose sees.
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not detections:
            return
        self._latest_heading_error_deg = detections[0].get('angle')
        self._latest_detection_time = time.time()


    def local_enemy_position_callback(self, msg):
        # sensor_fusion_node only publishes when it actually has a fused
        # YOLO+lidar detection (no "no enemy visible" message exists) -
        # staleness pruning happens in sense_and_think via
        # team_comms.get_enemies_snapshot() so an old detection doesn't
        # linger forever once the enemy is no longer seen.
        self.team_comms.record_local_detection(msg.pose.position.x, msg.pose.position.y)

    def update_blackboard_from_world_state(self):
        # 1. עדכון נתוני אויבים
        self.blackboard.num_enemies = len(self.enemies_list)
        current_pos = (self.blackboard.current_x, self.blackboard.current_y)

        if self.enemies_list:
            visible_enemies = []
            hidden_enemies = []

            # המרה לקואורדינטות רשת (גריד) עבור בדיקת קו ראייה
            grid_current_pos = (
                self.blackboard.current_x / A_planner.XY_RESOLUTION,
                self.blackboard.current_y / A_planner.XY_RESOLUTION
            )
            # The enemy's own position is a lidar return SLAM marks
            # occupied, so a ray checked all the way to it always reports
            # "blocked" by the target itself - ignore a margin around both
            # endpoints (see world_model.LOS_ENDPOINT_MARGIN_METERS). In
            # grid cells, since grid_* are in world/XY_RESOLUTION units.
            los_margin_cells = world_model.LOS_ENDPOINT_MARGIN_METERS / A_planner.XY_RESOLUTION

            for enemy in self.enemies_list:
                grid_enemy_pos = (
                    enemy['x'] / A_planner.XY_RESOLUTION,
                    enemy['y'] / A_planner.XY_RESOLUTION
                )
                if world_model.line_of_sight_clear(grid_current_pos, grid_enemy_pos, self.static_obstacles, endpoint_margin_cells=los_margin_cells):
                    visible_enemies.append(enemy)
                else:
                    hidden_enemies.append(enemy)

            if visible_enemies:
                closest_enemy = min(visible_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
                self.blackboard.has_line_of_sight_to_closest_enemy = True
            else:
                closest_enemy = min(hidden_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
                self.blackboard.has_line_of_sight_to_closest_enemy = False

            self.blackboard.dist_to_closest_enemy = distance(current_pos, (closest_enemy['x'], closest_enemy['y']))
            self.blackboard.closest_enemy_x = closest_enemy['x']
            self.blackboard.closest_enemy_y = closest_enemy['y']
        else:
            self.blackboard.dist_to_closest_enemy = 100.0
            self.blackboard.has_line_of_sight_to_closest_enemy = False
            self.blackboard.closest_enemy_x = None
            self.blackboard.closest_enemy_y = None
            # "Didn't encounter any enemies" - record this spot as a safe
            # retreat point for hide_x/y's fallback, provided we've moved
            # far enough from the last one to be worth a new entry.
            if not self.safe_waypoints or distance(current_pos, self.safe_waypoints[-1]) >= MIN_SAFE_WAYPOINT_SPACING_METERS:
                self.safe_waypoints.append(current_pos)

        # Raw camera bearing to whatever /ai/detections last saw, discarded
        # once stale (see DETECTION_STALE_SEC) so a frozen old reading
        # doesn't get treated as a live lock.
        detection_is_fresh = (
            self._latest_detection_time is not None
            and (time.time() - self._latest_detection_time) <= DETECTION_STALE_SEC
        )
        self.blackboard.current_heading_error = self._latest_heading_error_deg if detection_is_fresh else None

        # Attack-branch safety net: this mirrors attack_branch's own
        # IsEnemyClose/HasLineOfSightToEnemy preconditions exactly. When
        # they don't hold, neither AttackAction nor AlignToEnemyAction
        # ever runs this tick (Sequence short-circuits) - so nothing
        # inside either of them ever gets a chance to cease-fire, zero out
        # /cmd_vel, or reset the trigger's frame-delay/hysteresis state.
        # This runs unconditionally every tick (regardless of which BT
        # branch ends up selected), same as dist_to_closest_enemy above,
        # which is the only reliable way to catch "enemy lost" here - see
        # ballistics_helper.py's docstring and AttackAction's comment for
        # why terminate()/initialise() can't do this for a single-tick,
        # always-SUCCESS leaf.
        attack_preconditions_met = (
            self.blackboard.dist_to_closest_enemy <= MAX_COMBAT_DISTANCE
            and self.blackboard.has_line_of_sight_to_closest_enemy
        )
        # Shared with _publish_team_status's is_fighting field (a teammate
        # broadcasting this as True is disqualified as a safe retreat
        # target) - single source of truth, same reasoning as
        # am_i_in_danger below.
        self.blackboard.am_i_fighting = attack_preconditions_met

        self.get_logger().info(
            f"Shooting Inputs Debug | dist: {self.blackboard.dist_to_closest_enemy:.2f}, "
            f"LOS: {self.blackboard.has_line_of_sight_to_closest_enemy}, "
            f"heading_error: {self.blackboard.current_heading_error}, "
            f"preconditions_met: {attack_preconditions_met}"
        )

        if not attack_preconditions_met:
            self.shooting_control.reset()
            self.shooting_control.cease_fire()
            self.cmd_vel_pub.publish(Twist())

        # 2. עדכון נתוני חברי צוות
        teammates = self.team_comms.get_teammates()
        self.blackboard.num_teammates = len(teammates)
        teammate_is_safe_refuge = False
        if teammates:
            first_teammate_id = list(teammates.keys())[0]
            teammate_data = teammates[first_teammate_id]

            self.blackboard.teammate_x = teammate_data.get('x')
            self.blackboard.teammate_y = teammate_data.get('y')
            self.blackboard.dist_to_nearest_teammate = distance(
                current_pos,
                (self.blackboard.teammate_x, self.blackboard.teammate_y)
            )
            # needs_help/is_fighting come from the same teammate's own
            # distress-call broadcast (teams/team_X/status, see
            # team_comms.py), not from this position broadcast, which
            # never carries either.
            help_status = self.team_comms.get_teammate_help_status(first_teammate_id)
            self.blackboard.dist_to_help_teammate = self.blackboard.dist_to_nearest_teammate
            self.blackboard.teammate_requested_help = (
                help_status is not None and help_status.get('needs_help', False)
            )
            teammate_is_safe_refuge = not (help_status is not None and help_status.get('is_fighting', False))
        else:
            self.blackboard.dist_to_nearest_teammate = 100.0
            self.blackboard.teammate_requested_help = False

        # 3. hide_x/y: run to my nearest teammate, provided they're not
        # currently fighting (a teammate under fire is not a safe refuge) -
        # else retreat to the most recent point on my own trail where no
        # enemy was in sight (safe_waypoints, populated above). Used by
        # both HideAndWaitAction and RunFromEnemyAction's no-known-threat
        # fallback.
        if teammate_is_safe_refuge and self.blackboard.teammate_x is not None:
            self.blackboard.hide_x = self.blackboard.teammate_x
            self.blackboard.hide_y = self.blackboard.teammate_y
        elif self.safe_waypoints:
            self.blackboard.hide_x, self.blackboard.hide_y = self.safe_waypoints[-1]
        else:
            self.blackboard.hide_x, self.blackboard.hide_y = current_pos

        # am_i_in_danger is the single source of truth for "am I in
        # trouble" - read by both IsLowHealthOrOutnumbered (survival
        # branch) and _publish_team_status/sense_and_think (the
        # distress-call broadcast teammates react to), so the threshold
        # can't drift between them.
        self.blackboard.am_i_in_danger = (
            self.blackboard.health < 0.5
            or self.blackboard.num_enemies > self.blackboard.num_teammates + 1
        )

    def pose_callback(self, msg):
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.current_yaw = geometry_utils.get_yaw_from_quaternion(msg.pose.pose.orientation)

        if not self.safe_waypoints:
            self.safe_waypoints.append((self.blackboard.current_x, self.blackboard.current_y))

    def map_callback(self, msg):
        # slam_toolbox's /map lives in its own run-relative TF frame (it
        # re-zeros at wherever the robot started this run). current_x/
        # current_y come from a separate, supposedly-stable pose source
        # (/odometry/global). Without reconciling the two, obstacles read
        # straight off the grid would land in the wrong place relative to
        # where tactical_brain thinks it is - exactly the mismatch that
        # let A* plan straight through real walls. See localization_bridge.py.
        try:
            obstacles = self.localization_bridge.transform_occupancy_grid_to_world(
                msg, self.blackboard.current_x, self.blackboard.current_y, self.blackboard.current_yaw
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        # /map only ever tells us about walls/terrain - dead robots are a
        # separate, non-SLAM source of obstacles, merged in fresh each time
        # rather than kept in a second persistent set, since this
        # assignment would otherwise wholesale-overwrite them (see
        # team_comms.py's dead_robot_locations docstring).
        dead_cells = {
            (round(dx_m / A_planner.XY_RESOLUTION), round(dy_m / A_planner.XY_RESOLUTION))
            for dx_m, dy_m in self.team_comms.get_dead_robot_locations()
        }
        self.static_obstacles = obstacles | dead_cells
        self.get_logger().info(
            f"static_obstacles updated from /map: {len(obstacles)} occupied cells "
            f"(+{len(dead_cells)} dead-robot cells)"
        )

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def main(args=None):
    rclpy.init(args=args)
    node = TacticalBrainNode()
    # MultiThreadedExecutor + pose_callback's own callback group (see
    # __init__) so pose updates keep flowing while the tree timer's A*
    # search or map_callback's grid scan are running, instead of being
    # serialized behind them on a single thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()