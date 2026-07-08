import json
import math
import os
import threading
import time

from tactical_brain import world_model
from tactical_brain import A_planner
from tactical_brain import ballistics_helper
from tactical_brain import aim_controller

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import py_trees
import std_srvs.srv

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

#######################
# father action class
class Nav2BaseAction(py_trees.behaviour.Behaviour):
    # Replan from scratch at most this often. Re-sending the same cached
    # path on retry (instead of a fresh A* search from wherever the robot
    # has drifted to) gives MPPI one stable reference to actually commit
    # to, instead of restarting against a slightly-different path every
    # time the previous attempt concludes/aborts. Was 15.0 (too slow to
    # react to walls slam_toolbox only just discovered), then 3.0 (too
    # fast - cut off A*'s wide forward turn-around arcs before they could
    # complete, since reversing costs more than arcing forward per
    # A_planner.py's direction_cost/switch_gear_cost, so it kept
    # restarting the same early portion of a multi-second turn rather
    # than ever finishing it). 8.0 splits the difference.
    REPLAN_COOLDOWN_SEC = 8.0

    # A* failing this many ticks in a row (from roughly the same spot) is
    # treated as genuinely stuck rather than a one-off blip - see
    # _build_recovery_segment. A* is deterministic, so repeated failures
    # here mean check_collision rejects every candidate move from the
    # current position, not transient noise.
    CONSECUTIVE_PLAN_FAILURE_THRESHOLD = 3
    # Candidate step distances used to get unstuck (see
    # _build_recovery_segment, which probes several directions at each
    # distance in turn and validates each via check_collision), smallest
    # first. A single fixed distance isn't enough: a direction can fail
    # either because the step doesn't travel far enough to clear a
    # boundary/obstacle (needs MORE distance) or because its endpoint
    # lands too close to a real obstacle (needs LESS, or a different
    # direction) - confirmed directly that both failure modes occur in
    # practice, so no single value covers both. Escalating distances
    # (rather than one), tried smallest-first, prefers the least movement
    # that actually works. Makes no "arena center"/maze-size assumption,
    # so it behaves the same in sim or on a real robot.
    RECOVERY_STEP_DISTANCES = (0.3, 0.5, 0.8, 1.2)

    def __init__(self, name, ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        # Needed by _world_to_map_pose below, registered here so it's
        # available to every subclass regardless of which other keys they
        # individually register.
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        if self.ros_node:
            # אנחנו עכשיו מדברים ישירות עם הבקר המקומי (Controller) של Nav2
            self._action_client = ActionClient(self.ros_node, FollowPath, 'follow_path')
        self._is_driving = False
        self._cached_segments = None
        self._segment_index = 0
        self._last_plan_time = None
        self._consecutive_plan_failures = 0
        self._last_plan_failure_pos = None
        self._is_recovery_segment = False

    def get_tactical_path(self):
        """
        חייב להחזיר רשימה של נקודות: [(x, y, yaw, direction), ...]
        direction is 1 (forward) or -1 (reverse), as produced by
        A_planner.py's calc_hybrid_a_star.
        """
        raise NotImplementedError("Child classes must implement get_tactical_path!")

    @staticmethod
    def _split_into_segments(waypoints):
        # nav_msgs/Path has no per-pose direction field, and a single
        # FollowPath goal mixing forward and reverse waypoints left MPPI
        # with no consistent direction to commit to - it would get stuck
        # fighting itself (confirmed directly: logged path with 5 of 6
        # steps reverse matching the robot's stuck/overshoot behavior).
        # Splitting into same-direction runs gives each FollowPath goal
        # an unambiguous direction to execute.
        segments = []
        current_segment = []
        current_direction = None
        for x, y, yaw, direction in waypoints:
            if direction != current_direction and current_segment:
                segments.append((current_direction, current_segment))
                current_segment = []
            current_segment.append((x, y, yaw))
            current_direction = direction
        if current_segment:
            segments.append((current_direction, current_segment))
        return segments

    @staticmethod
    def _densify_segment(points, max_spacing=0.05):
        # A_planner's MOVE_STEP is 0.2m, so a short segment can reach
        # FollowPath as just 3-4 widely-spaced poses. Measured directly
        # (ground-truth trajectory logging) that MPPI does not slow down
        # approaching such a sparse path's last pose at all - it sails
        # through and overshoots by over a meter before it does anything
        # resembling stopping. Interpolating extra poses in between (without
        # touching A*'s own search step, which would blow up Hybrid A*'s
        # search cost) gives the controller a denser breadcrumb trail to
        # actually track near the goal.
        if len(points) < 2:
            return points
        dense = [points[0]]
        for (x0, y0, yaw0), (x1, y1, yaw1) in zip(points[:-1], points[1:]):
            dist = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(math.ceil(dist / max_spacing)))
            dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
            for i in range(1, steps + 1):
                t = i / steps
                dense.append((
                    x0 + (x1 - x0) * t,
                    y0 + (y1 - y0) * t,
                    yaw0 + dyaw * t,
                ))
        return dense

    def _build_recovery_segment(self):
        # Bypasses calc_hybrid_a_star's search entirely (that's the whole
        # point: A* has just failed CONSECUTIVE_PLAN_FAILURE_THRESHOLD
        # times in a row from this same spot, so it can't be trusted to
        # find a way out of it) - but the escape *direction* still has to
        # be validated somehow, rather than assumed.
        #
        # First version of this just backed straight up along the reverse
        # of the current heading. Confirmed directly (live sim test) that
        # this only ever moves along the heading axis - it does nothing
        # for a trap that's perpendicular to however the robot happens to
        # be facing (e.g. escaped past the plannable region's y bound
        # while facing along x: every "successful" reverse-along-heading
        # nudge kept y pinned at the same out-of-bounds value, cycling
        # forever without ever getting back in). So instead, probe a
        # spread of world-frame directions and validate each with
        # A_planner's own check_collision - the exact same rule A* itself
        # uses, so this makes no assumption about heading, arena size, or
        # "center", and is exactly as valid in sim or on a real robot.
        cx = self.blackboard.current_x
        cy = self.blackboard.current_y
        cyaw = self.blackboard.current_yaw

        xy_res = A_planner.XY_RESOLUTION
        spatial_index = A_planner.build_spatial_index(self.ros_node.static_obstacles)
        tx, ty, step_used = None, None, None
        for step in self.RECOVERY_STEP_DISTANCES:
            for i in range(8):
                heading = i * (2 * math.pi / 8)
                candidate_x = cx + step * math.cos(heading)
                candidate_y = cy + step * math.sin(heading)
                candidate_node = A_planner.Node(
                    int(round(candidate_x / xy_res)), int(round(candidate_y / xy_res)), 0, 1, None, 0
                )
                if A_planner.check_collision(candidate_node, self.ros_node.static_obstacles, xy_res, spatial_index):
                    tx, ty, step_used = candidate_x, candidate_y, step
                    break
            if tx is not None:
                break

        if tx is None:
            # Pathological case (surrounded on all sides at every tried
            # distance) - fall back to the original plain
            # reverse-along-heading at the largest tried distance as a
            # last resort, better than sending nothing.
            step_used = self.RECOVERY_STEP_DISTANCES[-1]
            tx = cx - step_used * math.cos(cyaw)
            ty = cy - step_used * math.sin(cyaw)

        # Pick whichever gear (forward/reverse) is the closer kinematic
        # match to the chosen target, so MPPI isn't fighting
        # PreferForwardCritic/PreferReverseCritic unnecessarily - it still
        # steers as needed to actually get there either way.
        bearing_to_target = math.atan2(ty - cy, tx - cx)
        heading_delta = math.atan2(
            math.sin(bearing_to_target - cyaw), math.cos(bearing_to_target - cyaw)
        )
        direction = 1 if abs(heading_delta) <= math.pi / 2 else -1

        if self.ros_node:
            self.ros_node.get_logger().warn(
                f"[{self.name}] A* failed {self.CONSECUTIVE_PLAN_FAILURE_THRESHOLD}x in a row from "
                f"({cx:.2f}, {cy:.2f}) - assuming stuck, moving {step_used}m to "
                f"({tx:.2f}, {ty:.2f}) (direction={direction}) before retrying planning."
            )
        return [(direction, [(cx, cy, cyaw), (tx, ty, cyaw)])]

    def _world_to_map_pose(self, wx, wy, wyaw):
        # ROOT CAUSE of this whole investigation's overshoot (confirmed
        # directly, 2026-06-28): A_planner computes waypoints in the same
        # ground-truth/ "world" frame as current_x/current_y (itay_amcl_topic),
        # but every FollowPath path was being declared frame_id='map' while
        # still carrying those raw world-frame numbers. slam_toolbox's real
        # 'map' frame origin is the robot's SPAWN point, not the world
        # origin - verified live: map->base_link reads ~(0,0) at spawn while
        # ground truth reads the real spawn coordinates (~1.78, 1.44). So
        # every goal was silently getting the entire spawn-point offset
        # added on top of where it was actually meant to be - nav2 was
        # correctly, faithfully driving to the (wrongly-labeled) coordinates
        # the whole time. This is the exact inverse of map_callback's
        # map->world obstacle transform, reusing the same live TF lookup +
        # current_x/y/yaw correspondence.
        tf = self.ros_node.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        map_x = tf.transform.translation.x
        map_y = tf.transform.translation.y
        map_yaw = self.ros_node.get_yaw_from_quaternion(tf.transform.rotation)

        yaw_offset = self.blackboard.current_yaw - map_yaw
        cos_o = math.cos(yaw_offset)
        sin_o = math.sin(yaw_offset)

        dx_w = wx - self.blackboard.current_x
        dy_w = wy - self.blackboard.current_y
        dx_m = dx_w * cos_o + dy_w * sin_o
        dy_m = -dx_w * sin_o + dy_w * cos_o

        return (map_x + dx_m, map_y + dy_m, wyaw - yaw_offset)

    def update(self):
        if self.ros_node is None: return py_trees.common.Status.SUCCESS
        if self._is_driving: return py_trees.common.Status.RUNNING

        # nav2's FollowPath action server isn't constructed until
        # controller_server's lifecycle activates (on_activate()) - before
        # that, there's nothing listening, so send_goal_async()'s future
        # would never resolve and we'd never know to retry. Check
        # non-blocking readiness instead of sending into the void.
        if not self._action_client.server_is_ready():
            self.ros_node.get_logger().warn(
                f"[{self.name}] Nav2 controller_server not active yet - waiting to send path."
            )
            return py_trees.common.Status.RUNNING

        now = self.ros_node.get_clock().now()
        seconds_since_plan = (
            (now - self._last_plan_time).nanoseconds / 1e9
            if self._last_plan_time is not None else None
        )
        needs_fresh_plan = (
            self._cached_segments is None
            or seconds_since_plan is None
            or seconds_since_plan < 0  # sim clock jumped backward (sim restarted) - cached plan is stale
            or seconds_since_plan >= self.REPLAN_COOLDOWN_SEC
        )

        if needs_fresh_plan:
            waypoints = self.get_tactical_path()
            if not waypoints:
                # Count failures from roughly the same spot only - can't
                # rely on py_trees' terminate(INVALID) to reset this
                # between genuinely different contexts, since reactive
                # (memory=False) Sequence/Selector composites call
                # child.stop(INVALID) before every re-tick following ANY
                # non-RUNNING status, including a plain FAILURE from this
                # same node a moment ago (confirmed directly - a
                # terminate()-based counter reset to 0 every single tick,
                # never accumulating). Comparing position instead sidesteps
                # that entirely and is arguably more correct anyway: only
                # count it as "still stuck" if the robot hasn't moved.
                cx, cy = self.blackboard.current_x, self.blackboard.current_y
                if self._last_plan_failure_pos is not None and math.hypot(
                    cx - self._last_plan_failure_pos[0], cy - self._last_plan_failure_pos[1]
                ) < 0.05:
                    self._consecutive_plan_failures += 1
                else:
                    self._consecutive_plan_failures = 1
                self._last_plan_failure_pos = (cx, cy)

                if self._consecutive_plan_failures < self.CONSECUTIVE_PLAN_FAILURE_THRESHOLD:
                    return py_trees.common.Status.FAILURE
                # Stuck: check_collision has rejected every candidate move
                # from here for several ticks in a row (see
                # etz-open-issues - arena-boundary escape and the
                # PLANNING_CLEARANCE dead zone share this exact
                # mechanism). A* can't route out of that by definition, so
                # don't call it again - back up a short fixed distance
                # instead and let normal planning retry from wherever that
                # leaves us.
                self._consecutive_plan_failures = 0
                self._last_plan_failure_pos = None
                self._cached_segments = self._build_recovery_segment()
                self._segment_index = 0
                self._last_plan_time = now
                self._is_recovery_segment = True
            else:
                self._consecutive_plan_failures = 0
                self._last_plan_failure_pos = None
                self._cached_segments = self._split_into_segments(waypoints)
                self._segment_index = 0
                self._last_plan_time = now
                self._is_recovery_segment = False
        elif self._segment_index >= len(self._cached_segments):
            # Finished every segment from the last plan but cooldown
            # hasn't elapsed yet - force a fresh plan next tick instead
            # of looping with nothing left to send.
            self._cached_segments = None
            return py_trees.common.Status.RUNNING
        else:
            self.ros_node.get_logger().info(
                f"[{self.name}] Re-sending cached segment (planned {seconds_since_plan:.1f}s "
                "ago) instead of replanning from scratch."
            )

        direction, segment = self._cached_segments[self._segment_index]

        # בניית הודעת מסלול (Path) תקנית של ROS2
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.ros_node.get_clock().now().to_msg()

        try:
            for x, y, yaw in self._densify_segment(segment):
                mx, my, myaw = self._world_to_map_pose(x, y, yaw)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(mx)
                pose.pose.position.y = float(my)
                # המרת הזווית (Yaw) לקווטרניון ש-ROS2 דורש
                pose.pose.orientation.z = float(math.sin(myaw / 2.0))
                pose.pose.orientation.w = float(math.cos(myaw / 2.0))
                path_msg.poses.append(pose)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.ros_node.get_logger().warn(
                f"[{self.name}] map->base_footprint not available yet ({e}) - "
                "retrying next tick instead of sending a world-frame path "
                "mislabeled as map-frame."
            )
            return py_trees.common.Status.RUNNING

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        # PreferForwardCritic fights the reversing motion a reverse
        # segment needs to finish (it activates near any segment's end,
        # regardless of direction) - FollowPathReverse is identical to
        # FollowPath but without it, confirmed via /cmd_vel_nav that
        # reverse segments stalled 44s/85s under plain FollowPath.
        goal_msg.controller_id = 'FollowPath' if direction == 1 else 'FollowPathReverse'
        # Tight 5cm+stopped checker only for a plan's truly final segment -
        # intermediate segments end at arbitrary A* waypoints, not
        # somewhere the robot is meant to actually stop, and requiring
        # 5cm+stopped there too turned ordinary transitions into 50+
        # second stalls (confirmed directly).
        is_final_segment = self._segment_index == len(self._cached_segments) - 1
        # A recovery nudge is a single "final" segment by construction, but
        # it isn't a real goal to stop precisely at - it just needs to get
        # far enough to let A* try again, so it always uses the loose
        # checker regardless of is_final_segment.
        goal_msg.goal_checker_id = (
            'loose_goal_checker' if (self._is_recovery_segment or not is_final_segment) else 'goal_checker'
        )

        self.ros_node.get_logger().info(
            f"[{self.name}] Sending segment {self._segment_index + 1}/{len(self._cached_segments)} "
            f"({len(segment)} points, direction={direction}, controller={goal_msg.controller_id}, "
            f"goal_checker={goal_msg.goal_checker_id}) to Nav2"
        )
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self._is_driving = True
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, future):
        # controller_server's action server can exist (discoverable) before
        # nav2's lifecycle bringup actually activates it - goals sent during
        # that window get rejected. Reset _is_driving so the next tree tick
        # retries instead of believing forever that it's already driving.
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            if self.ros_node:
                self.ros_node.get_logger().warn(
                    f"[{self.name}] FollowPath goal rejected (nav2 controller_server "
                    "may not be active yet) - will retry next tick."
                )
            self._is_driving = False
            return

        # Goal accepted - still need to know when it actually concludes
        # (success, abort, e.g. nav2's "Failed to make progress"), otherwise
        # _is_driving stays True forever and the tree never retries.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._is_driving = False
        try:
            status = future.result().status
        except Exception:
            status = None

        # STATUS_SUCCEEDED == 4 (action_msgs/msg/GoalStatus). Only advance
        # to the next segment on a real success - on abort/cancel, retry
        # the same segment (still bounded by REPLAN_COOLDOWN_SEC above
        # before giving up and replanning from scratch).
        if status == 4:
            self._segment_index += 1

        if self.ros_node:
            self.ros_node.get_logger().info(
                f"[{self.name}] FollowPath segment concluded (status={status}) - re-evaluating next tick."
            )

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self._is_driving and self.ros_node:
                self.ros_node.get_logger().info(f"[{self.name}] Interrupted! Canceling FollowPath.")
            self._is_driving = False
            # A different branch took over - this plan is stale, don't
            # resume it blindly once we're re-selected later.
            self._cached_segments = None
            self._segment_index = 0
            self._last_plan_time = None
            self._is_recovery_segment = False
            # _consecutive_plan_failures/_last_plan_failure_pos deliberately
            # NOT reset here - terminate(INVALID) fires on every re-tick
            # following ANY non-RUNNING status (see update()'s comment),
            # not just on a genuine switch to a different branch, so it
            # can't be used to distinguish those cases.

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
    MapsToGoalAction (a Nav2BaseAction) simply isn't ticked, and
    Nav2BaseAction.terminate() already cancels any in-flight FollowPath
    on invalidation, so Nav2 stops writing to cmd_vel on its own.

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

        # Ballistics/trigger decision - see ballistics_helper.py. The
        # shared TriggerController/publishers live on ros_node (not on
        # this behaviour) because sense_and_think's centralized safety net
        # needs to reset/publish-False the same instance on ticks where
        # this action never runs at all (see that method for why - a
        # single-tick, always-SUCCESS leaf like this one can't rely on
        # terminate()/initialise() to detect "I was skipped this tick").
     
        self.ros_node.command_shooting(self.blackboard.current_heading_error)
        return py_trees.common.Status.SUCCESS
    
#######################
# survival branch
class IsLowHealthOrOutnumbered(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_low_health_or_outnumbered"):
        super(IsLowHealthOrOutnumbered, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="health", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_enemies", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_teammates", access=py_trees.common.Access.READ)

    def update(self):
        health = self.blackboard.health
        num_enemies = self.blackboard.num_enemies
        num_teammates = self.blackboard.num_teammates

        if (health is not None and health < 0.5) or (num_enemies is not None and num_teammates is not None and num_enemies > num_teammates + 1):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AreTeammatesComing(py_trees.behaviour.Behaviour):
    def __init__(self, name="are_teammates_coming"):
        super(AreTeammatesComing, self).__init__(name)
    def update(self):
        return py_trees.common.Status.FAILURE # נניח שכרגע אף אחד לא בא

class HideAndWaitAction(Nav2BaseAction):
    def __init__(self, name="hide_and_wait_action", ros_node=None):
        super().__init__(name, ros_node)
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

class RunFromEnemyAction(Nav2BaseAction):
    def __init__(self, name="run_from_enemy_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        self.blackboard.robot_command = "RUNNING_FROM_ENEMY"
        # מחזיר מסלול עם נקודה אחת כדי לא לקרוס (עד שנחבר את A*)
        return [(0.0, 0.0, 0.0, 1)]
    
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

class DriveToTeammateAction(Nav2BaseAction):
    def __init__(self, name="drive_to_teammate_action", ros_node=None):
        super().__init__(name, ros_node)
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
#########################
# patrol branch
class MapsToGoalAction(Nav2BaseAction): 
    def __init__(self, name="maps_to_goal_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        gx, gy = self.blackboard.goal_x, self.blackboard.goal_y
        
        if gx is not None and gy is not None and self.ros_node is not None:
            self.blackboard.robot_command = "NAVIGATING (A* PATH)"
            
            start_pos = (cx, cy, cyaw)
            goal_pos = (gx, gy, math.atan2(gy - cy, gx - cx))

            self.ros_node.get_logger().info(f"[{self.name}] Running Hybrid A* planner...")
            
            # קריאה לאלגוריתם שלכם
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

            # אריזת הנתונים לפורמט [(x, y, yaw, direction), ...] עבור Nav2BaseAction
            tactical_waypoints = []
            for i in range(len(x_path)):
                tactical_waypoints.append((x_path[i], y_path[i], theta_path[i], directions[i]))
                
            return tactical_waypoints
            
        return None
    
# ==========================================
# פונקציית יצירת העץ המקורית שלך
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

    patrol_branch = py_trees.composites.Sequence(name="patrol_branch", memory = False)
    patrol_branch.add_children(
        [MapsToGoalAction(name = "maps to goal action", ros_node=ros_node)])

    root = py_trees.composites.Selector(name="root", memory = False)
    root.add_children([survival_branch, attack_branch, help_teammate_branch, patrol_branch])

    return py_trees.trees.BehaviourTree(root)


# ==========================================
# קוד ה-ROS2 Node (המעטפת שמריצה ובודקת את העץ)
# # ==========================================
# class TacticalBrainNode(Node):
#     def __init__(self):
#         super().__init__('tactical_brain_node')
#         self.get_logger().info("Tactical Brain is waking up and planting the tree...")
        
#         self.redis_client = redis.Redis(host='redis_server', port=6379, db=0) # שנה IP במידת הצורך
#         self.pubsub = self.redis_client.pubsub()
#         self.pubsub.subscribe('/detected_enemies', '/team/positions', '/aruco/poses') 
    
#         self.pose_sub = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',  
#             self.pose_callback,
#             10
#         )

#         # יצירת ה-Subscriber כדי לקרוא את מה שה-redis_manager משדר
#         self.aruco_pose_sub = self.create_subscription(
#             PoseStamped,
#             '/sensor_fusion_node/aruco_global_pose',
#             self.aruco_callback,
#             10
#         )
#          # משתנים לשמירת האמת האבסולוטית מהארוקו
#         self.aruco_x = 0.0
#         self.aruco_y = 0.0
#         self.aruco_yaw = 0.0

#         # משתנים לשמירת הסטייה המחושבת
#         self.drift_x = 0.0
#         self.drift_y = 0.0
#         self.drift_yaw = 0.0

#         # פבלישר לערוץ חטיפת המיקום של מערכת הניווט
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped,
#             '/initialpose',
#             10
#         )

#         # 2. אתחול משתני מצב מקומיים (מהקוד שלך)
#         self.static_obstacles = set() # (כאן תיכנס מפת הקירות מהלידאר בעתיד)
#         self.enemies_list = []
#         self.teammates_dict = {}
#         self.danger_dict = {}
#         self.teammates_aura_set = set()

#         # 1. רישום כל המשתנים שהעץ שלך צריך ב-Blackboard
#         self.blackboard = py_trees.blackboard.Client(name="global")
#         keys = [
#             "current_x", "current_y", "current_yaw", "hide_x", "hide_y", "goal_x", "goal_y", "teammate_x", "teammate_y",
#             "dist_to_closest_enemy", "has_line_of_sight_to_closest_enemy", 
#             "health", "num_enemies", "num_teammates", 
#             "teammate_requested_help", "dist_to_help_teammate", "robot_command"
#         ]
#         for key in keys:
#             self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
            
#         # מצב התחלתי שקט
#         self.reset_blackboard_to_safe_state()

#         # 2. הקמת העץ
#         self.tree = create_tree(ros_node=self)
#         self.tree.setup(timeout=15)

#         # 3. טיימרים
#         # טיימר שמפעיל את העץ כל חצי שנייה (2Hz)
#         self.tree_timer = self.create_timer(0.5, self.sense_and_think)

#         # טיימר שמשנה את התרחיש כל 5 שניות
#         #self.scenario_timer = self.create_timer(5.0, self.mock_scenario_changer)
#         #self.scenario_step = 0

#     def reset_blackboard_to_safe_state(self):
#         self.blackboard.dist_to_closest_enemy = 100.0
#         self.blackboard.has_line_of_sight_to_closest_enemy = False
#         self.blackboard.health = 1.0
#         self.blackboard.num_enemies = 0
#         self.blackboard.num_teammates = 1
#         self.blackboard.teammate_requested_help = False
#         self.blackboard.dist_to_help_teammate = 100.0
#         self.blackboard.robot_command = "WAIT"

#         self.blackboard.current_x = 2.0
#         self.blackboard.current_y = 2.0
#         self.blackboard.current_yaw = 0.0
#         self.blackboard.hide_x = 2.25
#         self.blackboard.hide_y = 4.75
#         self.blackboard.goal_x = 3.5
#         self.blackboard.goal_y = 4.0
#         self.blackboard.teammate_x = 1.5
#         self.blackboard.teammate_y = 1.0
        
#     def sense_and_think(self):
#         """
#         זוהי לולאת הליבה של הרובוט: Sense -> Think -> Act
#         """
#         # שלב א': Sense (קריאה מהרדיס באמצעות הפונקציה שלך)
#         self.danger_dict, self.teammates_aura_set, self.enemies_list, self.teammates_dict = redis_manager.get_latest_world_state(
#             self.pubsub, self.enemies_list, self.teammates_dict, self.static_obstacles
#         )
        
#         # שלב ב': Translation (תרגום המילונים למשתנים פשוטים שעץ ההתנהגות מבין)
#         self.update_blackboard_from_redis_state()
        
#         # שלב ג': Think (הפעלת העץ לקבלת החלטה)
#         self.tree.tick()
#         self.get_logger().info(f"Tree Output Command ---> {self.blackboard.robot_command}")
    

#     def update_blackboard_from_redis_state(self):
#         # 1. עדכון נתוני אויבים
#         self.blackboard.num_enemies = len(self.enemies_list)
        
#         if self.enemies_list:
#             current_pos = (self.blackboard.current_x, self.blackboard.current_y)
            
#             # 1. פיצול האויבים לשתי קבוצות: גלויים ומוסתרים
#             visible_enemies = []
#             hidden_enemies = []
            
#             for enemy in self.enemies_list:
#                 enemy_pos = (enemy['x'], enemy['y'])
#                 if redis_manager.line_of_sight_clear(current_pos, enemy_pos, self.static_obstacles):
#                     visible_enemies.append(enemy)
#                 else:
#                     hidden_enemies.append(enemy)
            
#             # 2. בחירת האיום העיקרי לפי עדיפות טקטית
#             if visible_enemies:
#                 # יש אויבים גלויים! ניקח את הקרוב מביניהם
#                 closest_enemy = min(visible_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
#                 self.blackboard.has_line_of_sight_to_closest_enemy = True
#             else:
#                 # כולם מוסתרים. ניקח את המוסתר הקרוב ביותר (למשל כדי להתכונן להסתערות שלו)
#                 closest_enemy = min(hidden_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
#                 self.blackboard.has_line_of_sight_to_closest_enemy = False
            
#             # 3. עדכון המרחק בלוח
#             self.blackboard.dist_to_closest_enemy = distance(current_pos, (closest_enemy['x'], closest_enemy['y']))
#         else:
#             # אין אויבים באזור
#             self.blackboard.dist_to_closest_enemy = 100.0
#             self.blackboard.has_line_of_sight_to_closest_enemy = False

#         # 2. עדכון נתוני חברי צוות
#         self.blackboard.num_teammates = len(self.teammates_dict)
#         if self.teammates_dict:
#             # ניקח כרגע את החבר הראשון במילון לשם הדוגמה
#             first_teammate_id = list(self.teammates_dict.keys())[0]
#             teammate_data = self.teammates_dict[first_teammate_id]
            
#             self.blackboard.teammate_x = teammate_data.get('x')
#             self.blackboard.teammate_y = teammate_data.get('y')
            
#             # אם הוא שידר דגל מצוקה ברדיס
#             self.blackboard.teammate_requested_help = teammate_data.get('needs_help', False)

    
#     def aruco_callback(self, msg):
#         # קליטת קואורדינטות האמת
#         self.aruco_x = msg.pose.position.x
#         self.aruco_y = msg.pose.position.y
#         self.aruco_yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
        
#         # חישוב הסטייה מול ה-Blackboard (AMCL)
#         self.drift_x = self.aruco_x - self.blackboard.current_x
#         self.drift_y = self.aruco_y - self.blackboard.current_y
#         yaw_diff = self.aruco_yaw - self.blackboard.current_yaw
#         self.drift_yaw = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        
#         # חישוב המרחק הכולל (היפוטנוזה של משולש X,Y)
#         total_drift_meters = math.hypot(self.drift_x, self.drift_y)
        
#         # אם הסטייה עולה על 5 ס"מ (0.05 מטר), אנחנו מבצעים איפוס קשיח!
#         if total_drift_meters > 0.05:
#             self.get_logger().warn(f"HIGH ODOM DRIFT: {total_drift_meters*100:.1f}cm! Resetting AMCL to ArUco ground truth.")
            
#             # יצירת הודעת האיפוס
#             reset_msg = PoseWithCovarianceStamped()
#             reset_msg.header.stamp = self.get_clock().now().to_msg()
#             reset_msg.header.frame_id = 'map'
            
#             # הזנת קואורדינטות הארוקו לתוך ה-AMCL
#             reset_msg.pose.pose.position.x = self.aruco_x
#             reset_msg.pose.pose.position.y = self.aruco_y
#             reset_msg.pose.pose.orientation = msg.pose.orientation
            
#             # (אופציונלי אך מומלץ): איפוס מטריצת השגיאות (Covariance) לזיהוי מדויק
#             reset_msg.pose.covariance[0] = 0.05 # רדיוס ביטחון סביר
#             reset_msg.pose.covariance[7] = 0.05
#             reset_msg.pose.covariance[35] = 0.05
            
#             # שיגור ההוראה למערכת הניווט!
#             self.initial_pose_pub.publish(reset_msg)
            
#             # איפוס המשתנים המקומיים כדי שלא נשגר איפוס כפול בפריים הבא
#             self.drift_x = 0.0
#             self.drift_y = 0.0

#     def pose_callback(self, msg):
#         # שימוש בפונקציית העזר כדי שהקוד יהיה נקי
#         self.blackboard.current_x = msg.pose.pose.position.x
#         self.blackboard.current_y = msg.pose.pose.position.y
#         self.blackboard.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

#     def get_yaw_from_quaternion(self, q):
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     # def mock_scenario_changer(self):
#     #     self.scenario_step += 1
#     #     print("\n" + "="*50)
        
#     #     if self.scenario_step == 1:
#     #         self.get_logger().info("SCENARIO 1: All clear. Expecting to Patrol.")
#     #         self.reset_blackboard_to_safe_state()
            
#     #     elif self.scenario_step == 2:
#     #         self.get_logger().info("SCENARIO 2: Enemy 2m away in line of sight! Expecting Combat.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.dist_to_closest_enemy = 2.0
#     #         self.blackboard.has_line_of_sight_to_closest_enemy = True
            
#     #     elif self.scenario_step == 3:
#     #         self.get_logger().info("SCENARIO 3: Teammate in danger 3m away! Expecting to Help.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.teammate_requested_help = True
#     #         self.blackboard.dist_to_help_teammate = 3.0
            
#     #     elif self.scenario_step == 4:
#     #         self.get_logger().info("SCENARIO 4: Ambushed! 3 enemies, low health. Expecting to Run/Hide.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.health = 0.3
#     #         self.blackboard.num_enemies = 3
#     #         self.blackboard.num_teammates = 1
            
#     #     else:
#     #         self.get_logger().info("Restarting scenarios...")
#     #         self.scenario_step = 0

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
        self.grid_to_meters = (
            self.get_parameter('arena_size_meters').get_parameter_value().double_value
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

        # Sharing what I personally see with the rest of the team, and
        # receiving what they see - both directions on the same team-scoped
        # topic, mirrored across Zenoh exactly like teams/team_X/positions.
        # enemies_by_detector is keyed by whichever robot_id reported each
        # sighting, so my own detection and a teammate's don't clobber each
        # other if both are currently tracking (possibly different) enemies.
        #
        # Guards enemies_by_detector against the executor's two threads:
        # the team/local-detection callbacks (pose_callback_group) mutate it
        # IN PLACE while sense_and_think (the tree timer, default group)
        # iterates it in prune_stale_enemies - a concurrent insert during
        # that iteration raises "dictionary changed size during iteration"
        # and kills the node. MultiThreadedExecutor + separate callback
        # groups (needed so A* doesn't starve pose updates) is exactly what
        # makes these run in parallel, so a lock is required. teammates_dict
        # is only ever REASSIGNED wholesale (never mutated in place), so it
        # doesn't need this - update_blackboard just captures one local
        # reference to avoid a read-after-reassign KeyError.
        self._state_lock = threading.Lock()
        self.enemies_by_detector = {}
        self.team_enemy_pub = self.create_publisher(
            String,
            f'teams/team_{self.my_team_idx}/detected_enemies',
            10
        )
        self.team_enemy_sub = self.create_subscription(
            String,
            f'teams/team_{self.my_team_idx}/detected_enemies',
            self.team_enemy_callback,
            10,
            callback_group=self.pose_callback_group
        )

        # Previously also republished this same ArUco pose on
        # /odometry/filtered for the low-level PID controller
        # (_publish_self_pose, now removed) - hardware's pid_controller.cpp
        # already subscribes directly to /odometry/global (ekf_global's
        # output, see pose_sub above), so that republish had no consumer.

        # משתנים לשמירת האמת האבסולוטית מהארוקו
        self.aruco_x = 0.0
        self.aruco_y = 0.0
        self.aruco_yaw = 0.0

        # משתנים לשמירת הסטייה המחושבת
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0

        # Large-drift snap-correction: force-reseed ekf_global's internal
        # state via its real /set_pose service (robot_localization) when
        # ArUco vs. current_x/y disagree by more than 5cm - kept as an
        # explicit safety net on top of ekf_global's continuous fusion,
        # which alone may not catch up fast enough after a sudden large
        # drift (e.g. wheel slip). Was publishing to /initialpose, which
        # has no subscriber in this stack (no AMCL; slam_toolbox and
        # robot_localization don't reset from that topic) - redirected to
        # the service ekf_global actually exposes for this.
        self.set_pose_client = self.create_client(SetPose, '/ekf_global/set_pose')

        # Attack branch shooting outputs (see ballistics_helper.py +
        # AttackAction). One shared TriggerController instance so both
        # AttackAction's per-tick evaluate() and sense_and_think's
        # centralized safety-reset (below) agree on the same debounce/
        # hysteresis state.
        self.trigger_controller = ballistics_helper.TriggerController()
        # Correct topic names matching shooting_node's subscriptions
        self.firing_pub = self.create_publisher(Bool, '/firing', 10)
        # ── Shooting control (matches shooting_node's interface) ──
        self.shooting_mode_pub = self.create_publisher(String, '/shooting_mode', 10)
        self.fire_once_client = self.create_client(
            std_srvs.srv.Trigger, '/shooting_node/fire_once')

        self.declare_parameter('shooting_mode', 'single')   # 'single' or 'auto'
        self.declare_parameter('auto_fire_rate_hz', 2.0)
        self.declare_parameter('max_fire_angle_deg', 30.0)

        # AlignToEnemyAction's output (see aim_controller.py). Same topic
        # Nav2's controller normally publishes to - safe because
        # attack_branch preempts patrol_branch (see AlignToEnemyAction's
        # docstring), so only one of the two is ever actively driving.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ArUco Odometry continuous publisher (for EKF fusion)
        self.aruco_odom_pub = self.create_publisher(Odometry, '/aruco/odom', 10)

        # 2. אתחול משתני מצב מקומיים (enemies_by_detector כבר אותחל למעלה,
        # ליד team_enemy_pub/team_enemy_sub; enemies_list נגזר ממנו כל טיק)
        self.static_obstacles = set()
        self.enemies_list = []
        self.teammates_dict = {}
        self.danger_dict = {}
        self.teammates_aura_set = set()

        # slam_toolbox's /map is in its own run-relative TF frame (re-zeros
        # at wherever the robot started this run), but current_x/current_y
        # come from /odometry/global's separate, supposedly-stable pose
        # source. Reconcile the two via the live map->base_footprint TF
        # vs. the already-tracked current_x/y/yaw, the same way a real
        # deployment would need to align slam's local map against the
        # overhead-camera's global pose.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        # 1. רישום כל המשתנים שהעץ צריך ב-Blackboard
        self.blackboard = py_trees.blackboard.Client(name="global")
        keys = [
            "current_x", "current_y", "current_yaw", "hide_x", "hide_y", "goal_x", "goal_y", "teammate_x", "teammate_y",
            "dist_to_closest_enemy", "has_line_of_sight_to_closest_enemy", "current_heading_error",
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
        self.blackboard.health = 1.0
        self.blackboard.num_enemies = 0
        self.blackboard.num_teammates = 1
        self.blackboard.teammate_requested_help = False
        self.blackboard.dist_to_help_teammate = 100.0
        self.blackboard.robot_command = "WAIT"

        self.blackboard.current_x = 2.0
        self.blackboard.current_y = 2.0
        self.blackboard.current_yaw = 0.0
        self.blackboard.hide_x = 2.25
        self.blackboard.hide_y = 4.75
        # Diagnostic direction-change test: every prior straight-east
        # test (goal ~0.8m east of spawn) converged on the same overshoot
        # magnitude (~13-14.4deg peak heading deviation) AND the same
        # final wedge position/heading (~5.3-5.4, ~2-3.1, ~70-75deg)
        # regardless of goal_checker tuning, path density, MPPI batch_size,
        # or noise regeneration - suspicious for a goal-driven controller.
        # Pointing the goal north instead (perpendicular, same ~0.8m
        # distance, clear of every wall in maze/model.sdf by >0.7m) tests
        # whether the robot still converges on that same east-side
        # location/heading regardless of where the goal actually is -
        # which would prove the attractor is goal-independent.
        # Back to the pure-straight test (zero planned curvature, yaw=0
        # throughout) so any angular.z seen on /cmd_vel_nav is unambiguous
        # MPPI bias, not legitimate path-curvature-following.
        self.blackboard.goal_x = 3.9
        self.blackboard.goal_y = 1.3
        self.blackboard.teammate_x = 1.5
        self.blackboard.teammate_y = 1.0
        
    def sense_and_think(self):
        """
        זוהי לולאת הליבה של הרובוט: Sense -> Think -> Act
        """
        # שלב א': Sense (enemies_by_detector/teammates_dict כבר מתעדכנים
        # ישירות ע"י local_enemy_position_callback/team_enemy_callback/
        # my_team_positions_callback - כאן רק מנקים זיהויים ישנים (מכל
        # הרובוטים שדיווחו) ומחשבים מהם את הרשתות הטקטיות, בעזרת
        # static_obstacles שרק אנחנו מחזיקים)
        # Lock while pruning/snapshotting: the detection callbacks insert
        # into enemies_by_detector in place on the other executor thread,
        # and iterating it here without the lock can raise "dictionary
        # changed size during iteration" (see self._state_lock in __init__).
        # enemies_list is a plain snapshot the rest of the tick reads from,
        # so nothing past this point touches the live dict.
        with self._state_lock:
            self.enemies_by_detector = world_model.prune_stale_enemies(self.enemies_by_detector)
            self.enemies_list = list(self.enemies_by_detector.values())
        self.danger_dict = world_model.get_danger_dict(self.danger_dict, self.enemies_list, self.static_obstacles)
        self.danger_dict = world_model.delete_old_danger_squares(self.danger_dict)
        self.teammates_aura_set = world_model.get_teammates_aura_set(self.teammates_dict)

        # שלב ב': Translation
        self.update_blackboard_from_world_state()

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
            self._publish_aruco_odom(aruco_x, aruco_y, aruco_yaw)

        self.teammates_dict = {
            r['id']: {'x': r['x'] * self.grid_to_meters, 'y': r['y'] * self.grid_to_meters, 'needs_help': False}
            for r in robots if r.get('id') != self.robot_id
        }

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
        # world_model.prune_stale_enemies so an old detection doesn't linger
        # forever once the enemy is no longer seen.
        now = time.time()
        with self._state_lock:
            self.enemies_by_detector[self.robot_id] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'timestamp': now,
            }
        # Don't just keep this to myself - broadcast it on the team-scoped
        # topic (mirrored across Zenoh exactly like teams/team_X/positions)
        # so every teammate's main_brain merges it into their own picture.
        self.team_enemy_pub.publish(String(data=json.dumps({
            'detector_id': self.robot_id,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'timestamp': now,
        })))

    def team_enemy_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        detector_id = data.get('detector_id')
        if detector_id is None or detector_id == self.robot_id:
            # Our own broadcast echoing back - already recorded directly
            # in local_enemy_position_callback, no need to process again.
            return

        with self._state_lock:
            self.enemies_by_detector[detector_id] = {
                'x': data['x'],
                'y': data['y'],
                'timestamp': data.get('timestamp', time.time()),
            }

    def update_blackboard_from_world_state(self):
        # 1. עדכון נתוני אויבים
        self.blackboard.num_enemies = len(self.enemies_list)
        
        if self.enemies_list:
            current_pos = (self.blackboard.current_x, self.blackboard.current_y)
            
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
        else:
            self.blackboard.dist_to_closest_enemy = 100.0
            self.blackboard.has_line_of_sight_to_closest_enemy = False

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
        # inside either of them ever gets a chance to publish
        # shooting_cmd=False, zero out /cmd_vel, or reset the trigger's
        # frame-delay/hysteresis state. This runs unconditionally every
        # tick (regardless of which BT branch ends up selected), same as
        # dist_to_closest_enemy above, which is the only reliable way to
        # catch "enemy lost" here - see ballistics_helper.py's docstring
        # and the AttackAction comment for why terminate()/initialise()
        # can't do this for a single-tick, always-SUCCESS leaf.
        attack_preconditions_met = (
            self.blackboard.dist_to_closest_enemy <= MAX_COMBAT_DISTANCE
            and self.blackboard.has_line_of_sight_to_closest_enemy
        )
        
        self.get_logger().info(
            f"Shooting Inputs Debug | dist: {self.blackboard.dist_to_closest_enemy:.2f}, "
            f"LOS: {self.blackboard.has_line_of_sight_to_closest_enemy}, "
            f"heading_error: {self.blackboard.current_heading_error}, "
            f"preconditions_met: {attack_preconditions_met}"
        )

        if not attack_preconditions_met:
            self.trigger_controller.reset()
            self.cease_fire()
            self.cmd_vel_pub.publish(Twist())

        # 2. עדכון נתוני חברי צוות
        # Capture one reference: my_team_positions_callback reassigns
        # self.teammates_dict wholesale on the other executor thread, so
        # reading it twice (keys()[0] then [first_teammate_id]) could hit
        # two different dicts and KeyError. One local binding is a
        # consistent snapshot - no lock needed since it's never mutated in
        # place, only rebound.
        teammates = self.teammates_dict
        self.blackboard.num_teammates = len(teammates)
        if teammates:
            first_teammate_id = list(teammates.keys())[0]
            teammate_data = teammates[first_teammate_id]

            self.blackboard.teammate_x = teammate_data.get('x')
            self.blackboard.teammate_y = teammate_data.get('y')
            self.blackboard.teammate_requested_help = teammate_data.get('needs_help', False)

    
    def _publish_aruco_odom(self, aruco_x, aruco_y, aruco_yaw):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = float(aruco_x)
        odom_msg.pose.pose.position.y = float(aruco_y)
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = math.sin(aruco_yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(aruco_yaw / 2.0)

        # High confidence for ArUco absolute position (0.01)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        self.aruco_odom_pub.publish(odom_msg)

    def command_shooting(self, heading_error_deg):
        """Unified shooting control — checks angle, manages mode/firing/fire_once."""
        mode = self.get_parameter('shooting_mode').value
        dist = self.blackboard.dist_to_closest_enemy

        # שימוש במנגנון ה-Debounce וה-Hysteresis במקום if פשוט ורגיש לרעשים
        should_fire = self.trigger_controller.evaluate(dist, heading_error_deg)

        self.get_logger().info(f"Shooting Control Debug | should_fire: {should_fire}, mode: {mode}")

        # Publish the current mode every tick so shooting_node stays in sync
        self.shooting_mode_pub.publish(String(data=mode))

        if not should_fire:
            self.firing_pub.publish(Bool(data=False))
            return

        # Angle within threshold — fire
        if mode == 'auto':
            self.firing_pub.publish(Bool(data=True))
        else:
            # Single mode: enable firing + call fire_once service
            self.firing_pub.publish(Bool(data=True))
            if self.fire_once_client.service_is_ready():
                self.fire_once_client.call_async(std_srvs.srv.Trigger.Request())
            else:
                self.get_logger().warn('fire_once service not ready')

    def cease_fire(self):
        """Kill all shooting outputs — used by the safety net."""
        self.firing_pub.publish(Bool(data=False))

    def pose_callback(self, msg):
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg):
        # slam_toolbox's /map lives in its own run-relative TF frame (it
        # re-zeros at wherever the robot started this run). current_x/
        # current_y come from a separate, supposedly-stable pose source
        # (/odometry/global). Without reconciling the two, obstacles read
        # straight off the grid would land in the wrong place relative to
        # where tactical_brain thinks it is - exactly the mismatch that
        # let A* plan straight through real walls.
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        map_x = tf.transform.translation.x
        map_y = tf.transform.translation.y
        map_yaw = self.get_yaw_from_quaternion(tf.transform.rotation)

        yaw_offset = self.blackboard.current_yaw - map_yaw
        cos_o = math.cos(yaw_offset)
        sin_o = math.sin(yaw_offset)
        origin_x = self.blackboard.current_x
        origin_y = self.blackboard.current_y

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        grid_origin_x = msg.info.origin.position.x
        grid_origin_y = msg.info.origin.position.y
        data = msg.data

        obstacles = set()
        for row in range(height):
            base = row * width
            cell_y = grid_origin_y + (row + 0.5) * res
            dy = cell_y - map_y
            for col in range(width):
                if data[base + col] < 50:
                    continue
                cell_x = grid_origin_x + (col + 0.5) * res
                dx = cell_x - map_x
                wx = origin_x + (dx * cos_o - dy * sin_o)
                wy = origin_y + (dx * sin_o + dy * cos_o)
                obstacles.add((round(wx / A_planner.XY_RESOLUTION), round(wy / A_planner.XY_RESOLUTION)))

        self.static_obstacles = obstacles
        self.get_logger().info(f"static_obstacles updated from /map: {len(obstacles)} occupied cells")

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