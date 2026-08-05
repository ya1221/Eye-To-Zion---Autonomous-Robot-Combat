"""Feeds a movement leaf's Hybrid A* waypoints into nav2's FollowPath
action: replan cadence, per-direction segment splitting, path
densification, and stuck-recovery.

Composed into a behavior-tree leaf (one PathExecutor instance per leaf,
each owning its own segment-cache/cooldown state) rather than an
inheritance base class - so tactical decision logic (which leaf gets
picked, what goal it wants) and path-execution machinery (how a goal
actually gets driven) can change independently. A leaf supplies a
path_provider callable - the same shape as the old get_tactical_path()
override: () -> [(x, y, yaw, direction), ...] or None - so it still owns
picking its own goal and calling A_planner itself; PathExecutor only
owns turning whatever comes back into FollowPath traffic.
"""
import math

from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import py_trees

from tactical_brain import A_planner


class PathExecutor:
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

    def __init__(self, ros_node, name):
        self.ros_node = ros_node
        self.name = name
        # Needed for _build_recovery_segment and the failure-position
        # check below. A dedicated client (rather than reaching into a
        # leaf's own blackboard client) keeps PathExecutor usable without
        # depending on which leaf composed it in.
        self.blackboard = py_trees.blackboard.Client(name=f"{name}_path_executor")
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)

        self._action_client = ActionClient(ros_node, FollowPath, 'follow_path')
        self._is_driving = False
        self._cached_segments = None
        self._segment_index = 0
        self._last_plan_time = None
        self._consecutive_plan_failures = 0
        self._last_plan_failure_pos = None
        self._is_recovery_segment = False

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

        self.ros_node.get_logger().warn(
            f"[{self.name}] A* failed {self.CONSECUTIVE_PLAN_FAILURE_THRESHOLD}x in a row from "
            f"({cx:.2f}, {cy:.2f}) - assuming stuck, moving {step_used}m to "
            f"({tx:.2f}, {ty:.2f}) (direction={direction}) before retrying planning."
        )
        return [(direction, [(cx, cy, cyaw), (tx, ty, cyaw)])]

    def tick(self, path_provider):
        """Call once per BT tick with the leaf's own path_provider (its old
        get_tactical_path()). Returns the py_trees Status the leaf's
        update() should return."""
        if self._is_driving:
            return py_trees.common.Status.RUNNING

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
            waypoints = path_provider()
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
                # from here for several ticks in a row. A* can't route out
                # of that by definition, so don't call it again - back up a
                # short fixed distance instead and let normal planning
                # retry from wherever that leaves us.
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

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.ros_node.get_clock().now().to_msg()

        try:
            for x, y, yaw in self._densify_segment(segment):
                mx, my, myaw = self.ros_node.localization_bridge.world_to_map(
                    x, y, yaw,
                    self.blackboard.current_x, self.blackboard.current_y, self.blackboard.current_yaw,
                )
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(mx)
                pose.pose.position.y = float(my)
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

        self.ros_node.get_logger().info(
            f"[{self.name}] FollowPath segment concluded (status={status}) - re-evaluating next tick."
        )

    def terminate(self):
        """Call from the leaf's terminate(INVALID) - a different branch
        took over, this plan is stale, don't resume it blindly once this
        leaf is re-selected later."""
        if self._is_driving:
            self.ros_node.get_logger().info(f"[{self.name}] Interrupted! Canceling FollowPath.")
        self._is_driving = False
        self._cached_segments = None
        self._segment_index = 0
        self._last_plan_time = None
        self._is_recovery_segment = False
        # _consecutive_plan_failures/_last_plan_failure_pos deliberately
        # NOT reset here - terminate(INVALID) fires on every re-tick
        # following ANY non-RUNNING status, not just on a genuine switch to
        # a different branch, so it can't be used to distinguish those
        # cases (see the leaf's own terminate()).