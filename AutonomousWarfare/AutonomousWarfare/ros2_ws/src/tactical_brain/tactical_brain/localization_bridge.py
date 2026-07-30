"""Reconciles slam_toolbox's run-relative 'map' TF frame against the
robot's stable world-frame pose (/odometry/global, ArUco-corrected), and
bridges the overhead-camera ArUco pose into EKF fusion.

slam_toolbox's /map frame re-zeros at wherever the robot started this
run; current_x/current_y instead come from /odometry/global, a separate,
supposedly-stable source. Every point that crosses between the two
(outgoing FollowPath waypoints, incoming /map obstacles) needs the same
map<->world reconciliation - centralized here instead of duplicated
inline in two places.
"""
import math

from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from robot_localization.srv import SetPose

from tactical_brain import A_planner
from tactical_brain.geometry_utils import get_yaw_from_quaternion


class LocalizationBridge:
    # Above this much disagreement between ArUco and the currently tracked
    # pose, ekf_global's continuous /aruco/odom fusion (see
    # publish_aruco_odom) is assumed to not have caught up fast enough
    # (e.g. after wheel slip) - check_drift then force-reseeds it via the
    # /set_pose service instead of waiting for the continuous filter to
    # converge on its own.
    DRIFT_SNAP_THRESHOLD_METERS = 0.05

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, ros_node)
        # ArUco Odometry continuous publisher (for EKF fusion)
        self.aruco_odom_pub = ros_node.create_publisher(Odometry, '/aruco/odom', 10)

        # Large-drift snap-correction: force-reseed ekf_global's internal
        # state via its real /set_pose service (robot_localization) when
        # ArUco vs. current_x/y disagree too much - kept as an explicit
        # safety net on top of ekf_global's continuous fusion, which alone
        # may not catch up fast enough after a sudden large drift (e.g.
        # wheel slip). /initialpose has no subscriber in this stack (no
        # AMCL; slam_toolbox and robot_localization don't reset from that
        # topic), so this is the service ekf_global actually exposes for it.
        self.set_pose_client = ros_node.create_client(SetPose, '/ekf_global/set_pose')

    def get_map_pose(self):
        """Returns (map_x, map_y, map_yaw) for map->base_footprint.

        Raises LookupException/ConnectivityException/ExtrapolationException
        (tf2_ros) if the transform isn't available yet - callers decide how
        to handle that (see main_brain.py's map_callback and
        path_executor.py's PathExecutor.tick()).
        """
        tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        map_x = tf.transform.translation.x
        map_y = tf.transform.translation.y
        map_yaw = get_yaw_from_quaternion(tf.transform.rotation)
        return map_x, map_y, map_yaw

    def world_to_map(self, wx, wy, wyaw, current_x, current_y, current_yaw):
        """World-frame (wx, wy, wyaw) -> map-frame pose, for an outgoing
        FollowPath waypoint.

        ROOT CAUSE of a past overshoot investigation (2026-06-28):
        A_planner computes waypoints in the same world frame as
        current_x/current_y, but slam_toolbox's 'map' frame origin is the
        robot's spawn point, not the world origin - every goal needs this
        correction before nav2 sees it, or the spawn-point offset gets
        silently added on top of the intended target.
        """
        map_x, map_y, map_yaw = self.get_map_pose()

        yaw_offset = current_yaw - map_yaw
        cos_o = math.cos(yaw_offset)
        sin_o = math.sin(yaw_offset)

        dx_w = wx - current_x
        dy_w = wy - current_y
        dx_m = dx_w * cos_o + dy_w * sin_o
        dy_m = -dx_w * sin_o + dy_w * cos_o

        return (map_x + dx_m, map_y + dy_m, wyaw - yaw_offset)

    def transform_occupancy_grid_to_world(self, msg, current_x, current_y, current_yaw):
        """Inverse of world_to_map: reprojects /map's occupied cells into
        world-frame grid indices (A_planner.XY_RESOLUTION units) so A*'s
        obstacle set lines up with current_x/current_y. Raises the same TF
        exceptions as get_map_pose() if the transform isn't ready yet.
        """
        map_x, map_y, map_yaw = self.get_map_pose()

        yaw_offset = current_yaw - map_yaw
        cos_o = math.cos(yaw_offset)
        sin_o = math.sin(yaw_offset)

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
                wx = current_x + (dx * cos_o - dy * sin_o)
                wy = current_y + (dx * sin_o + dy * cos_o)
                obstacles.add((round(wx / A_planner.XY_RESOLUTION), round(wy / A_planner.XY_RESOLUTION)))

        return obstacles

    def publish_aruco_odom(self, aruco_x, aruco_y, aruco_yaw):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
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

    def check_drift(self, aruco_x, aruco_y, aruco_yaw, current_x, current_y, current_yaw):
        """Large-drift safety net alongside the continuous /aruco/odom
        fusion above (see ekf_global.yaml's odom2) - snap-corrects
        ekf_global via its real /set_pose service (robot_localization) when
        ArUco and the currently tracked pose disagree by more than
        DRIFT_SNAP_THRESHOLD_METERS. /initialpose has no subscriber in this
        stack (no AMCL; slam_toolbox and robot_localization don't reset
        from that topic), so this is the service ekf_global actually
        exposes for a one-shot reseed.
        """
        drift_x = aruco_x - current_x
        drift_y = aruco_y - current_y
        total_drift_meters = math.hypot(drift_x, drift_y)

        if total_drift_meters <= self.DRIFT_SNAP_THRESHOLD_METERS:
            return

        self.ros_node.get_logger().warn(
            f"HIGH ODOM DRIFT: {total_drift_meters * 100:.1f}cm! "
            "Resetting ekf_global to ArUco ground truth."
        )

        reset_msg = PoseWithCovarianceStamped()
        reset_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        reset_msg.header.frame_id = 'map'

        reset_msg.pose.pose.position.x = float(aruco_x)
        reset_msg.pose.pose.position.y = float(aruco_y)
        reset_msg.pose.pose.orientation.z = math.sin(aruco_yaw / 2.0)
        reset_msg.pose.pose.orientation.w = math.cos(aruco_yaw / 2.0)

        reset_msg.pose.covariance[0] = 0.05
        reset_msg.pose.covariance[7] = 0.05
        reset_msg.pose.covariance[35] = 0.05

        if self.set_pose_client.service_is_ready():
            request = SetPose.Request()
            request.pose = reset_msg
            self.set_pose_client.call_async(request)
        else:
            self.ros_node.get_logger().warn(
                "ekf_global/set_pose not available - drift reset skipped this tick."
            )