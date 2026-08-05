"""Shared quaternion->yaw conversion - the one piece of frame math used by
both pose ingestion (main_brain.py's pose_callback) and the map<->world
reconciliation (localization_bridge.py), so there's a single definition
instead of two copies drifting apart.
"""
import math


def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)