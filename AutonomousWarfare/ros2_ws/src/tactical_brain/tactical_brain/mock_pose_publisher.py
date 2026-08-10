import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates


class MockPosePublisher(Node):
    """Gazebo-only stand-in for the real EKF + overhead-ArUco-camera pose source ('itay_amcl_topic'); republishes Gazebo's ground-truth world-frame pose, which is stable/run-independent like the real overhead camera will be, unlike slam_toolbox's re-zeroing map frame.
    Do not add this node to main.launch.py - it has no business running on the RPi5."""

    ROBOT_ENTITY_NAME = 'robot1'

    def __init__(self):
        super().__init__('mock_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'itay_amcl_topic', 10)
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_model_states, 10
        )

    def on_model_states(self, msg):
        try:
            idx = msg.name.index(self.ROBOT_ENTITY_NAME)
        except ValueError:
            return

        pose = msg.pose[idx]

        out = PoseWithCovarianceStamped()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose = pose
        out.pose.covariance[0] = 0.05
        out.pose.covariance[7] = 0.05
        out.pose.covariance[35] = 0.05
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()