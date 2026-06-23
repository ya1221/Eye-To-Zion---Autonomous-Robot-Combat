// ============================================================
//  Position Listener Node
//
//  Subscribes to overhead ArUco tracker topics (bridged via Zenoh)
//  and republishes them as:
//    /aruco/odom   — nav_msgs/Odometry  (for the global EKF)
//    /aruco/target — geometry_msgs/Point (for path planning)
//
//  Parameters:
//    team_index           (int)    — which team this robot belongs to (default 0)
//    grid_to_meters_scale (double) — multiplier to convert grid coords to meters (default 1.0)
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PositionListenerNode : public rclcpp::Node
{
public:
    PositionListenerNode()
    : Node("position_listener_node")
    {
        // ── Parameters ──────────────────────────────────────────
        this->declare_parameter<int>("team_index", 0);
        this->declare_parameter<double>("grid_to_meters_scale", 1.0);

        team_index_ = this->get_parameter("team_index").as_int();
        scale_      = this->get_parameter("grid_to_meters_scale").as_double();

        // ── Subscribers ─────────────────────────────────────────
        // These topics are published by the remote PC's ArUco tracker
        // and bridged to the robot via Zenoh over Tailscale.
        std::string poses_topic  = "teams/team_" + std::to_string(team_index_) + "/positions";
        std::string target_topic = "teams/team_" + std::to_string(team_index_) + "/target_position";

        poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            poses_topic, 10,
            std::bind(&PositionListenerNode::poses_callback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            target_topic, 10,
            std::bind(&PositionListenerNode::target_callback, this, std::placeholders::_1));

        // ── Publishers ──────────────────────────────────────────
        odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("aruco/odom", 10);
        target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("aruco/target", 10);

        RCLCPP_INFO(this->get_logger(),
            "Position Listener ONLINE — team %d | scale %.4f | sub: [%s, %s]",
            team_index_, scale_, poses_topic.c_str(), target_topic.c_str());
    }

private:
    // ── Callbacks ───────────────────────────────────────────────

    void poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            return;
        }

        // Take the first pose in the array (single robot per team assumption).
        const auto & pose = msg->poses[0];

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = msg->header.stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id  = "base_link";

        // Position — apply grid→meters scaling
        odom.pose.pose.position.x = pose.position.x * scale_;
        odom.pose.pose.position.y = pose.position.y * scale_;
        odom.pose.pose.position.z = 0.0;

        // Orientation — pass through directly (already a quaternion)
        odom.pose.pose.orientation = pose.orientation;

        // Covariance — moderate confidence for an overhead camera measurement.
        // Diagonal: [x, y, z, roll, pitch, yaw]
        // These values tell the EKF how much to trust the ArUco reading.
        odom.pose.covariance[0]  = 0.05;   // x
        odom.pose.covariance[7]  = 0.05;   // y
        odom.pose.covariance[14] = 1e6;    // z      (unused, very high = ignore)
        odom.pose.covariance[21] = 1e6;    // roll   (unused)
        odom.pose.covariance[28] = 1e6;    // pitch  (unused)
        odom.pose.covariance[35] = 0.1;    // yaw

        odom_pub_->publish(odom);

        RCLCPP_DEBUG(this->get_logger(),
            "ArUco pose → /aruco/odom  x=%.1f y=%.1f yaw_q=(%.3f, %.3f)",
            odom.pose.pose.position.x, odom.pose.pose.position.y,
            pose.orientation.z, pose.orientation.w);
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        geometry_msgs::msg::Point scaled;
        scaled.x = msg->x * scale_;
        scaled.y = msg->y * scale_;
        scaled.z = 0.0;

        target_pub_->publish(scaled);

        RCLCPP_DEBUG(this->get_logger(),
            "Target → /aruco/target  x=%.1f y=%.1f", scaled.x, scaled.y);
    }

    // ── Members ─────────────────────────────────────────────────
    int    team_index_;
    double scale_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr     target_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     target_pub_;
};

// ================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionListenerNode>());
    rclcpp::shutdown();
    return 0;
}
