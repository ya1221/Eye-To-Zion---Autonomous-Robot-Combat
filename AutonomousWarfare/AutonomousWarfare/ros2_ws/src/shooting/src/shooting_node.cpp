#include "shooting/shooting_node.hpp"

#include <chrono>
#include <functional>

static constexpr double DEFAULT_FIRE_RATE_HZ = 2.0;

ShootingNode::ShootingNode() : Node("shooting_node") {
    pub_ = create_publisher<std_msgs::msg::Bool>("/shooting_cmd", 10);

    // ── Parameters ──────────────────────────────────────────────────────────
    // fire_rate_hz: shots-per-second in auto mode.  Changeable at runtime via
    // `ros2 param set` but ONLY while the mode is "auto".
    declare_parameter("fire_rate_hz", DEFAULT_FIRE_RATE_HZ);

    // Register a dynamic-parameter callback to enforce constraints.
    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&ShootingNode::on_parameters_changed, this, std::placeholders::_1));

    // ── Mode subscription ───────────────────────────────────────────────────
    // Another node (e.g. main_brain) publishes "single" or "auto" here.
    mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/shooting_mode", 10,
        std::bind(&ShootingNode::mode_callback, this, std::placeholders::_1));

    // ── Single-shot service ─────────────────────────────────────────────────
    // `ros2 service call /shooting_node/fire_once std_srvs/srv/Trigger`
    fire_once_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/fire_once",
        std::bind(&ShootingNode::fire_once_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    last_toggle_time_ = this->now();

    // Fast fixed-rate heartbeat (100 Hz). fire_rate_hz is re-read from the
    // parameter server on every tick, so runtime changes take effect on the
    // next tick — no restart, no timer recreation needed.
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&ShootingNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "ShootingNode started  —  mode=%s  rate=%.1f Hz",
                fire_mode_.c_str(), get_parameter("fire_rate_hz").as_double());
}

// ─── Mode subscription callback ────────────────────────────────────────────
void ShootingNode::mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& new_mode = msg->data;

    if (new_mode != "auto" && new_mode != "single") {
        RCLCPP_WARN(get_logger(),
                    "Ignoring unknown shooting mode '%s' (must be 'auto' or 'single')",
                    new_mode.c_str());
        return;
    }
    if (new_mode == fire_mode_) {
        return;  // no change
    }

    std::string old_mode = fire_mode_;
    fire_mode_ = new_mode;

    if (new_mode == "single") {
        // Rule 4: switching to single → reset fire_rate_hz to default.
        // The internal flag bypasses on_parameters_changed's single-mode block.
        internal_rate_reset_ = true;
        set_parameter(rclcpp::Parameter("fire_rate_hz", DEFAULT_FIRE_RATE_HZ));
        internal_rate_reset_ = false;
        // Make sure any leftover HIGH is cleared.
        if (state_) {
            state_ = false;
            publish_state();
        }
        single_shot_pending_ = false;
        RCLCPP_INFO(get_logger(),
                    "Mode changed: %s → single  (fire_rate_hz reset to %.1f)",
                    old_mode.c_str(), DEFAULT_FIRE_RATE_HZ);
    } else {
        // Switching to auto — fire_rate_hz keeps whatever it currently is
        // (already DEFAULT_FIRE_RATE_HZ if coming from single per rule 4,
        //  or whatever the user set previously in auto).
        // Rule 3: the parameter already holds a valid value (either the
        // default or a user-set one), so no special handling needed.
        last_toggle_time_ = this->now();
        RCLCPP_INFO(get_logger(),
                    "Mode changed: %s → auto  (fire_rate_hz = %.1f)",
                    old_mode.c_str(), get_parameter("fire_rate_hz").as_double());
    }
}

// ─── Dynamic parameter callback ────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult ShootingNode::on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& p : params) {
        if (p.get_name() == "fire_rate_hz") {
            // Rule 5: cannot change fire_rate_hz while in single mode.
            // (internal_rate_reset_ allows the mode_callback's own reset.)
            if (fire_mode_ == "single" && !internal_rate_reset_) {
                result.successful = false;
                result.reason = "fire_rate_hz cannot be changed while in single mode";
                return result;
            }
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                p.as_double() <= 0.0) {
                result.successful = false;
                result.reason = "fire_rate_hz must be a positive double";
                return result;
            }
            RCLCPP_INFO(get_logger(), "fire_rate_hz changed to %.2f", p.as_double());
        }
    }
    return result;
}

// ─── Single-shot service ────────────────────────────────────────────────────
void ShootingNode::fire_once_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (fire_mode_ != "single") {
        response->success = false;
        response->message =
            "Cannot fire_once: mode is '" + fire_mode_ + "', not 'single'";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }
    if (!firing_) {
        response->success = false;
        response->message =
            "Cannot fire_once: firing is disabled (set_firing(false))";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }

    single_shot_pending_ = true;
    response->success = true;
    response->message = "Single shot queued";
    RCLCPP_INFO(get_logger(), "Single shot queued");
}

void ShootingNode::set_firing(bool on) {
    firing_ = on;
}

// ─── Main tick (100 Hz heartbeat) ───────────────────────────────────────────
void ShootingNode::tick() {
    if (!firing_) {
        if (state_) {
            state_ = false;
            publish_state();
        }
        return;
    }

    // ── Single mode ─────────────────────────────────────────────────────────
    if (fire_mode_ == "single") {
        if (single_shot_pending_) {
            // Produce one HIGH→LOW pulse across two consecutive ticks.
            if (!state_) {
                state_ = true;   // rising edge
                publish_state();
            } else {
                state_ = false;  // falling edge — done
                publish_state();
                single_shot_pending_ = false;
            }
        } else if (state_) {
            state_ = false;
            publish_state();
        }
        return;
    }

    // ── Auto mode ───────────────────────────────────────────────────────────
    double hz = get_parameter("fire_rate_hz").as_double();
    if (hz <= 0.0) {
        return;
    }
    double half_period_sec = 1.0 / (2.0 * hz);

    rclcpp::Time now = this->now();
    if ((now - last_toggle_time_).seconds() >= half_period_sec) {
        state_ = !state_;
        last_toggle_time_ = now;
        publish_state();
    }
}

void ShootingNode::publish_state() {
    std_msgs::msg::Bool msg;
    msg.data = state_;
    pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShootingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}