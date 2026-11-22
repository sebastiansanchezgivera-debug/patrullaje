#ifndef TB4_AUTONOMOUS_PATROL_SAFE_RECOVERY_HPP_
#define TB4_AUTONOMOUS_PATROL_SAFE_RECOVERY_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Recuperación acotada mediante Nav2 Spin; no publica cmd_vel y no desactiva
// Collision Monitor. Si los datos no demuestran espacio libre, NO gira.
class SafeRecovery {
public:
  using Spin = nav2_msgs::action::Spin;
  using Handle = rclcpp_action::ClientGoalHandle<Spin>;
  using Clock = std::chrono::steady_clock;
  enum class State { IDLE, SENDING, ACTIVE, CANCELING, SUCCEEDED, FAILED };
  SafeRecovery(rclcpp::Node::SharedPtr node,
               rclcpp::Node::SharedPtr action_node);
  void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void start(int requested_direction = 0);
  int direction() const { return last_direction_; }
  void tick();
  void cancel();
  bool busy() const {
    return state_ == State::SENDING || state_ == State::ACTIVE ||
           state_ == State::CANCELING;
  }
  State state() const { return state_; }
  std::string reason() const { return reason_; }
  void reset() { if (!busy()) { state_ = State::IDLE; reason_.clear(); } }
private:
  void fail(std::string message);
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Spin>::SharedPtr client_;
  State state_ = State::IDLE;
  std::string reason_;
  bool scan_valid_ = false;
  bool scan_complete_ = false;
  bool footprint_clear_ = false;
  double side_left_ = 0.0;
  double side_right_ = 0.0;
  int last_direction_ = 1;
  Clock::time_point scan_time_{};
  Clock::time_point deadline_{};
  Handle::SharedPtr handle_;
  std::shared_future<Handle::SharedPtr> send_;
  std::shared_future<Handle::WrappedResult> result_;
  bool cancel_pending_ = false;
};
#endif
