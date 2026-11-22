#ifndef TB4_AUTONOMOUS_PATROL_SAFE_RECOVERY_HPP_
#define TB4_AUTONOMOUS_PATROL_SAFE_RECOVERY_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Secuencia de recuperacion para simulacion y hardware:
// BackUp si hay cobertura 360 y corredor posterior despejado -> Spin seguro
// -> replanteamiento Nav2 gestionado por MissionSupervisor.
// NUNCA publica cmd_vel ni ignora Collision Monitor.
class SafeRecovery {
public:
  using Spin = nav2_msgs::action::Spin;
  using BackUp = nav2_msgs::action::BackUp;
  using SpinHandle = rclcpp_action::ClientGoalHandle<Spin>;
  using BackupHandle = rclcpp_action::ClientGoalHandle<BackUp>;
  using Clock = std::chrono::steady_clock;
  enum class State { IDLE, SENDING, ACTIVE, CANCELING, SUCCEEDED, FAILED };

  SafeRecovery(rclcpp::Node::SharedPtr node,
               rclcpp::Node::SharedPtr action_node);
  void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void start(int requested_direction = 0, bool allow_backup = true);
  int direction() const { return last_direction_; }
  void tick();
  void cancel();
  bool busy() const {
    return state_ == State::SENDING || state_ == State::ACTIVE ||
           state_ == State::CANCELING;
  }
  State state() const { return state_; }
  std::string reason() const { return reason_; }
  void reset() {
    if (!busy()) { state_ = State::IDLE; reason_.clear(); phase_ = Phase::NONE; }
  }

private:
  enum class Phase { NONE, BACKUP, SPIN };
  void fail(std::string message);
  void startBackup();
  void startSpin();
  void requestCancel(std::string reason);
  bool freshScan() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  State state_ = State::IDLE;
  Phase phase_ = Phase::NONE;
  std::string reason_;
  std::string cancellation_reason_;
  bool scan_valid_ = false;
  bool scan_complete_ = false;
  bool footprint_clear_ = false;
  bool rear_clear_ = false;
  double side_left_ = 0.0;
  double side_right_ = 0.0;
  int last_direction_ = 1;
  Clock::time_point scan_time_{};
  Clock::time_point deadline_{};

  SpinHandle::SharedPtr spin_handle_;
  std::shared_future<SpinHandle::SharedPtr> spin_send_;
  std::shared_future<SpinHandle::WrappedResult> spin_result_;
  BackupHandle::SharedPtr backup_handle_;
  std::shared_future<BackupHandle::SharedPtr> backup_send_;
  std::shared_future<BackupHandle::WrappedResult> backup_result_;
  bool cancel_pending_ = false;
};
#endif
