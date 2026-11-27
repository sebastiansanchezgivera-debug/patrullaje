#ifndef TB4_AUTONOMOUS_PATROL_DOCK_CONTROLLER_HPP_
#define TB4_AUTONOMOUS_PATROL_DOCK_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#ifdef TB4_HAS_CREATE3
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#endif

// Docking real es opt-in. El modo simulado nunca finge medir una bateria
// real ni confirma contacto electrico.
class DockController {
public:
  using Clock = std::chrono::steady_clock;
  enum class State { IDLE, DOCKING, DOCKED, UNDOCKING, CANCELING, FAILED };

  DockController(rclcpp::Node::SharedPtr param_node,
                 rclcpp::Node::SharedPtr action_node);
  void startDock();
  void startUndock();
  void tick();
  void cancel();
  State state() const { return state_; }
  std::string error() const { return error_; }
  bool busy() const { return state_ == State::DOCKING ||
                             state_ == State::UNDOCKING ||
                             state_ == State::CANCELING; }
  bool docked() const { return state_ == State::DOCKED; }
  bool idle() const { return state_ == State::IDLE; }
  bool realMode() const { return !simulated_; }
  bool canConfirmDock() const;
  bool stageNavigationReady() const;
  static const char* name(State s);

private:
  void failed(const std::string & msg);
  bool dockStatusFresh() const;
  bool simulated_ = true;
  State state_ = State::IDLE;
  std::string error_;
  Clock::time_point deadline_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr action_node_;
#ifdef TB4_HAS_CREATE3
  using Dock = irobot_create_msgs::action::Dock;
  using Undock = irobot_create_msgs::action::Undock;
  using DockHandle = rclcpp_action::ClientGoalHandle<Dock>;
  using UndockHandle = rclcpp_action::ClientGoalHandle<Undock>;
  rclcpp_action::Client<Dock>::SharedPtr dock_client_;
  rclcpp_action::Client<Undock>::SharedPtr undock_client_;
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr status_sub_;
  std::atomic<bool> status_seen_{false};
  std::atomic<bool> status_docked_{false};
  std::atomic<int64_t> status_time_ms_{0};
  bool docking_result_success_ = false;
  bool undocking_result_success_ = false;
  bool cancel_pending_ = false;
  DockHandle::SharedPtr dock_handle_;
  UndockHandle::SharedPtr undock_handle_;
  std::shared_future<DockHandle::SharedPtr> dock_send_;
  std::shared_future<UndockHandle::SharedPtr> undock_send_;
  std::shared_future<DockHandle::WrappedResult> dock_result_;
  std::shared_future<UndockHandle::WrappedResult> undock_result_;
#endif
};
#endif
