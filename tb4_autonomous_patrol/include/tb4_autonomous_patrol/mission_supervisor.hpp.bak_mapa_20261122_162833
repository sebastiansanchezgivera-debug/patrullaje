#ifndef TB4_AUTONOMOUS_PATROL_MISSION_SUPERVISOR_HPP_
#define TB4_AUTONOMOUS_PATROL_MISSION_SUPERVISOR_HPP_

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <thread>
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tb4_autonomous_patrol/dock_controller.hpp"
#include "tb4_autonomous_patrol/safe_recovery.hpp"

class MissionSupervisor : public BT::StatefulActionNode
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Steady = std::chrono::steady_clock;

  MissionSupervisor(const std::string & name, const BT::NodeConfig & config);
  ~MissionSupervisor() override;
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class GoalKind { NONE, STATION, HOME, EMERGENCY };
  enum class NavPhase { IDLE, SENDING, ACTIVE, CANCELING };

  struct Target {
    GoalKind kind = GoalKind::NONE;
    int station = -1;
    geometry_msgs::msg::PoseStamped pose;
    bool valid = false;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateAvoidProgress();
  void resetAvoidCycles();
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void emergencyCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool laserFresh() const;
  bool batteryFresh() const;
  void updateBatteryLatch();
  bool obstaclePresent() const;
  Target makeTarget(GoalKind kind, int station = -1) const;
  Target requestedTarget();
  bool sameTarget(const Target & a, const Target & b) const;
  void advanceNavigation(const Target & wanted);
  void advanceRecovery();
  void advanceDocking();
  void onGoalCompleted(const Target & goal, bool succeeded);
  void clearGoal();
  void setFault(const std::string & why);
  void publishStatus(bool force = false);
  std::string statusName() const;
  void markEvent(const std::string & message);

  std::shared_ptr<rclcpp::Node> node_;
  // El cliente de acciones tiene su propio nodo y executor.
  rclcpp::Node::SharedPtr action_node_;
  rclcpp_action::Client<Action>::SharedPtr nav_client_;
  std::unique_ptr<SafeRecovery> recovery_;
  std::unique_ptr<DockController> dock_;
  bool recovery_requested_ = false;
  int recovery_spin_count_ = 0;
  int recovery_direction_ = 0;
  // Contador entre comandos AVOID para evitar repeticiones sobre el mismo obstaculo.
  int avoid_cycles_without_progress_ = 0;
  bool avoid_loop_hold_ = false;
  bool odom_valid_ = false;
  bool avoid_anchor_valid_ = false;
  double odom_x_ = 0.0, odom_y_ = 0.0;
  double avoid_anchor_x_ = 0.0, avoid_anchor_y_ = 0.0;
  DockController::State last_dock_state_ = DockController::State::IDLE;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> action_executor_;
  std::thread action_spin_thread_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr emergency_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  bool scan_received_ = false;
  bool front_valid_ = false;
  double front_distance_ = std::numeric_limits<double>::infinity();
  Steady::time_point last_scan_{};

  bool battery_received_ = false;
  double battery_real_pct_ = std::numeric_limits<double>::quiet_NaN();
  double battery_pct_ = std::numeric_limits<double>::quiet_NaN();
  bool battery_hold_ = false;
  Steady::time_point last_battery_{};

  bool pause_requested_ = false;
  bool canceled_ = false;
  bool emergency_active_ = false;
  bool emergency_arrived_ = false;
  bool home_arrived_ = false;
  bool avoiding_ = false;
  bool station_waiting_ = false;
  int station_index_ = 0;
  geometry_msgs::msg::PoseStamped emergency_pose_;
  Steady::time_point avoid_deadline_{};
  Steady::time_point station_wait_deadline_{};
  Steady::time_point last_status_{};
  std::string event_ = "INICIADO";
  std::string fault_;

  NavPhase nav_phase_ = NavPhase::IDLE;
  Target active_target_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool cancellation_sent_ = false;
  Steady::time_point phase_deadline_{};
};

#endif
