#include "tb4_autonomous_patrol/dock_controller.hpp"
#include <algorithm>

using namespace std::chrono_literals;

namespace {
std::chrono::steady_clock::duration secondsDuration(double v) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, v)));
}
int64_t monotonicMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

DockController::DockController(rclcpp::Node::SharedPtr node,
                               rclcpp::Node::SharedPtr action_node)
    : simulated_(node->get_parameter("simulate_docking").as_bool()),
      node_(std::move(node)), action_node_(std::move(action_node)) {
#ifdef TB4_HAS_CREATE3
  dock_client_ = rclcpp_action::create_client<Dock>(
      action_node_, node_->get_parameter("dock_action_name").as_string());
  undock_client_ = rclcpp_action::create_client<Undock>(
      action_node_, node_->get_parameter("undock_action_name").as_string());
  status_sub_ = action_node_->create_subscription<irobot_create_msgs::msg::DockStatus>(
      node_->get_parameter("dock_status_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
        status_seen_.store(true);
        status_docked_.store(msg->is_docked);
        status_time_ms_.store(monotonicMs());
      });
#endif
}

const char* DockController::name(State s) {
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::DOCKING: return "DOCKING";
    case State::DOCKED: return "DOCKED";
    case State::UNDOCKING: return "UNDOCKING";
    case State::CANCELING: return "CANCELING_DOCK";
    case State::FAILED: return "DOCKING_FAILED";
  }
  return "UNKNOWN";
}

void DockController::failed(const std::string & msg) {
  error_ = msg;
  state_ = State::FAILED;
  RCLCPP_ERROR(node_->get_logger(), "DOCKING: %s", msg.c_str());
}

bool DockController::dockStatusFresh() const {
#ifdef TB4_HAS_CREATE3
  return status_seen_.load() &&
      monotonicMs() - status_time_ms_.load() < 3000;
#else
  return false;
#endif
}

bool DockController::canConfirmDock() const {
  return simulated_ || (dockStatusFresh()
#ifdef TB4_HAS_CREATE3
      && status_docked_.load()
#endif
  );
}

bool DockController::stageNavigationReady() const {
  if (!node_->get_parameter("enable_stage_docking_actions").as_bool()) return true;
  if (!node_->get_parameter("use_sim_time").as_bool() ||
      node_->get_parameter("enable_real_docking").as_bool() || simulated_) return false;
#ifdef TB4_HAS_CREATE3
  return dockStatusFresh() && !status_docked_.load();
#else
  return false;
#endif
}

void DockController::startDock() {
  if (busy() || state_ == State::DOCKED) return;
  error_.clear();
  if (simulated_) {
    state_ = State::DOCKING;
    deadline_ = Clock::now() + secondsDuration(
        node_->get_parameter("sim_dock_duration_sec").as_double());
    return;
  }
#ifndef TB4_HAS_CREATE3
  failed("irobot_create_msgs no instalado al compilar; no es posible docking real");
#else
  const bool sim_time = node_->get_parameter("use_sim_time").as_bool();
  const bool stage = node_->get_parameter("enable_stage_docking_actions").as_bool();
  const bool real = node_->get_parameter("enable_real_docking").as_bool();
  if (!(sim_time ? (stage && !real) : (real && !stage))) {
    failed("Acciones de docking no autorizadas para el entorno");
    return;
  }
  if (!dock_client_->action_server_is_ready()) {
    failed("Servidor /dock no disponible");
    return;
  }
  cancel_pending_ = false;
  docking_result_success_ = false;
  dock_handle_.reset(); dock_send_ = {}; dock_result_ = {};
  if (dockStatusFresh() && status_docked_.load()) {
    state_ = State::DOCKED;
    return;
  }
  dock_send_ = dock_client_->async_send_goal(Dock::Goal{});
  state_ = State::DOCKING;
  deadline_ = Clock::now() + secondsDuration(
      node_->get_parameter("dock_timeout_sec").as_double());
#endif
}

void DockController::startUndock() {
  if (busy() || state_ != State::DOCKED) return;
  error_.clear();
  if (simulated_) {
    state_ = State::UNDOCKING;
    deadline_ = Clock::now() + secondsDuration(
        node_->get_parameter("sim_undock_duration_sec").as_double());
    return;
  }
#ifndef TB4_HAS_CREATE3
  failed("irobot_create_msgs no instalado al compilar; no es posible undocking real");
#else
  const bool sim_time = node_->get_parameter("use_sim_time").as_bool();
  const bool stage = node_->get_parameter("enable_stage_docking_actions").as_bool();
  const bool real = node_->get_parameter("enable_real_docking").as_bool();
  if (!(sim_time ? (stage && !real) : (real && !stage))) {
    failed("Acciones de docking no autorizadas para el entorno");
    return;
  }
  if (!undock_client_->action_server_is_ready()) {
    failed("Servidor /undock no disponible");
    return;
  }
  cancel_pending_ = false;
  undocking_result_success_ = false;
  undock_handle_.reset(); undock_send_ = {}; undock_result_ = {};
  undock_send_ = undock_client_->async_send_goal(Undock::Goal{});
  state_ = State::UNDOCKING;
  deadline_ = Clock::now() + secondsDuration(
      node_->get_parameter("undock_timeout_sec").as_double());
#endif
}

void DockController::cancel() {
  if (!busy()) return;
  if (simulated_) { state_ = State::IDLE; return; }
#ifdef TB4_HAS_CREATE3
  cancel_pending_ = true;
  state_ = State::CANCELING;
  // Si la peticion aún no fue aceptada, tick() cancelará tras la aceptación.
  if (dock_handle_) dock_client_->async_cancel_goal(dock_handle_);
  if (undock_handle_) undock_client_->async_cancel_goal(undock_handle_);
  deadline_ = Clock::now() + 10s;
#endif
}

void DockController::tick() {
  if (!busy()) return;
  if (simulated_) {
    if (Clock::now() < deadline_) return;
    if (state_ == State::DOCKING) state_ = State::DOCKED;
    else if (state_ == State::UNDOCKING) state_ = State::IDLE;
    return;
  }
#ifdef TB4_HAS_CREATE3
  const bool canceling = state_ == State::CANCELING;
  if (dock_send_.valid() && !dock_handle_ &&
      dock_send_.wait_for(0ms) == std::future_status::ready) {
    dock_handle_ = dock_send_.get(); dock_send_ = {};
    if (!dock_handle_) { failed("/dock rechazo objetivo"); return; }
    dock_result_ = dock_client_->async_get_result(dock_handle_);
    if (cancel_pending_) dock_client_->async_cancel_goal(dock_handle_);
  }
  if (undock_send_.valid() && !undock_handle_ &&
      undock_send_.wait_for(0ms) == std::future_status::ready) {
    undock_handle_ = undock_send_.get(); undock_send_ = {};
    if (!undock_handle_) { failed("/undock rechazo objetivo"); return; }
    undock_result_ = undock_client_->async_get_result(undock_handle_);
    if (cancel_pending_) undock_client_->async_cancel_goal(undock_handle_);
  }
  if (dock_result_.valid() &&
      dock_result_.wait_for(0ms) == std::future_status::ready) {
    const auto result = dock_result_.get();
    dock_result_ = {};
    if (canceling) { state_ = State::IDLE; return; }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      failed("/dock termino sin exito"); return;
    }
    docking_result_success_ = true;
  }
  if (undock_result_.valid() &&
      undock_result_.wait_for(0ms) == std::future_status::ready) {
    const auto result = undock_result_.get();
    undock_result_ = {};
    if (canceling) { state_ = State::IDLE; return; }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      failed("/undock termino sin exito"); return;
    }
    undocking_result_success_ = true;
  }
  if (state_ == State::DOCKING && docking_result_success_ &&
      dockStatusFresh() && status_docked_.load()) {
    state_ = State::DOCKED;
    return;
  }
  if (state_ == State::UNDOCKING && undocking_result_success_ &&
      dockStatusFresh() && !status_docked_.load()) {
    state_ = State::IDLE;
    return;
  }
  if (canceling && !dock_result_.valid() && !undock_result_.valid() &&
      !dock_send_.valid() && !undock_send_.valid()) {
    state_ = State::IDLE;
    return;
  }
  if (Clock::now() >= deadline_) {
    // Puede seguir actuando un servidor remoto: NO asumir cancelacion segura.
    if (dock_handle_) dock_client_->async_cancel_goal(dock_handle_);
    if (undock_handle_) undock_client_->async_cancel_goal(undock_handle_);
    failed("Tiempo de docking/undocking excedido; inspeccionar acciones remotas");
  }
#endif
}
