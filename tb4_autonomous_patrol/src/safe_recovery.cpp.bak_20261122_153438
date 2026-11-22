#include "tb4_autonomous_patrol/safe_recovery.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

using namespace std::chrono_literals;

namespace {
constexpr double PI = 3.14159265358979323846;
std::chrono::steady_clock::duration durationSeconds(double d) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, d)));
}
}

SafeRecovery::SafeRecovery(rclcpp::Node::SharedPtr node,
                           rclcpp::Node::SharedPtr action_node)
    : node_(std::move(node)) {
  client_ = rclcpp_action::create_client<Spin>(action_node, "/spin");
}

void SafeRecovery::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  scan_time_ = Clock::now();
  scan_valid_ = msg->angle_increment > 0.0 && msg->range_max > 0.0;
  footprint_clear_ = scan_valid_;
  // Con menos de 360 grados NO se puede asegurar el despeje posterior.
  scan_complete_ = (msg->angle_min <= -PI + 0.10 &&
                    msg->angle_max >= PI - 0.10);
  side_left_ = side_right_ = 0.0;
  int left_count = 0, right_count = 0;
  const double laser_x = node_->get_parameter("recovery_laser_x_m").as_double();
  const double clearance = node_->get_parameter("robot_radius_m").as_double() +
      node_->get_parameter("recovery_margin_m").as_double();
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double distance = msg->ranges[i];
    if (!std::isfinite(distance) || distance < msg->range_min ||
        distance > msg->range_max) {
      // Una lectura NaN o menor al mínimo impide asegurar el despeje.
      if (std::isnan(distance) || distance < msg->range_min) footprint_clear_ = false;
      continue;
    }
    const double bx = laser_x + distance * std::cos(angle);
    const double by = distance * std::sin(angle);
    if (std::hypot(bx, by) < clearance) footprint_clear_ = false;
    if (angle >= 0.35 && angle <= 1.5) { side_left_ += std::min(distance, 2.0); ++left_count; }
    if (angle <= -0.35 && angle >= -1.5) { side_right_ += std::min(distance, 2.0); ++right_count; }
  }
  if (left_count > 0) side_left_ /= left_count;
  if (right_count > 0) side_right_ /= right_count;
  if (left_count == 0 || right_count == 0) scan_valid_ = false;
}

void SafeRecovery::fail(std::string message) {
  reason_ = std::move(message);
  state_ = State::FAILED;
  RCLCPP_WARN(node_->get_logger(), "RECOVERY: %s", reason_.c_str());
}

void SafeRecovery::start(int requested_direction) {
  if (busy()) return;
  reset();
  if (!node_->get_parameter("enable_spin_recovery").as_bool()) {
    fail("Spin deshabilitado por parametro"); return;
  }
  const double age = std::chrono::duration<double>(Clock::now() - scan_time_).count();
  if (!scan_valid_ || age > node_->get_parameter("laser_timeout_sec").as_double()) {
    fail("Lectura LiDAR no valida/reciente"); return;
  }
  if (!footprint_clear_) { fail("Radio de giro sin despeje suficiente"); return; }
  if (!scan_complete_) {
    const bool stage_only =
        node_->get_parameter("allow_partial_scan_sim_only").as_bool() &&
        node_->get_parameter("simulate_docking").as_bool() &&
        node_->get_parameter("use_sim_time").as_bool();
    if (!stage_only) {
      fail("LiDAR parcial: giro prohibido salvo modo experimental Stage"); return;
    }
  }
  if (!client_->action_server_is_ready()) { fail("Accion Nav2 /spin no disponible"); return; }
  const double degrees = node_->get_parameter("recovery_spin_deg").as_double();
  const double seconds = node_->get_parameter("recovery_spin_timeout_sec").as_double();
  if (!std::isfinite(degrees) || degrees <= 0.0 || degrees > 90.0 ||
      !std::isfinite(seconds) || seconds < 1.0) {
    fail("Parametros de giro invalidos"); return;
  }
  Spin::Goal goal;
  // Elegimos un lado la primera vez y lo mantenemos durante toda la
  // exploracion: evita oscilaciones izquierda-derecha entre segmentos.
  last_direction_ = requested_direction == 0 ?
      (side_left_ >= side_right_ ? 1 : -1) :
      (requested_direction > 0 ? 1 : -1);
  goal.target_yaw = static_cast<float>(last_direction_ * degrees * PI / 180.0);
  goal.time_allowance.sec = static_cast<int32_t>(std::ceil(seconds));
  goal.time_allowance.nanosec = 0;
  handle_.reset(); send_ = {}; result_ = {}; cancel_pending_ = false;
  // Diagnostico de progreso: no altera el movimiento ni las protecciones.
  rclcpp_action::Client<Spin>::SendGoalOptions options;
  const auto logger = node_->get_logger();
  const auto clock = node_->get_clock();
  options.feedback_callback =
      [logger, clock](Handle::SharedPtr,
                      const std::shared_ptr<const Spin::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(
            logger, *clock, 1000,
            "RECOVERY: Spin avance angular %.1f grados",
            static_cast<double>(feedback->angular_distance_traveled) * 180.0 / PI);
      };
  send_ = client_->async_send_goal(goal, options);
  deadline_ = Clock::now() + durationSeconds(seconds + 3.0);
  state_ = State::SENDING;
  RCLCPP_INFO(node_->get_logger(), "RECOVERY: solicitando Spin %.1f grados", double(goal.target_yaw) * 180.0 / PI);
}

void SafeRecovery::tick() {
  if (!busy()) return;
  if (state_ == State::SENDING || (state_ == State::CANCELING && !handle_)) {
    if (send_.valid() && send_.wait_for(0ms) == std::future_status::ready) {
      handle_ = send_.get(); send_ = {};
      if (!handle_) { fail("Nav2 rechazo Spin"); return; }
      RCLCPP_INFO(node_->get_logger(), "RECOVERY: Nav2 ACEPTO Spin");
      result_ = client_->async_get_result(handle_);
      if (cancel_pending_) {
        client_->async_cancel_goal(handle_);
        state_ = State::CANCELING;
      } else {
        state_ = State::ACTIVE;
      }
    }
  }
  if ((state_ == State::ACTIVE || state_ == State::CANCELING) && result_.valid() &&
      result_.wait_for(0ms) == std::future_status::ready) {
    const auto result = result_.get(); result_ = {};
    if (state_ == State::CANCELING) { fail("Spin cancelado"); return; }
    const char * code_name = "DESCONOCIDO";
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) code_name = "SUCCEEDED";
    else if (result.code == rclcpp_action::ResultCode::ABORTED) code_name = "ABORTED";
    else if (result.code == rclcpp_action::ResultCode::CANCELED) code_name = "CANCELED";
    else if (result.code == rclcpp_action::ResultCode::UNKNOWN) code_name = "UNKNOWN";
    const double elapsed = result.result
        ? static_cast<double>(result.result->total_elapsed_time.sec) +
              static_cast<double>(result.result->total_elapsed_time.nanosec) * 1.0e-9
        : -1.0;
    RCLCPP_INFO(node_->get_logger(),
                "RECOVERY: resultado Spin=%s, tiempo_nav2=%.2f s",
                code_name, elapsed);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      state_ = State::SUCCEEDED;
      reason_.clear();
    } else {
      fail(std::string("Spin termino en estado ") + code_name +
           "; verificar /cmd_vel_raw, /cmd_vel y logs de behavior_server");
    }
    return;
  }
  if (Clock::now() >= deadline_) {
    if (handle_) client_->async_cancel_goal(handle_);
    fail("Tiempo de Spin excedido; verificar cancelacion en Nav2");
  }
}

void SafeRecovery::cancel() {
  if (!busy()) return;
  cancel_pending_ = true;
  state_ = State::CANCELING;
  if (handle_) client_->async_cancel_goal(handle_);
  deadline_ = Clock::now() + 10s;
}
