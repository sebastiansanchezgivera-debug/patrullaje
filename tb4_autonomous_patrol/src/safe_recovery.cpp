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
  spin_client_ = rclcpp_action::create_client<Spin>(action_node, "/spin");
  backup_client_ = rclcpp_action::create_client<BackUp>(action_node, "/backup");
}

void SafeRecovery::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                              double laser_x, double laser_y, double laser_yaw) {
  scan_time_ = Clock::now();
  scan_valid_ = std::isfinite(laser_x) && std::isfinite(laser_y) &&
      std::isfinite(laser_yaw) && std::isfinite(msg->angle_increment) &&
      std::isfinite(msg->angle_min) && std::isfinite(msg->angle_max) &&
      std::isfinite(msg->range_max) && std::isfinite(msg->range_min) &&
      msg->range_min >= 0.0 && msg->range_max > msg->range_min &&
      msg->angle_increment > 0.0 && !msg->ranges.empty();
  if (!scan_valid_) { invalidateScan(); return; }
  scan_complete_ = scan_valid_ &&
      msg->angle_min <= -PI + 0.10 && msg->angle_max >= PI - 0.10 &&
      (msg->ranges.size() - 1) * msg->angle_increment >= 2.0 * PI - 0.20;
  footprint_clear_ = scan_valid_;
  rear_clear_ = scan_complete_;
  side_left_ = side_right_ = 0.0;
  int left_count = 0, right_count = 0, rear_count = 0;

  const double radius = node_->get_parameter("robot_radius_m").as_double() +
      node_->get_parameter("recovery_margin_m").as_double();
  const double distance = node_->get_parameter("recovery_backup_m").as_double();
  if (!std::isfinite(radius) || radius <= 0.0 ||
      msg->range_max < radius + std::hypot(laser_x, laser_y)) {
    invalidateScan(); return;
  }
  if (!std::isfinite(distance) || distance <= 0.0 ||
      msg->range_max < distance + radius + std::hypot(laser_x, laser_y)) {
    rear_clear_ = false;
  }

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle = std::remainder(msg->angle_min +
        static_cast<double>(i) * msg->angle_increment + laser_yaw, 2.0 * PI);
    const double r = msg->ranges[i];
    const bool behind = std::fabs(angle) >= 2.35; // Sector posterior (+/-45 grados)
    if (behind) ++rear_count;

    // No deducir espacio libre a partir de NaN o una lectura inferior al minimo.
    if (std::isnan(r) || r < msg->range_min || r < 0.0) {
      if (behind) rear_clear_ = false;
      footprint_clear_ = false;
      continue;
    }
    if (!std::isfinite(r)) {
      if (r < 0.0) { rear_clear_ = false; footprint_clear_ = false; }
      continue; // +inf significa sin retorno dentro del rango anunciado
    }
    if (r > msg->range_max) continue;

    // Impacto en base_link: traslacion y yaw del montaje planar.
    const double x = laser_x + r * std::cos(angle);
    const double y = laser_y + r * std::sin(angle);
    if (std::hypot(x, y) < radius) footprint_clear_ = false;

    // Colision con el circulo barrido durante BackUp desde x=0 a x=-distance.
    const double closest_x = std::clamp(x, -distance, 0.0);
    if (std::hypot(x - closest_x, y) < radius) rear_clear_ = false;

    if (angle >= 0.35 && angle <= 1.5) {
      side_left_ += std::min(r, 2.0);
      ++left_count;
    }
    if (angle <= -0.35 && angle >= -1.5) {
      side_right_ += std::min(r, 2.0);
      ++right_count;
    }
  }
  if (left_count > 0) side_left_ /= left_count;
  if (right_count > 0) side_right_ /= right_count;
  if (left_count == 0 || right_count == 0) scan_valid_ = false;
  if (rear_count < 10 || !scan_complete_) rear_clear_ = false;
}

bool SafeRecovery::freshScan() const {
  const double age = std::chrono::duration<double>(Clock::now() - scan_time_).count();
  const double max_age = node_->get_parameter("laser_timeout_sec").as_double();
  return scan_valid_ && std::isfinite(max_age) && max_age > 0.0 &&
         age <= max_age;
}

void SafeRecovery::fail(std::string message) {
  reason_ = std::move(message);
  state_ = State::FAILED;
  RCLCPP_WARN(node_->get_logger(), "RECOVERY: %s", reason_.c_str());
}

void SafeRecovery::start(int requested_direction, bool allow_backup) {
  if (busy()) return;
  reset();
  if (!node_->get_parameter("enable_spin_recovery").as_bool()) {
    fail("Spin deshabilitado por parametro"); return;
  }
  if (!freshScan()) { fail("Lectura LiDAR no valida/reciente"); return; }
  if (!footprint_clear_) { fail("Radio de giro sin despeje suficiente"); return; }

  // En hardware no se acepta la excepcion de cobertura parcial.
  if (!scan_complete_) {
    const bool stage_only =
        node_->get_parameter("allow_partial_scan_sim_only").as_bool() &&
        node_->get_parameter("simulate_docking").as_bool() &&
        node_->get_parameter("use_sim_time").as_bool();
    if (!stage_only) {
      fail("LiDAR parcial: recovery no segura para robot fisico"); return;
    }
  }

  last_direction_ = requested_direction == 0 ?
      (side_left_ >= side_right_ ? 1 : -1) :
      (requested_direction > 0 ? 1 : -1);

  // La simulacion Stage con LiDAR parcial NUNCA autoriza retroceso a ciegas.
  if (allow_backup &&
      node_->get_parameter("enable_backup_recovery").as_bool() &&
      rear_clear_ && scan_complete_) {
    startBackup();
  } else {
    if (allow_backup && node_->get_parameter("enable_backup_recovery").as_bool()) {
      RCLCPP_WARN(node_->get_logger(),
          "RECOVERY: retroceso OMITIDO: cobertura posterior/espacio no validos");
    }
    startSpin();
  }
}

void SafeRecovery::startBackup() {
  if (!backup_client_->action_server_is_ready()) {
    fail("Accion Nav2 /backup no disponible"); return;
  }
  const double meters = node_->get_parameter("recovery_backup_m").as_double();
  const double speed = node_->get_parameter("recovery_backup_speed_mps").as_double();
  const double seconds = node_->get_parameter("recovery_backup_timeout_sec").as_double();
  if (!std::isfinite(meters) || meters <= 0.0 || meters > 0.30 ||
      !std::isfinite(speed) || speed <= 0.0 || speed > 0.10 ||
      !std::isfinite(seconds) || seconds < 2.0) {
    fail("Parametros BackUp fuera de limites"); return;
  }

  BackUp::Goal goal;
  goal.target.x = -meters; // Nav2 Humble: negativo = retroceder
  goal.target.y = 0.0;
  goal.target.z = 0.0;
  goal.speed = static_cast<float>(speed);
  goal.time_allowance.sec = static_cast<int32_t>(std::ceil(seconds));
  goal.time_allowance.nanosec = 0;
  backup_handle_.reset(); backup_send_ = {}; backup_result_ = {};
  cancel_pending_ = false; cancellation_reason_.clear();

  rclcpp_action::Client<BackUp>::SendGoalOptions opts;
  const auto logger = node_->get_logger();
  const auto clock = node_->get_clock();
  opts.feedback_callback = [logger, clock](
      BackupHandle::SharedPtr, const std::shared_ptr<const BackUp::Feedback> f) {
      RCLCPP_INFO_THROTTLE(logger, *clock, 1000,
          "RECOVERY: BackUp %.3f m", double(f->distance_traveled));
    };
  backup_send_ = backup_client_->async_send_goal(goal, opts);
  deadline_ = Clock::now() + durationSeconds(seconds + 3.0);
  phase_ = Phase::BACKUP;
  state_ = State::SENDING;
  RCLCPP_INFO(node_->get_logger(), "RECOVERY: solicitando BackUp %.2f m a %.2f m/s",
      meters, speed);
}

void SafeRecovery::startSpin() {
  if (!freshScan() || !footprint_clear_) {
    fail("LiDAR no confirma seguridad para girar"); return;
  }
  if (!spin_client_->action_server_is_ready()) {
    fail("Accion Nav2 /spin no disponible"); return;
  }
  const double degrees = node_->get_parameter("recovery_spin_deg").as_double();
  const double seconds = node_->get_parameter("recovery_spin_timeout_sec").as_double();
  if (!std::isfinite(degrees) || degrees <= 0.0 || degrees > 90.0 ||
      !std::isfinite(seconds) || seconds < 1.0) {
    fail("Parametros Spin invalidos"); return;
  }

  Spin::Goal goal;
  goal.target_yaw = static_cast<float>(last_direction_ * degrees * PI / 180.0);
  goal.time_allowance.sec = static_cast<int32_t>(std::ceil(seconds));
  goal.time_allowance.nanosec = 0;
  spin_handle_.reset(); spin_send_ = {}; spin_result_ = {};
  cancel_pending_ = false; cancellation_reason_.clear();

  rclcpp_action::Client<Spin>::SendGoalOptions opts;
  const auto logger = node_->get_logger();
  const auto clock = node_->get_clock();
  opts.feedback_callback = [logger, clock](
      SpinHandle::SharedPtr, const std::shared_ptr<const Spin::Feedback> f) {
      RCLCPP_INFO_THROTTLE(logger, *clock, 1000,
          "RECOVERY: Spin avance angular %.1f grados",
          double(f->angular_distance_traveled) * 180.0 / PI);
    };
  spin_send_ = spin_client_->async_send_goal(goal, opts);
  deadline_ = Clock::now() + durationSeconds(seconds + 3.0);
  phase_ = Phase::SPIN;
  state_ = State::SENDING;
  RCLCPP_INFO(node_->get_logger(), "RECOVERY: solicitando Spin %.1f grados",
      double(goal.target_yaw) * 180.0 / PI);
}

void SafeRecovery::requestCancel(std::string why) {
  if (!busy() || state_ == State::CANCELING) return;
  cancellation_reason_ = std::move(why);
  state_ = State::CANCELING;
  cancel_pending_ = true;
  deadline_ = Clock::now() + 8s;
  if (phase_ == Phase::BACKUP && backup_handle_) {
    backup_client_->async_cancel_goal(backup_handle_);
  } else if (phase_ == Phase::SPIN && spin_handle_) {
    spin_client_->async_cancel_goal(spin_handle_);
  }
  RCLCPP_WARN(node_->get_logger(), "RECOVERY: solicitando cancelacion: %s",
      cancellation_reason_.c_str());
}

void SafeRecovery::tick() {
  if (!busy()) return;

  // Verificacion continua: si el LiDAR deja de ser reciente o detecta peligro,
  // ordenar cancelacion del comportamiento activo, incluso tras aceptacion.
  if (state_ != State::CANCELING) {
    if (!freshScan()) requestCancel("LiDAR sin datos recientes");
    else if (phase_ == Phase::BACKUP && !rear_clear_)
      requestCancel("Espacio posterior ya no seguro");
    else if (phase_ == Phase::SPIN && !footprint_clear_)
      requestCancel("Radio de giro no despejado");
  }

  if (phase_ == Phase::BACKUP) {
    if (backup_send_.valid() &&
        backup_send_.wait_for(0ms) == std::future_status::ready) {
      backup_handle_ = backup_send_.get(); backup_send_ = {};
      if (!backup_handle_) { fail("Nav2 rechazo BackUp"); return; }
      backup_result_ = backup_client_->async_get_result(backup_handle_);
      if (cancel_pending_) backup_client_->async_cancel_goal(backup_handle_);
      else { state_ = State::ACTIVE;
        RCLCPP_INFO(node_->get_logger(), "RECOVERY: Nav2 ACEPTO BackUp"); }
    }
    if (backup_result_.valid() &&
        backup_result_.wait_for(0ms) == std::future_status::ready) {
      const auto result = backup_result_.get(); backup_result_ = {};
      if (cancel_pending_) { fail(cancellation_reason_); return; }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "RECOVERY: BackUp SUCCEEDED");
        startSpin();
      } else {
        fail("BackUp no finalizo en SUCCEEDED; no se gira");
      }
      return;
    }
  } else if (phase_ == Phase::SPIN) {
    if (spin_send_.valid() &&
        spin_send_.wait_for(0ms) == std::future_status::ready) {
      spin_handle_ = spin_send_.get(); spin_send_ = {};
      if (!spin_handle_) { fail("Nav2 rechazo Spin"); return; }
      spin_result_ = spin_client_->async_get_result(spin_handle_);
      if (cancel_pending_) spin_client_->async_cancel_goal(spin_handle_);
      else { state_ = State::ACTIVE;
        RCLCPP_INFO(node_->get_logger(), "RECOVERY: Nav2 ACEPTO Spin"); }
    }
    if (spin_result_.valid() &&
        spin_result_.wait_for(0ms) == std::future_status::ready) {
      const auto result = spin_result_.get(); spin_result_ = {};
      if (cancel_pending_) { fail(cancellation_reason_); return; }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        state_ = State::SUCCEEDED;
        reason_.clear();
        RCLCPP_INFO(node_->get_logger(), "RECOVERY: Spin SUCCEEDED");
      } else {
        fail("Spin no finalizo en SUCCEEDED");
      }
      return;
    }
  }

  if (Clock::now() >= deadline_) {
    if (state_ == State::CANCELING) {
      fail("Nav2 NO confirmo cancelacion; detener mision y verificar velocidades");
    } else {
      requestCancel("Tiempo de recuperacion excedido");
    }
  }
}

void SafeRecovery::cancel() {
  requestCancel("Cancelacion solicitada por supervisor");
}
