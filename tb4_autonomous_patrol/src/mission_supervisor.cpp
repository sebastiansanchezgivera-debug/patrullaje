#include "tb4_autonomous_patrol/mission_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <limits>
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include <utility>

using namespace std::chrono_literals;

namespace {
constexpr double PI = 3.14159265358979323846;

std::string upper(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return text;
}

bool near(double a, double b) { return std::fabs(a - b) < 1.0e-5; }
}  // namespace

MissionSupervisor::MissionSupervisor(
    const std::string & name, const BT::NodeConfig & config)
    : BT::StatefulActionNode(name, config) {
  node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
  // Aislar las respuestas de Nav2 de los callbacks de LiDAR/estado.
  action_node_ = std::make_shared<rclcpp::Node>("tb4_patrol_nav_client");
  nav_client_ = rclcpp_action::create_client<Action>(
      action_node_, "navigate_to_pose");
  action_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  action_executor_->add_node(action_node_);
  recovery_ = std::make_unique<SafeRecovery>(node_, action_node_);
  dock_ = std::make_unique<DockController>(node_, action_node_);
  action_spin_thread_ = std::thread([this]() {
    action_executor_->spin();
  });
  RCLCPP_INFO(node_->get_logger(),
              "Cliente Nav2 en executor independiente: /navigate_to_pose");
  status_pub_ = node_->create_publisher<std_msgs::msg::String>("/mission/status", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        static_map_ = std::move(msg);
      });

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      node_->get_parameter("laser_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scanCallback(msg); });

  // Odometra: un giro sobre el propio eje no cuenta como progreso de evasión.
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      node_->get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      node_->get_parameter("battery_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) { batteryCallback(msg); });

  command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/mission/command", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { commandCallback(msg); });

  emergency_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/emergency_goal", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { emergencyCallback(msg); });
}

MissionSupervisor::~MissionSupervisor() {
  if (action_executor_) action_executor_->cancel();
  if (action_spin_thread_.joinable()) action_spin_thread_.join();
}

BT::PortsList MissionSupervisor::providedPorts() {
  return {BT::OutputPort<std::string>("state"),
          BT::OutputPort<int>("station")};
}

BT::NodeStatus MissionSupervisor::onStart() {
  markEvent("PATRULLA_INICIADA");
  return onRunning();
}

void MissionSupervisor::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  recovery_->updateScan(msg);
  last_scan_ = Steady::now();
  scan_received_ = true;
  front_valid_ = false;
  front_distance_ = std::numeric_limits<double>::infinity();
  near_wall_classified_ = false;
  front_type_ = "SIN_DATOS";

  const double threshold = node_->get_parameter("stop_distance").as_double();
  const double half_angle =
      node_->get_parameter("front_half_angle_deg").as_double() * PI / 180.0;

  if (!std::isfinite(threshold) || threshold <= 0.0 ||
      !std::isfinite(half_angle) || half_angle <= 0.0 ||
      half_angle > PI || msg->range_max < threshold ||
      msg->angle_increment <= 0.0) {
    return;
  }

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle = msg->angle_min +
        static_cast<double>(i) * msg->angle_increment;
    if (std::fabs(angle) > half_angle) continue;

    const double value = static_cast<double>(msg->ranges[i]);
    if (std::isnan(value) || value < 0.0) continue;
    if (std::isinf(value)) {
      if (value > 0.0) front_valid_ = true;
      continue;
    }
    front_valid_ = true;
    // Valor menor al alcance mínimo: lectura conservadora.
    front_distance_ = std::min(front_distance_, value);
  }
  // SOLO SIMULACION: si todas las detecciones proximas son paredes conocidas,
  // no activar el latch AVOID. Nav2 y Collision Monitor conservan sus barreras.
  const bool only_stage =
      node_->get_parameter("classify_map_walls_stage_only").as_bool() &&
      node_->get_parameter("use_sim_time").as_bool() &&
      node_->get_parameter("simulate_docking").as_bool();
  if (front_distance_ > threshold) {
    front_type_ = "DESPEJADO";
  } else if (!only_stage) {
    front_type_ = "OBSTACULO_SIN_CLASIFICAR";
  } else {
    near_wall_classified_ = scanNearPointsMatchMap(msg, threshold, half_angle);
    front_type_ = near_wall_classified_ ? "PARED_MAPA" : "NUEVO_O_INCIERTO";
  }
}

// Compara cada eco frontal cercano con el PRIMER obstaculo del mapa a lo
// largo del mismo rayo. Si faltan datos, TF, mapa, o hay discrepancia,
// clasifica como objeto nuevo/incierto: parada conservadora.
bool MissionSupervisor::scanNearPointsMatchMap(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    double threshold, double half_angle) {
  if (!static_map_ || !tf_buffer_) return false;
  const auto & grid = *static_map_;
  const auto & info = grid.info;
  if (info.resolution <= 0.0 || info.width == 0 || info.height == 0 ||
      grid.data.size() != static_cast<size_t>(info.width) * info.height) return false;

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  const auto & q = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                1.0 - 2.0 * (q.y*q.y + q.z*q.z));
  const auto & o = info.origin.orientation;
  const double oyaw = std::atan2(2.0 * (o.w*o.z + o.x*o.y),
                                 1.0 - 2.0 * (o.y*o.y + o.z*o.z));
  const double co = std::cos(oyaw), so = std::sin(oyaw);
  const double res = static_cast<double>(info.resolution);
  const double tolerance = std::clamp(
      node_->get_parameter("map_wall_match_tolerance_m").as_double(),
      0.05, 0.20);
  const double lx = tf.transform.translation.x;
  const double ly = tf.transform.translation.y;
  // Raycast sub-celda; valor desconocido del mapa = clasificacion insegura.
  auto occupied = [&](double wx, double wy) -> int {
    const double dx=wx-info.origin.position.x;
    const double dy=wy-info.origin.position.y;
    const int ix=static_cast<int>(std::floor((co*dx + so*dy)/res));
    const int iy=static_cast<int>(std::floor((-so*dx + co*dy)/res));
    if (ix<0 || iy<0 || ix>=static_cast<int>(info.width) ||
        iy>=static_cast<int>(info.height)) return -1;
    const int val=grid.data[static_cast<size_t>(iy)*info.width + ix];
    if (val<0) return -1;
    return val>=65 ? 1 : 0;
  };
  bool any_near = false;
  for (size_t i=0; i<scan->ranges.size(); ++i) {
    const double a=scan->angle_min + i*scan->angle_increment;
    if (std::abs(a)>half_angle) continue;
    const double d=scan->ranges[i];
    if (!std::isfinite(d) || d>threshold) continue;
    any_near=true;
    if (d<scan->range_min || d<=0.0) return false;
    const double angle=yaw+a;
    const double ca=std::cos(angle), sa=std::sin(angle);
    double expected=std::numeric_limits<double>::infinity();
    const double step=std::max(0.01, res*0.5);
    for (double s=0.0; s<=d+tolerance+step; s+=step) {
      const int status=occupied(lx+s*ca, ly+s*sa);
      if (status<0) return false;
      if (status>0) {expected=s; break;}
    }
    if (!std::isfinite(expected) || std::abs(expected-d)>tolerance) return false;
  }
  return any_near;
}

void MissionSupervisor::batteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  const double fraction = msg->percentage;
  if (std::isfinite(fraction) && fraction >= 0.0 && fraction <= 1.0) {
    battery_real_pct_ = fraction * 100.0;
    battery_received_ = true;
    last_battery_ = Steady::now();
  }
}

bool MissionSupervisor::laserFresh() const {
  if (!scan_received_ || !front_valid_) return false;
  const double age = std::chrono::duration<double>(
      Steady::now() - last_scan_).count();
  const double limit = node_->get_parameter("laser_timeout_sec").as_double();
  return std::isfinite(limit) && limit > 0.0 && age <= limit;
}

void MissionSupervisor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  odom_x_ = x;
  odom_y_ = y;
  odom_valid_ = true;
}

void MissionSupervisor::resetAvoidCycles() {
  avoid_cycles_without_progress_ = 0;
  avoid_loop_hold_ = false;
  avoid_anchor_valid_ = false;
}

void MissionSupervisor::updateAvoidProgress() {
  if (!odom_valid_ || !avoid_anchor_valid_) return;
  if (avoid_cycles_without_progress_ == 0 && !avoid_loop_hold_) return;
  const double d = std::hypot(odom_x_ - avoid_anchor_x_,
                              odom_y_ - avoid_anchor_y_);
  const double threshold =
      node_->get_parameter("avoid_progress_reset_distance_m").as_double();
  if (!std::isfinite(threshold) || threshold <= 0.0) return;
  if (d >= threshold) {
    resetAvoidCycles();
    if (avoiding_) avoiding_ = false;
    markEvent("EVASION_PROGRESO_ODOMETRIA_CONFIRMADO");
  }
}

bool MissionSupervisor::batteryFresh() const {
  if (node_->get_parameter("simulate_battery").as_bool()) return true;
  if (!battery_received_) return false;
  const double age = std::chrono::duration<double>(
      Steady::now() - last_battery_).count();
  const double limit = node_->get_parameter("battery_timeout_sec").as_double();
  return std::isfinite(limit) && limit > 0.0 && age <= limit;
}

void MissionSupervisor::updateBatteryLatch() {
  if (node_->get_parameter("simulate_battery").as_bool()) {
    battery_pct_ = node_->get_parameter("sim_battery_pct").as_double();
  } else {
    battery_pct_ = battery_real_pct_;
  }
  if (batteryFresh() && std::isfinite(battery_pct_) &&
      battery_pct_ < node_->get_parameter("battery_threshold_pct").as_double() &&
      !battery_hold_) {
    battery_hold_ = true;
    // Si estaba desacoplando, suspender la salida y volver a planificar HOME.
    if (dock_->state() == DockController::State::UNDOCKING) {
      dock_->cancel();
      home_arrived_ = false;
    }
    emergency_active_ = false;
    emergency_arrived_ = false;
    station_waiting_ = false;
    avoiding_ = false;
    recovery_requested_ = false;
    recovery_->cancel();
    markEvent("BATERIA_BAJA_RETORNO_CARGA");
  }
}

bool MissionSupervisor::obstaclePresent() const {
  if (!laserFresh()) return false;
  if (front_distance_ > node_->get_parameter("stop_distance").as_double()) return false;
  const bool only_stage =
      node_->get_parameter("classify_map_walls_stage_only").as_bool() &&
      node_->get_parameter("use_sim_time").as_bool() &&
      node_->get_parameter("simulate_docking").as_bool();
  return !only_stage || !near_wall_classified_;
}

void MissionSupervisor::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
  const std::string command = upper(msg->data);
  if (canceled_ || !fault_.empty()) return;

  if (command == "STOP" || command == "PAUSE") {
    pause_requested_ = true;
    emergency_active_ = false;
    emergency_arrived_ = false;
    station_waiting_ = false;
    avoiding_ = false;
    recovery_requested_ = false;
    recovery_->cancel();
    if (dock_->state() == DockController::State::UNDOCKING) {
      dock_->cancel();
      home_arrived_ = false;
    }
    markEvent("PARADA_OPERADOR_REGRESAR_CARGA");
  } else if (command == "RESUME") {
    // No aceptar RESUME durante docking: podría dejarlo acoplado y
    // reanudar el patrullaje sin haber ejecutado UNDOCKING.
    if (dock_->busy()) {
      markEvent("RESUME_RECHAZADO_ACOPLAMIENTO_EN_PROGRESO");
      return;
    }
    const double needed = node_->get_parameter("battery_resume_pct").as_double();
    const double minimum = battery_hold_ ? needed :
        node_->get_parameter("battery_threshold_pct").as_double();
    if (!batteryFresh() || !std::isfinite(battery_pct_) ||
        battery_pct_ < minimum) {
      markEvent("RESUME_RECHAZADO_BATERIA_O_DATOS");
      return;
    }
    battery_hold_ = false;
    pause_requested_ = false;
    // HOME solo queda libre tras verificar el undocking.
    if (dock_->docked()) {
      dock_->startUndock();
      markEvent("SALIDA_DE_CARGA_INICIADA");
    } else if (dock_->idle()) {
      home_arrived_ = false;
    }
    markEvent("PATRULLA_REANUDADA");
  } else if (command == "CANCEL") {
    canceled_ = true;
    avoiding_ = false;
    recovery_requested_ = false;
    recovery_->cancel();
    dock_->cancel();
    markEvent("CANCELACION_DEFINITIVA");
  } else if (command == "AVOID") {
    if (avoid_loop_hold_) {
      markEvent("AVOID_BLOQUEADO_SIN_PROGRESO_REQUIERE_INTERVENCION");
    } else if (laserFresh() && obstaclePresent() && !avoiding_ &&
               !recovery_requested_ && !recovery_->busy() &&
               !battery_hold_ && !pause_requested_ &&
               !dock_->busy() && !dock_->docked()) {
      const int max_cycles = std::clamp(static_cast<int>(
          node_->get_parameter("avoid_max_cycles_without_progress").as_int()), 1, 5);
      if (avoid_cycles_without_progress_ >= max_cycles) {
        avoid_loop_hold_ = true;
        avoiding_ = false;
        recovery_requested_ = false;
        markEvent("EVASION_REPETIDA_SIN_AVANCE_ESPERAR_OPERADOR");
        return;
      }
      // Unicamente el desplazamiento de odometria o la llegada al objetivo
      // reinicia este contador; AVOID no lo vuelve a poner a cero.
      if (!avoid_anchor_valid_ && odom_valid_) {
        avoid_anchor_valid_ = true;
        avoid_anchor_x_ = odom_x_;
        avoid_anchor_y_ = odom_y_;
      }
      ++avoid_cycles_without_progress_;
      recovery_requested_ = true;
      recovery_spin_count_ = 0;
      recovery_direction_ = 0;
      avoiding_ = false;
      markEvent("RECOVERY_SOLICITADA_CANCELAR_NAV2");
    } else {
      markEvent("AVOID_NO_ADMITIDO_ESTADO_ACTUAL");
    }
  } else if (command == "RESET_AVOID") {
    // Rearme manual solo con el frente libre y LiDAR reciente.
    if (!laserFresh() || obstaclePresent()) {
      markEvent("RESET_AVOID_RECHAZADO_FRENTE_NO_LIBRE");
      return;
    }
    resetAvoidCycles();
    markEvent("EVASION_REARMADA_POR_OPERADOR");
  } else if (command == "WAIT") {
    recovery_requested_ = false;
    avoiding_ = false;
    recovery_->cancel();
    markEvent("MODO_ESPERAR");
  } else if (command == "EMERGENCY_CANCEL" || command == "CLEAR_EMERGENCY") {
    if (emergency_active_) {
      emergency_active_ = false;
      emergency_arrived_ = false;
      station_waiting_ = false;
      markEvent("EMERGENCIA_CANCELADA_REANUDAR_PATRULLA");
    }
  } else {
    markEvent("COMANDO_DESCONOCIDO_" + command);
  }
}

void MissionSupervisor::emergencyCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (canceled_ || !fault_.empty() || pause_requested_ || battery_hold_ ||
      dock_->busy() || dock_->docked()) {
    markEvent("EMERGENCIA_NO_ADMITIDA_EN_ESTADO_ACTUAL");
    return;
  }
  if (msg->header.frame_id != "map" ||
      !std::isfinite(msg->pose.position.x) ||
      !std::isfinite(msg->pose.position.y)) {
    markEvent("EMERGENCIA_INVALIDA_FRAME_MAP_REQUERIDO");
    return;
  }
  emergency_pose_ = *msg;
  emergency_active_ = true;
  emergency_arrived_ = false;
  station_waiting_ = false;
  avoiding_ = false;
  recovery_requested_ = false;
  recovery_->cancel();
  markEvent("EMERGENCIA_INICIADA");
}

MissionSupervisor::Target MissionSupervisor::makeTarget(
    GoalKind kind, int station) const {
  Target goal;
  goal.kind = kind;
  goal.station = station;
  goal.pose.header.frame_id = "map";

  if (kind == GoalKind::EMERGENCY) {
    goal.pose = emergency_pose_;
    goal.valid = (emergency_pose_.header.frame_id == "map");
    return goal;
  }
  if (kind == GoalKind::NONE) return goal;

  std::string prefix = kind == GoalKind::HOME ? "home" :
      "station_" + std::to_string(station + 1);
  const double x = node_->get_parameter(prefix + "_x").as_double();
  const double y = node_->get_parameter(prefix + "_y").as_double();
  const double yaw = node_->get_parameter(prefix + "_yaw").as_double();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      x == -9999.0 || y == -9999.0) return goal;

  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  goal.valid = true;
  return goal;
}

MissionSupervisor::Target MissionSupervisor::requestedTarget() {
  Target no_target;
  // Sin sensor fresco o durante cualquier accion de recuperacion/carga,
  // nunca emitir nuevos objetivos Nav2.
  if (!laserFresh() || !batteryFresh() || recovery_requested_ ||
      recovery_->busy() || dock_->busy() || dock_->docked()) return no_target;
  if (home_arrived_ && dock_->idle() && !pause_requested_ && !battery_hold_) {
    home_arrived_ = false;  // undocking confirmado
  }

  // Al girar el frente queda libre temporalmente, pero eso no implica
  // haber rodeado el obstaculo. Mantener la fase de replanteamiento
  // hasta observar desplazamiento real o superar el plazo.
  if (avoid_loop_hold_) return no_target;
  const bool blocked = obstaclePresent();
  if (avoiding_ && Steady::now() >= avoid_deadline_) {
    avoiding_ = false;
    markEvent("EVASION_TIEMPO_AGOTADO_VOLVER_A_ESPERAR");
  }
  // Regla de seguridad invariable: con obstaculo <= stop_distance
  // se cancela Nav2, incluso en fase de evasion.
  if (blocked) {
    if (avoiding_) {
      avoiding_ = false;
      markEvent("EVASION_REENCUENTRA_OBSTACULO_PARAR");
    }
    return no_target;
  }

  if (battery_hold_ || pause_requested_) {
    if (home_arrived_) return no_target;
    auto target = makeTarget(GoalKind::HOME);
    if (!target.valid) setFault("HOME_NO_CONFIGURADO");
    return target;
  }
  if (emergency_active_) {
    if (emergency_arrived_) return no_target;
    return makeTarget(GoalKind::EMERGENCY);
  }
  if (station_waiting_) {
    if (Steady::now() < station_wait_deadline_) return no_target;
    station_waiting_ = false;
    station_index_ = (station_index_ + 1) % 3;
    markEvent("SIGUIENTE_ESTACION");
  }
  return makeTarget(GoalKind::STATION, station_index_);
}

bool MissionSupervisor::sameTarget(const Target & a, const Target & b) const {
  if (a.kind != b.kind || a.station != b.station || a.valid != b.valid) return false;
  if (!a.valid) return true;
  const auto & p = a.pose.pose;
  const auto & q = b.pose.pose;
  return near(p.position.x, q.position.x) &&
      near(p.position.y, q.position.y) &&
      near(p.orientation.z, q.orientation.z) &&
      near(p.orientation.w, q.orientation.w);
}

void MissionSupervisor::clearGoal() {
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};
  active_target_ = Target{};
  nav_phase_ = NavPhase::IDLE;
  cancellation_sent_ = false;
}

void MissionSupervisor::onGoalCompleted(const Target & goal, bool succeeded) {
  if (succeeded) resetAvoidCycles();
  if (!succeeded) {
    if (avoiding_ && obstaclePresent()) {
      avoiding_ = false;
      markEvent("EVASION_FALLIDA_VOLVER_A_ESPERAR");
    } else {
      setFault("NAV2_NO_ALCANZO_DESTINO");
    }
    return;
  }

  if (goal.kind == GoalKind::STATION) {
    station_waiting_ = true;
    const std::string name = "station_" + std::to_string(goal.station + 1) + "_wait_sec";
    const double seconds = node_->get_parameter(name).as_double();
    station_wait_deadline_ = Steady::now() +
        std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(
            std::max(0.0, seconds)));
    markEvent("ESTACION_" + std::to_string(goal.station + 1) + "_ALCANZADA");
  } else if (goal.kind == GoalKind::HOME) {
    home_arrived_ = true;
    dock_->startDock();
    markEvent("DOCKING_INICIADO");
  } else if (goal.kind == GoalKind::EMERGENCY) {
    emergency_arrived_ = true;
    markEvent("PUNTO_EMERGENCIA_ALCANZADO_ESPERANDO_CANCELACION");
  }
}

void MissionSupervisor::advanceNavigation(const Target & wanted) {
  // Primero procesar las respuestas de Nav2, incluso si se cambió de prioridad.
  if (nav_phase_ == NavPhase::SENDING) {
    if (goal_future_.valid() && goal_future_.wait_for(0ms) == std::future_status::ready) {
      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        if (sameTarget(wanted, active_target_)) {
          setFault("NAV2_RECHAZO_OBJETIVO");
        }
        clearGoal();
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "NAV2_ACEPTO_OBJETIVO");
      result_future_ = nav_client_->async_get_result(goal_handle_);
      nav_phase_ = NavPhase::ACTIVE;
    } else if (Steady::now() >= phase_deadline_) {
      // No sabemos si Nav2 aceptará más tarde: no emitimos otro objetivo.
      setFault("TIMEOUT_ACEPTACION_OBJETIVO_REQUIERE_INSPECCION");
      return;
    } else {
      return;
    }
  }

  if (nav_phase_ == NavPhase::ACTIVE || nav_phase_ == NavPhase::CANCELING) {
    const bool changed = canceled_ || !fault_.empty() ||
        !sameTarget(wanted, active_target_);

    if (result_future_.valid() && result_future_.wait_for(0ms) == std::future_status::ready) {
      auto result = result_future_.get();
      RCLCPP_INFO(node_->get_logger(), "NAV2_RESULTADO_CODIGO=%d",
                  static_cast<int>(result.code));
      const Target finished = active_target_;
      const bool interrupted = changed || cancellation_sent_;
      clearGoal();
      if (!interrupted) {
        onGoalCompleted(finished, result.code == rclcpp_action::ResultCode::SUCCEEDED);
      }
      return;
    }

    if (changed && !cancellation_sent_) {
      nav_client_->async_cancel_goal(goal_handle_);
      cancellation_sent_ = true;
      nav_phase_ = NavPhase::CANCELING;
      phase_deadline_ = Steady::now() + 8s;
      markEvent("CANCELANDO_OBJETIVO_NAV2");
    }
    if (nav_phase_ == NavPhase::CANCELING && Steady::now() >= phase_deadline_) {
      setFault("TIMEOUT_CANCELACION_NAV2_REQUIERE_INTERVENCION");
    }
    return;
  }

  if (nav_phase_ != NavPhase::IDLE || canceled_ || !fault_.empty() || !wanted.valid) return;
  if (!nav_client_->action_server_is_ready()) return;

  Action::Goal nav_goal;
  nav_goal.pose = wanted.pose;
  nav_goal.pose.header.stamp = node_->now();
  active_target_ = wanted;
  goal_future_ = nav_client_->async_send_goal(nav_goal);
  nav_phase_ = NavPhase::SENDING;
  phase_deadline_ = Steady::now() + 8s;
  if (wanted.kind != GoalKind::HOME) home_arrived_ = false;
  markEvent(wanted.kind == GoalKind::HOME ? "NAVEGANDO_A_CARGA" :
            wanted.kind == GoalKind::EMERGENCY ? "NAVEGANDO_A_EMERGENCIA" :
            "NAVEGANDO_ESTACION_" + std::to_string(wanted.station + 1));
}

void MissionSupervisor::advanceRecovery() {
  // Prioridad de cancelacion: nunca enviar /spin mientras exista objetivo
  // activo de NavigateToPose.
  recovery_->tick();
  if (!recovery_requested_) return;
  if (nav_phase_ != NavPhase::IDLE) return;
  // Se explora en segmentos de giro sin traslacion, sin abandonar
  // la estacion original. No se publica cmd_vel desde el supervisor.
  if (recovery_->state() == SafeRecovery::State::IDLE) {
    recovery_->start(recovery_direction_, recovery_spin_count_ == 0);
    if (recovery_->busy()) {
      recovery_direction_ = recovery_->direction();
      markEvent("RECOVERY_RETROCESO_OPCIONAL_Y_GIRO_" + std::to_string(recovery_spin_count_ + 1) +
                "_EVALUANDO_ENTORNO");
    }
  }
  if (recovery_->state() == SafeRecovery::State::SUCCEEDED) {
    ++recovery_spin_count_;
    recovery_->reset();
    // No interpretar un LiDAR sin datos como frente despejado.
    if (!laserFresh()) {
      recovery_requested_ = false;
      avoiding_ = false;
      markEvent("RECOVERY_LIDAR_NO_VALIDO_VOLVER_A_ESPERAR");
      return;
    }
    if (obstaclePresent()) {
      const int max_spins = node_->get_parameter("recovery_max_spins").as_int();
      if (recovery_spin_count_ >= std::clamp(max_spins, 1, 5)) {
        recovery_requested_ = false;
        avoiding_ = false;
        markEvent("RECOVERY_SIN_SALIDA_VOLVER_A_ESPERAR");
      } else {
        markEvent("RECOVERY_GIRO_" + std::to_string(recovery_spin_count_) +
                  "_FRENTE_BLOQUEADO_CONTINUAR_EXPLORACION");
      }
      return;
    }
    recovery_requested_ = false;
    avoiding_ = true;
    const double seconds = node_->get_parameter("avoid_attempt_timeout_sec").as_double();
    avoid_deadline_ = Steady::now() +
        std::chrono::duration_cast<Steady::duration>(
            std::chrono::duration<double>(std::max(1.0, seconds)));
    markEvent("RECOVERY_FRENTE_LIBRE_REPLANIFICAR_MISMA_ESTACION");
  } else if (recovery_->state() == SafeRecovery::State::FAILED) {
    recovery_requested_ = false;
    avoiding_ = false;
    const auto reason = recovery_->reason();
    recovery_->reset();
    RCLCPP_WARN(node_->get_logger(), "RECOVERY NO EJECUTADA: %s", reason.c_str());
    markEvent("RECOVERY_NO_SEGURA_VOLVER_A_ESPERAR");
  }
}

void MissionSupervisor::advanceDocking() {
  dock_->tick();
  const auto state = dock_->state();
  if (state == DockController::State::FAILED) {
    setFault("DOCKING_FALLO_" + dock_->error());
    return;
  }
  if (state != last_dock_state_) {
    last_dock_state_ = state;
    if (state == DockController::State::DOCKED) markEvent("DOCKED_VERIFICADO");
    else if (state == DockController::State::IDLE && home_arrived_ &&
             !battery_hold_ && !pause_requested_) {
      home_arrived_ = false;
      markEvent("UNDOCKING_COMPLETADO_REANUDAR_PATRULLA");
    } else if (state == DockController::State::UNDOCKING) markEvent("UNDOCKING_INICIADO");
  }
}

void MissionSupervisor::setFault(const std::string & why) {
  if (fault_.empty()) {
    fault_ = why;
    RCLCPP_ERROR(node_->get_logger(), "FALLO SUPERVISOR: %s", why.c_str());
  }
}

void MissionSupervisor::markEvent(const std::string & message) {
  event_ = message;
  RCLCPP_INFO(node_->get_logger(), "EVENTO: %s", message.c_str());
  publishStatus(true);
}

std::string MissionSupervisor::statusName() const {
  if (!fault_.empty()) return "FAULT";
  if (canceled_) return "CANCELLING";
  if (!laserFresh()) return "LIDAR_NO_VALIDO";
  if (!batteryFresh()) return "BATERIA_SIN_DATOS";
  if (recovery_requested_ || recovery_->busy()) return "RECOVERY_SPIN";
  if (dock_->state() == DockController::State::DOCKING) return "DOCKING";
  if (dock_->state() == DockController::State::UNDOCKING) return "UNDOCKING";
  if (dock_->docked()) return "DOCKED";
  if (avoid_loop_hold_) return "OBSTACLE_WAIT";
  if (obstaclePresent()) return "OBSTACLE_WAIT";
  if (avoiding_) return "AVOID_REQUESTED";
  if ((battery_hold_ || pause_requested_) && home_arrived_) return "STANDBY_HOME";
  if (battery_hold_ || pause_requested_) return "RETURNING_HOME";
  if (emergency_active_ && emergency_arrived_) return "EMERGENCY_ON_SITE";
  if (emergency_active_) return "EMERGENCY_NAV";
  if (station_waiting_) return "STATION_WAIT";
  return "PATROLLING";
}

void MissionSupervisor::publishStatus(bool force) {
  const auto now = Steady::now();
  if (!force && last_status_ != Steady::time_point{} &&
      now - last_status_ < 500ms) return;
  last_status_ = now;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2)
     << "state=" << statusName()
     << " station=" << station_index_ + 1
     << " front_m=" << front_distance_
     << " front_type=" << front_type_
     << " battery_pct=" << battery_pct_
     << " nav=" << static_cast<int>(nav_phase_)
     << " battery_hold=" << battery_hold_
     << " dock=" << DockController::name(dock_->state())
     << " avoid_cycles=" << avoid_cycles_without_progress_
     << " avoid_hold=" << avoid_loop_hold_
     << " event=" << event_;
  if (!fault_.empty()) ss << " error=" << fault_;
  std_msgs::msg::String message;
  message.data = ss.str();
  status_pub_->publish(message);
  setOutput("state", statusName());
  setOutput("station", station_index_ + 1);
}

BT::NodeStatus MissionSupervisor::onRunning() {
  updateAvoidProgress();
  updateBatteryLatch();
  advanceDocking();

  Target wanted;
  if (!canceled_ && fault_.empty()) wanted = requestedTarget();
  if (!wanted.valid && !canceled_ && fault_.empty() &&
      laserFresh() && batteryFresh() && !obstaclePresent() &&
      !home_arrived_ && !station_waiting_ && !emergency_arrived_ &&
      !pause_requested_ && !battery_hold_ && !recovery_requested_ &&
      !avoid_loop_hold_ &&
      !recovery_->busy() && !dock_->busy() && !dock_->docked()) {
    setFault("COORDENADAS_OBJETIVO_INVALIDAS");
  }

  advanceNavigation(wanted);
  advanceRecovery();
  publishStatus();

  if (nav_phase_ == NavPhase::IDLE && canceled_ &&
      !recovery_->busy() && !dock_->busy()) {
    markEvent("CANCELADA");
    return BT::NodeStatus::SUCCESS;
  }
  // Si falla Nav2, conservar el nodo vivo hasta que la meta activa haya
  // terminado o confirmado su cancelacion. Nunca finalizar el ejecutable
  // dejando una accion aceptada por Nav2 sin gestionar.
  if (!fault_.empty()) {
    return (nav_phase_ == NavPhase::IDLE && !dock_->busy() &&
            !recovery_->busy()) ? BT::NodeStatus::FAILURE
                                      : BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}

void MissionSupervisor::onHalted() {
  recovery_->cancel();
  dock_->cancel();
  if (nav_phase_ == NavPhase::ACTIVE && goal_handle_) {
    nav_client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(node_->get_logger(), "Cancelacion solicitada al detener arbol");
  } else if (nav_phase_ == NavPhase::SENDING) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Arbol detenido durante aceptacion del objetivo; puede quedar accion pendiente en Nav2");
  }
}
