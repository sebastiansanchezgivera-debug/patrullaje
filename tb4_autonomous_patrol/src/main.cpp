#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tb4_autonomous_patrol/mission_supervisor.hpp"
#include "tb4_autonomous_patrol/mission_state_indicator.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tb4_autonomous_patrol");

  const double xs[3] = {0.28907108306884766, 19.907833099365234,
                        11.876189231872559};
  const double ys[3] = {0.05821871757507324, -0.03801417350769043,
                        -7.791070938110352};
  for (int i = 1; i <= 3; ++i) {
    const auto p = "station_" + std::to_string(i);
    node->declare_parameter<double>(p + "_x", xs[i - 1]);
    node->declare_parameter<double>(p + "_y", ys[i - 1]);
    node->declare_parameter<double>(p + "_yaw", 0.0);
    node->declare_parameter<double>(p + "_wait_sec", 2.0);
  }

  // HOME no tiene coordenadas inventadas: configurarlo con Publish Point de RViz.
  node->declare_parameter<double>("home_x", -9999.0);
  node->declare_parameter<double>("home_y", -9999.0);
  node->declare_parameter<double>("home_yaw", 0.0);

  node->declare_parameter<std::string>("laser_topic", "/base_scan");
  node->declare_parameter<double>("stop_distance", 0.45);
  // Solo Stage: el mapa estático distingue paredes de objetos no cartografiados.
  // En hardware permanece deshabilitado por defecto; Collision Monitor intacto.
  node->declare_parameter<bool>("classify_map_walls_stage_only", false);
  node->declare_parameter<double>("map_wall_match_tolerance_m", 0.12);
  node->declare_parameter<double>("front_half_angle_deg", 15.0);
  // Gazebo: sector frontal referido al eje de base_link.
  node->declare_parameter<bool>("front_scan_in_base_frame", false);
  node->declare_parameter<double>("laser_timeout_sec", 0.7);
  node->declare_parameter<double>("avoid_attempt_timeout_sec", 15.0);
  node->declare_parameter<std::string>("odom_topic", "/odom");
  node->declare_parameter<int>("avoid_max_cycles_without_progress", 2);
  node->declare_parameter<double>("avoid_progress_reset_distance_m", 0.60);

  // Recuperacion Nav2 Spin: nunca puentea Collision Monitor.
  node->declare_parameter<bool>("enable_spin_recovery", true);
  // BackUp previo a Spin solo con cobertura LiDAR trasera completa.
  node->declare_parameter<bool>("enable_backup_recovery", false);
  node->declare_parameter<double>("recovery_backup_m", 0.15);
  node->declare_parameter<double>("recovery_backup_speed_mps", 0.05);
  node->declare_parameter<double>("recovery_backup_timeout_sec", 10.0);
  node->declare_parameter<double>("recovery_spin_deg", 35.0);
  node->declare_parameter<int>("recovery_max_spins", 3);
  node->declare_parameter<double>("recovery_spin_timeout_sec", 10.0);
  node->declare_parameter<double>("robot_radius_m", 0.22);
  node->declare_parameter<double>("recovery_margin_m", 0.06);
  node->declare_parameter<double>("recovery_laser_x_m", 0.15);
  node->declare_parameter<bool>("allow_partial_scan_sim_only", false);

  // Carga: Stage usa tiempos simulados; en hardware se requiere confirmacion.
  node->declare_parameter<bool>("simulate_docking", true);
  node->declare_parameter<bool>("enable_real_docking", false);
  node->declare_parameter<std::string>("dock_action_name", "/dock");
  node->declare_parameter<std::string>("undock_action_name", "/undock");
  node->declare_parameter<std::string>("dock_status_topic", "/dock_status");
  node->declare_parameter<double>("sim_dock_duration_sec", 3.0);
  node->declare_parameter<double>("sim_undock_duration_sec", 3.0);
  node->declare_parameter<double>("dock_timeout_sec", 65.0);
  node->declare_parameter<double>("undock_timeout_sec", 50.0);

  node->declare_parameter<std::string>("battery_topic", "/battery_state");
  node->declare_parameter<bool>("simulate_battery", true);
  node->declare_parameter<double>("sim_battery_pct", 100.0);
  node->declare_parameter<double>("battery_threshold_pct", 20.0);
  node->declare_parameter<double>("battery_resume_pct", 25.0);
  node->declare_parameter<double>("battery_timeout_sec", 5.0);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MissionSupervisor>("MissionSupervisor");
  factory.registerNodeType<MissionStateIndicator>("MissionStateIndicator");

  const std::string xml =
      ament_index_cpp::get_package_share_directory("tb4_autonomous_patrol") +
      "/behavior_trees/mission.xml";

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml, blackboard);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Error XML: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "MissionSupervisor iniciado; Groot2 puerto 1667");
  RCLCPP_INFO(node->get_logger(),
              "Control: /mission/command (STOP, RESUME, CANCEL, AVOID, RESET_AVOID, WAIT, EMERGENCY_CANCEL)");
  RCLCPP_INFO(node->get_logger(), "Estado: /mission/status");

  // Executor persistente: procesa respuestas y resultados de Nav2.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  {
    BT::Groot2Publisher groot(tree, 1667);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      executor.spin_some();
      status = tree.tickExactlyOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (status == BT::NodeStatus::RUNNING) {
      tree.haltTree();
    }
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Mision cancelada definitivamente");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Supervisor termino con error: revisar estado");
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
