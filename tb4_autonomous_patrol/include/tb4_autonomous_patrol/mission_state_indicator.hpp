#ifndef TB4_AUTONOMOUS_PATROL_MISSION_STATE_INDICATOR_HPP_
#define TB4_AUTONOMOUS_PATROL_MISSION_STATE_INDICATOR_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

// Nodo de visualizacion: NO controla motores ni seguridad.
// Devuelve RUNNING cuando el supervisor esta en el estado indicado.
class MissionStateIndicator : public BT::StatefulActionNode {
public:
  MissionStateIndicator(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("state"),
            BT::InputPort<std::string>("equals")};
  }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    std::string actual, expected;
    if (!getInput("state", actual) || !getInput("equals", expected)) {
      return BT::NodeStatus::FAILURE;
    }
    return (expected == "*" || actual == expected)
        ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
};
#endif
