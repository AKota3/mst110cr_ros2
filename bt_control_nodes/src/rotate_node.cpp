#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <com3_msgs/msg/joint_cmd.hpp>

namespace BT
{

class RotateNode : public StatefulActionNode
{
public:
  RotateNode(const std::string & name, const NodeConfiguration & config)
    : StatefulActionNode(name, config)
    , node_{rclcpp::Node::make_shared("rotate_node")}
    , target_angle_{1.0}
    , tolerance_{0.1}
    , current_angle_{0.0}
  {
    // publisher / subscriber
    pub_ = node_->create_publisher<com3_msgs::msg::JointCmd>(
      "/mst110cr/front_cmd", rclcpp::QoS(10));
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/mst110cr/joint_state", rclcpp::QoS(10),
      std::bind(&RotateNode::jointStateCallback, this,
                std::placeholders::_1));

    setRegistrationID("RotateNode");
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<double>("target_angle", 0.0, "目標 swing_joint 角度 [rad]"),
      InputPort<double>("tolerance",   0.1, "到達判定の許容誤差 [rad]")
    };
  }

  NodeStatus onStart() override
  {
    getInput("target_angle", target_angle_);
    getInput("tolerance",   tolerance_);
    RCLCPP_INFO(node_->get_logger(),
                "RotateNode: target_angle=%.3f, tolerance=%.3f",
                target_angle_, tolerance_);
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    if (!last_msg_) {
      rclcpp::spin_some(node_);
      return NodeStatus::RUNNING;
    }

    const auto & msg = *last_msg_;
    com3_msgs::msg::JointCmd out;
    out.joint_name = msg.name;

    size_t n = msg.name.size();
    out.position.resize(n);
    out.velocity = msg.velocity;
    out.effort   = msg.effort;

    for (size_t i = 0; i < n; ++i) {
      if (msg.name[i] == "swing_joint") {
        out.position[i] = target_angle_;
      } else if (i < msg.position.size()) {
        out.position[i] = msg.position[i];
      } else {
        out.position[i] = 0.0;
      }
    }

    pub_->publish(out);

    if (std::fabs(current_angle_ - target_angle_) <= tolerance_) {
      RCLCPP_INFO(node_->get_logger(),
                  "swing_joint reached %.3f (target %.3f)",
                  current_angle_, target_angle_);
      return NodeStatus::SUCCESS;
    }

    rclcpp::spin_some(node_);
    return NodeStatus::RUNNING;
  }

  void onHalted() override
  {
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_msg_ = msg;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "swing_joint" && i < msg->position.size()) {
        current_angle_ = msg->position[i];
        break;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  double target_angle_;
  double tolerance_;
  double current_angle_;
};

} 


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::RotateNode>("RotateNode");
}