#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <com3_msgs/msg/joint_cmd.hpp>
#include <cmath>

namespace BT
{

class RotateFwdBwdAlongTrackNode : public StatefulActionNode
{
public:
  RotateFwdBwdAlongTrackNode(const std::string & name, const NodeConfiguration & config)
    : StatefulActionNode(name, config)
    , node_{rclcpp::Node::make_shared("rotate_fwdbwdalongtrack_node")}
    , tolerance_{0.1}
    , current_angle_(0.0)
    , target_selected_(false)
    , target_angle_(0.0)
  {
    pub_ = node_->create_publisher<com3_msgs::msg::JointCmd>(
      "/mst110cr/front_cmd", rclcpp::QoS(10));
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/mst110cr/joint_state", rclcpp::QoS(10),
      std::bind(&RotateFwdBwdAlongTrackNode::jointStateCallback, this,
                std::placeholders::_1));

    setRegistrationID("RotateFwdBwdAlongTrackNode");
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<double>("tolerance",   0.1, "到達判定の許容誤差 [rad]")
    };
  }

  NodeStatus onStart() override
  {
    // tolerance はポートから読み込む
    getInput("tolerance", tolerance_);
    RCLCPP_INFO(node_->get_logger(),
                "RotateFwdBwdAlongTrackNode: tolerance=%.3f", tolerance_);
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    // joint_state がまだ来ていなければ待つ
    if (!last_msg_) {
      rclcpp::spin_some(node_);
      return NodeStatus::RUNNING;
    }

    // ① ターゲット角度を一度だけ決定
    if (!target_selected_) {
      // 現在角度 current_angle_ が 0 ～ 2π の範囲と仮定
      double angle0 = 0.0;
      double angle180 = M_PI;
      // 距離の小さいほうを選ぶ
      if (std::fabs(current_angle_ - angle0) < std::fabs(current_angle_ - angle180)) {
        target_angle_ = angle0;
      } else {
        target_angle_ = angle180;
      }
      target_selected_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "RotateFwdBwdAlongTrackNode: current=%.3f rad → target=%.3f rad",
                  current_angle_, target_angle_);
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
                  "RotateFwdBwdAlongTrackNode: reached target %.3f rad", target_angle_);
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

  rclcpp::Node::SharedPtr        node_;
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  double tolerance_;
  double current_angle_;
  bool   target_selected_;
  double target_angle_;
};

} 



BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::RotateFwdBwdAlongTrackNode>("RotateFwdBwdAlongTrackNode");
}