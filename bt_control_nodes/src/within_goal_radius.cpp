// =========================================
// Decorator: WithinGoalRadius
// 目標Poseと指定フレームの距離が r[m] 以内なら子をtick開始。
// 子が RUNNING の間は本ノードも RUNNING を返し継続tick。
// 子が SUCCESS / FAILURE になった時点でその結果を親へ返し、終了。
// （距離判定は既定でXY平面。起動後は距離に関係なく継続実行＝ラッチ動作）
// 依存:
// - blackboard["node"] に rclcpp::Node（Nav2流）
// - blackboard["tf_buffer"] に共有 tf2_ros::Buffer（無ければローカル生成）
// =========================================

#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <memory>
#include <string>
#include <chrono>

namespace BT
{

class WithinGoalRadius : public DecoratorNode
{
public:
  WithinGoalRadius(const std::string& name, const NodeConfiguration& config)
  : DecoratorNode(name, config)
  {
    setRegistrationID("WithinGoalRadius");

    if (!config.blackboard->get("node", node_))
    {
      node_ = std::make_shared<rclcpp::Node>("within_goal_radius_bt_node");
      RCLCPP_WARN(node_->get_logger(),
                  "[WithinGoalRadius] blackboard 'node' not found. Using local node.");
    }

    if (!config.blackboard->get("tf_buffer", tf_buffer_))
    {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_WARN(node_->get_logger(),
                  "[WithinGoalRadius] blackboard 'tf_buffer' not found. Creating local TF listener.");
    }
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<geometry_msgs::msg::PoseStamped>("goal", "目標Pose (PoseStamped)"),
      InputPort<double>("radius", 1.0, "半径[m]（この以内で子を起動）"),
      // InputPort<std::string>("frame", "base_link", "判定対象フレーム名"),
      InputPort<std::string>("frame", "mst110cr_2/base_link"),
      InputPort<bool>("use_xy_only", true, "XY平面のみで距離判定するか")
    };
  }

  NodeStatus tick() override
  {
    std::string frame = "mst110cr_2/base_link";
    getInput("frame", frame);

    RCLCPP_ERROR(node_->get_logger(), "WithinGoalRadius frame = %s", frame.c_str());

    if (!engaged_)
    {
      geometry_msgs::msg::PoseStamped goal;
      if (!getInput("goal", goal))
      {
        RCLCPP_ERROR(node_->get_logger(), "[WithinGoalRadius] 'goal' port is required");
        return NodeStatus::FAILURE;
      }

      double radius{};
      if (!getInput("radius", radius))
      {
        RCLCPP_ERROR(node_->get_logger(), "[WithinGoalRadius] 'radius' port is required");
        return NodeStatus::FAILURE;
      }

      std::string frame = "base_link";
      (void)getInput("frame", frame);

      bool use_xy_only = true;
      (void)getInput("use_xy_only", use_xy_only);

      const std::string goal_frame =
        goal.header.frame_id.empty() ? std::string("map") : goal.header.frame_id;

      geometry_msgs::msg::TransformStamped tf;
      try
      {
        using namespace std::chrono_literals;
        if (!tf_buffer_->canTransform(goal_frame, frame, tf2::TimePointZero, 100ms))
        {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[WithinGoalRadius] TF not available %s -> %s", goal_frame.c_str(), frame.c_str());
          return NodeStatus::FAILURE;
        }
        tf = tf_buffer_->lookupTransform(goal_frame, frame, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN(node_->get_logger(), "[WithinGoalRadius] TF error: %s", ex.what());
        return NodeStatus::FAILURE;
      }

      const double gx = goal.pose.position.x;
      const double gy = goal.pose.position.y;
      const double gz = goal.pose.position.z;

      const double px = tf.transform.translation.x;
      const double py = tf.transform.translation.y;
      const double pz = tf.transform.translation.z;

      double dist_sq = 0.0;
      if (use_xy_only)
      {
        const double dx = gx - px;
        const double dy = gy - py;
        dist_sq = dx*dx + dy*dy;
      }
      else
      {
        const double dx = gx - px;
        const double dy = gy - py;
        const double dz = gz - pz;
        dist_sq = dx*dx + dy*dy + dz*dz;
      }

      double dist = std::sqrt(dist_sq);
      RCLCPP_INFO(node_->get_logger(), "[WithinGoalRadius] Distance to goal: %.3f [m] (threshold=%.3f, frame=%s)", dist, radius, frame.c_str());

      const bool within = (dist_sq <= radius * radius);
      if (!within)
      {
        return NodeStatus::FAILURE;
      }

      engaged_ = true;
    }

    const auto child_status = child_node_->executeTick();

    switch (child_status)
    {
      case NodeStatus::RUNNING:
        return NodeStatus::RUNNING;

      case NodeStatus::SUCCESS:
        engaged_ = false;
        return NodeStatus::SUCCESS;

      case NodeStatus::FAILURE:
      default:
        engaged_ = false;
        return NodeStatus::FAILURE;
    }
  }

  void halt() override
  {
    if (child_node_ && child_node_->status() == NodeStatus::RUNNING)
    {
      child_node_->halt();
    }
    engaged_ = false;
    setStatus(NodeStatus::IDLE);
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool engaged_{false};
};

} // namespace BT

// =======================
// Registration
// =======================
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::WithinGoalRadius>("WithinGoalRadius");
}
