#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace nav2_custom_controller {

class CustomController : public nav2_core::Controller {
public:
  CustomController() = default;
  ~CustomController() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker *goal_checker) override;

  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

protected:
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D *costmap_;
  nav_msgs::msg::Path global_plan_;

  // 参数：最大线速度与角速度
  double max_angular_speed_;
  double max_linear_speed_;

  // 阿克曼控制器专用参数
  double lookahead_distance_;
  double Kp_, Kd_,Ki_;
  
  double lateral_weight_;
  double heading_weight_;

  
  double last_lateral_error_ = 0.0;
  double integral_;                 // 积分累积值
  
  double goal_dist_threshold_;
  double goal_yaw_threshold_;

  rclcpp::Time last_time_;

  // 替代最近点：返回路径中前瞻距离的目标点
  geometry_msgs::msg::PoseStamped
  getLookaheadPose(const geometry_msgs::msg::PoseStamped &current_pose);
  
  // 用于发布前瞻点可视化标记
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  


};

} // namespace nav2_custom_controller

#endif // NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
