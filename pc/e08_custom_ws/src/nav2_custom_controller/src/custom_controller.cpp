#include <angles/angles.h>//改进误差

#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace nav2_custom_controller {

// 控制器初始化与参数读取
void CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;

  // 声明并获取参数
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".Kp", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".Kd", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".Ki", rclcpp::ParameterValue(0.0));

  // 参数化误差加权
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".lateral_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".heading_weight", rclcpp::ParameterValue(1.0));
  
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".goal_dist_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".goal_yaw_threshold", rclcpp::ParameterValue(0.1));
  
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".Kp", Kp_);
  node_->get_parameter(plugin_name_ + ".Ki", Ki_);
  node_->get_parameter(plugin_name_ + ".Kd", Kd_);
  
  node_->get_parameter(plugin_name_ + ".lateral_weight", lateral_weight_);
  node_->get_parameter(plugin_name_ + ".heading_weight", heading_weight_);
  
  node_->get_parameter(plugin_name_ + ".goal_dist_threshold", goal_dist_threshold_);
  node_->get_parameter(plugin_name_ + ".goal_yaw_threshold", goal_yaw_threshold_);

  RCLCPP_INFO(node_->get_logger(),
            "使用误差权重: lateral_weight = %.2f, heading_weight = %.2f",
            lateral_weight_, heading_weight_);


  // 初始化时间记录（用于微分项计算）
  last_time_ = node_->get_clock()->now();
  
  
  // 初始化积分项
  integral_ = 0.0;
  
  //初始化这个 marker publisher
  lookahead_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
  "lookahead_point_marker", 10);

}

// 生命周期回调函数（标准插件结构）
void CustomController::cleanup() {
  RCLCPP_INFO(node_->get_logger(), "Cleaning up controller: %s", plugin_name_.c_str());
}

void CustomController::activate() {
  RCLCPP_INFO(node_->get_logger(), "Activating controller: %s", plugin_name_.c_str());
}

void CustomController::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "Deactivating controller: %s", plugin_name_.c_str());
}

// 接收由全局规划器传来的路径
void CustomController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
  
  RCLCPP_INFO(node_->get_logger(), "[setPlan] setPlan() 收到路径，长度为: %zu，frame_id: %s",
              global_plan_.poses.size(),
              global_plan_.header.frame_id.c_str());
}

// 本控制器不使用速度限制
void CustomController::setSpeedLimit(const double &, const bool &) {}


geometry_msgs::msg::PoseStamped CustomController::getLookaheadPose(
    const geometry_msgs::msg::PoseStamped &current_pose) 
{
    using nav2_util::geometry_utils::euclidean_distance;
    for (const auto &pose : global_plan_.poses) 
    {
      if (euclidean_distance(current_pose, pose) > lookahead_distance_ && euclidean_distance(current_pose, pose) < 0.4) 
      {
        // 计算 dx, dy
        double dx = pose.pose.position.x - current_pose.pose.position.x;
        double dy = pose.pose.position.y - current_pose.pose.position.y;

        // 计算机器人朝向
        double yaw = tf2::getYaw(current_pose.pose.orientation);

        // 将目标点向量投影到机器人前向方向（朝向向量 dot 差值向量）
        double forward_component = dx * std::cos(yaw) + dy * std::sin(yaw);

        // 判断是否在前方（正前方分量 > 0）
        if (forward_component > 0.0) 
          {	                
                RCLCPP_INFO(node_->get_logger(),
                              "[getLookaheadPose] 找到前瞻点，距离大于 %.2f 米，使用该路径点，frame_id: '%s'",
                              lookahead_distance_,
                              pose.header.frame_id.c_str());
                
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = pose.header.frame_id.empty() ? "map" : pose.header.frame_id;
                      marker.header.stamp = node_->get_clock()->now();
                marker.ns = "lookahead_point";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = pose.pose;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 或更长

                lookahead_marker_pub_->publish(marker);

                return pose;
              
          }
      }
    }




  // ✅ 如果没有找到合适点（没有找到足够远的目标点，使用路径的最后一个点（终点）作为 fallback），手动构造终点（确保 header 不为空）
  geometry_msgs::msg::PoseStamped fallback = global_plan_.poses.back();
  fallback.header = global_plan_.header;  // ✅ 添加 header，避免 tf2 报错
  
  RCLCPP_WARN(node_->get_logger(),
              "[getLookaheadPose] 未找到距离大于 %.2f 米的前瞻点，使用路径终点作为备选目标点，frame_id: '%s'",
              lookahead_distance_,
              fallback.header.frame_id.c_str());

  // ✅ 添加 fallback 前瞻点 marker 可视化
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = fallback.header.frame_id.empty() ? "map" : fallback.header.frame_id;
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "lookahead_point";
  marker.id = 1;  // 避免与正常 marker 冲突
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = fallback.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;  
  marker.color.g = 0.0;
  marker.color.b = 1.0; // fallback marker 用蓝色
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  lookahead_marker_pub_->publish(marker);
  
  return fallback;
}


// 主函数：计算控制速度
geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
    
  //调试信息
  RCLCPP_INFO(node_->get_logger(), "[自定义局部控制器] computeVelocityCommands() 被调用");


  // 检查路径是否为空
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Global plan is empty");
  }

  // 当前机器人姿态转换为路径坐标系
  geometry_msgs::msg::PoseStamped pose_in_globalframe;
  if (!nav2_util::transformPoseInTargetFrame(
          pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1)) {
    throw nav2_core::PlannerException("坐标转换失败");
  }

  // 查找前瞻点
  auto target_pose = getLookaheadPose(pose_in_globalframe);
  
  // ✅ 判断是否到达目标点（只在接近终点时 getLookaheadPose 返回的是最后一个点）
  // 计算与目标点的欧式距离与朝向误差
	double dx = target_pose.pose.position.x - pose_in_globalframe.pose.position.x;
	double dy = target_pose.pose.position.y - pose_in_globalframe.pose.position.y;
	double distance_to_goal = std::hypot(dx, dy);

	double yaw_robot = tf2::getYaw(pose_in_globalframe.pose.orientation);
	double yaw_goal = tf2::getYaw(target_pose.pose.orientation);
	double yaw_error = angles::shortest_angular_distance(yaw_robot, yaw_goal);

	// ✅ 如果距离和角度都在阈值内，则认为已到达终点，返回 0 速度
	if (distance_to_goal < goal_dist_threshold_ && std::abs(yaw_error) < goal_yaw_threshold_) {
	    RCLCPP_INFO(node_->get_logger(),
		"[控制] 已到达终点！距离：%.3f，角度误差：%.3f，阈值：%.3f/%.3f，发送停止指令",
		distance_to_goal, yaw_error, goal_dist_threshold_, goal_yaw_threshold_);

	    geometry_msgs::msg::TwistStamped stop_cmd;
	    stop_cmd.header.stamp = node_->get_clock()->now();
	    stop_cmd.header.frame_id = "base_link";
	    stop_cmd.twist.linear.x = 0.0;
	    stop_cmd.twist.angular.z = 0.0;
	    return stop_cmd;
	}


  // 将前瞻点变换到 base_link 坐标系
  geometry_msgs::msg::PoseStamped target_pose_base;
  try {

    auto global_frame = global_plan_.header.frame_id;
    
    RCLCPP_INFO(node_->get_logger(), "global_plan_.header.frame_id: '%s'", global_plan_.header.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "target_pose.header.frame_id: '%s'", target_pose.header.frame_id.c_str());

    
    geometry_msgs::msg::TransformStamped transform =
    tf_->lookupTransform("base_link", global_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    
    tf2::doTransform(target_pose, target_pose_base, transform);

  } catch (tf2::TransformException &ex) {
    throw nav2_core::PlannerException("TF转换失败：" + std::string(ex.what()));
  }

  // 获取 lateral error（目标点在 base_footprint 坐标系下的 y 值）
  double lateral_error = target_pose_base.pose.position.y;
  
  // 获取当前朝向（yaw_current）与目标点朝向（yaw_target）
  tf2::Quaternion q_current, q_target;
  tf2::fromMsg(pose.pose.orientation, q_current);
  tf2::fromMsg(target_pose.pose.orientation, q_target);

  double yaw_current = tf2::getYaw(q_current);
  double yaw_target = tf2::getYaw(q_target);

  // 计算 heading 误差（目标方向 - 当前方向），范围为 [-pi, pi]
  double heading_error = angles::shortest_angular_distance(yaw_current, yaw_target);
  
  // ✅ 综合误差（可以选择是否加权）
  double total_error = lateral_weight_ * lateral_error ;//+ heading_weight_ * heading_error;
  
  // 打印调试信息
  RCLCPP_INFO(node_->get_logger(),
  "[控制] lateral_error=%.4f, heading_error=%.4f, total_error=%.4f (权重：%.2f, %.2f)",
  lateral_error, heading_error, total_error,
  lateral_weight_, heading_weight_);


  // 计算时间差和误差变化率（微分项）
  auto now = node_->get_clock()->now();
  double dt = (now - last_time_).seconds();
  double derivative = dt > 0.0 ? (total_error - last_lateral_error_) / dt : 0.0;
  last_time_ = now;
  last_lateral_error_ = total_error;
  
  // 更新积分值（防止dt为0）
  if (dt > 0.0) {
    integral_ += total_error * dt;

    // 积分抗饱和（可选）
    double integral_limit = 1.0;
    integral_ = std::clamp(integral_, -integral_limit, integral_limit);
}

  // PID 控制：比例 + 微分
  double angular_z = 0.1* (Kp_ * total_error + Ki_ * integral_ ); //+ Kd_ * derivative;

  // 构造速度指令（单位为 base_footprint 坐标系）
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.twist.linear.x = std::max(0.0, max_linear_speed_); // 强制不倒退
  cmd_vel.twist.angular.z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);
  //cmd_vel.twist.angular.z = 0.5;

  // 调试打印
  RCLCPP_INFO(node_->get_logger(), "[自定义局部控制器] cmd_vel: linear.x=%.2f, angular.z=%.2f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

  return cmd_vel;
}
} // namespace nav2_custom_controller

// 导出插件（供 Nav2 插件机制加载）
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)

