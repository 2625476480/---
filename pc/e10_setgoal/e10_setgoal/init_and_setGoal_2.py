from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation as R


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 设置初始位置
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.5
    r_init = R.from_euler('xyz', [0.0, 0.0, 0.0])
    q_init = r_init.as_quat()
    initial_pose.pose.orientation.x = q_init[0]
    initial_pose.pose.orientation.y = q_init[1]
    initial_pose.pose.orientation.z = q_init[2]
    initial_pose.pose.orientation.w = q_init[3]
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # 创建多个目标点
    goal_poses = []

    def create_goal(x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        r = R.from_euler('xyz', [0.0, 0.0, yaw_rad])
        q = r.as_quat()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    goal_poses.append(create_goal(0.5, 3.5, 0.18))   # 点1：
    goal_poses.append(create_goal(1.5, 3.2, -1.4))   # 点2：
    goal_poses.append(create_goal(1.17, 2.56, 3.14))   # 点3：

    # 循环逐个导航目标点
    for i, pose in enumerate(goal_poses):
        navigator.get_logger().info(f'开始导航到第 {i + 1} 个目标点...')
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f'  当前距离剩余：{feedback.distance_remaining:.2f} 米 | '
                    f'预计剩余时间：{feedback.estimated_time_remaining.sec} 秒 | '
                    f'已导航时间：{feedback.navigation_time.sec} 秒'
                )

                # 超时取消任务（例如 5 分钟）
                if feedback.navigation_time.sec > 300:
                    navigator.cancelTask()
                    navigator.get_logger().warn('导航超时，导航到第 {i + 1} 个目标点的任务取消')
                    break

        # 获取导航结果
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'成功到达第 {i + 1} 个目标点')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'第 {i + 1} 个导航被取消')
            break
        elif result == TaskResult.FAILED:
            navigator.get_logger().error(f'无法到达第 {i + 1} 个目标点，跳过')
            continue
        else:
            navigator.get_logger().error('未知导航结果状态')

    # 清理
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

