from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from scipy.spatial.transform import Rotation as R

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    # 创建点集
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.3
    goal_pose1.pose.position.y = 2.5

    roll1 = 0.0
    pitch1 = 0.0
    yaw1 = 1.57  # 例如旋转90度，即π/2
      
    r1 = R.from_euler('xyz', [roll1, pitch1, yaw1])
    q1 = r1.as_quat()  

    goal_pose1.pose.orientation.x = q1[0]
    goal_pose1.pose.orientation.y = q1[1]
    goal_pose1.pose.orientation.z = q1[2]
    goal_pose1.pose.orientation.w = q1[3]

    goal_poses.append(goal_pose1)
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.0
    goal_pose2.pose.position.y = 3.6

    roll2 = 0.0
    pitch2 = 0.0
    yaw2 = 0.18  # 例如旋转90度，即π/2
      
    r2 = R.from_euler('xyz', [roll2, pitch2, yaw2])
    q2 = r2.as_quat()  

    goal_pose2.pose.orientation.x = q2[0]
    goal_pose2.pose.orientation.y = q2[1]
    goal_pose2.pose.orientation.z = q2[2]
    goal_pose2.pose.orientation.w = q2[3]


    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 1.5
    goal_pose3.pose.position.y = 3.2

    roll3 = 0.0
    pitch3 = 0.0
    yaw3 = -1.4  # 例如旋转90度，即π/2
      
    r3 = R.from_euler('xyz', [roll3, pitch3, yaw3])
    q3 = r3.as_quat()  

    goal_pose3.pose.orientation.x = q3[0]
    goal_pose3.pose.orientation.y = q3[1]
    goal_pose3.pose.orientation.z = q3[2]
    goal_pose3.pose.orientation.w = q3[3]

    goal_poses.append(goal_pose3)
    
    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 0.0
    goal_pose4.pose.position.y = 0.0

    roll4 = 0.0
    pitch4 = 0.0
    yaw4 = 3.14  # 例如旋转90度，即π/2
      
    r4 = R.from_euler('xyz', [roll4, pitch4, yaw4])
    q4 = r4.as_quat()  

    goal_pose4.pose.orientation.x = q4[0]
    goal_pose4.pose.orientation.y = q4[1]
    goal_pose4.pose.orientation.z = q4[2]
    goal_pose4.pose.orientation.w = q4[3]

    goal_poses.append(goal_pose4)
    
    
    # 调用路点导航服务
    navigator.followWaypoints(goal_poses)
    # 判断结束及获取反馈
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f'当前目标编号：{feedback.current_waypoint}')
    # 最终结果判断
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('导航结果：被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('导航结果：失败')
    else:
        navigator.get_logger().error('导航结果：返回状态无效')

if __name__ == '__main__':
    main()
