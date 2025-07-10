import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
import math
import socket
import struct
import threading
import time

class PointDetectorNode(Node):
    def __init__(self):
        super().__init__('point_detector_node')

        # 目标点（map 坐标系中）
        self.target_points = [
            {'x': 0.692, 'y': 2.47},
            {'x': 0.695, 'y': 2.65},
            {'x': 0.701, 'y': 2.91},
            {'x': 0.665, 'y': 3.13}
        ]

        # 仅当机器人进入该区域后才进行检测
        self.detect_zone = {
            'x_min': 0.899, 'x_max': 2.34,
            'y_min': 1.23, 'y_max': 2.8
        }

        # 判断是否命中的半径距离
        self.detect_distance = 0.1

        self.robot_in_zone = False

        # 订阅位姿话题，判断是否进入检测区域
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # 发布 marker 显示检测结果
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_points', 10)

        # 创建 TF buffer 和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.last_detection_result = [False] * len(self.target_points)
        self.last_obstacle_positions = [None] * len(self.target_points)

        # TCP服务器参数（上位机作为服务器）
        self.server_ip = '0.0.0.0'  # 监听所有本地IP
        self.server_port = 8891     # 监听端口，可修改
        self.server_socket = None   # 服务器套接字
        self.client_socket = None   # 客户端连接套接字
        self.client_connected = False  # 客户端是否已连接
        self.client_address = None  # 客户端地址

        # 启动TCP服务器线程
        self.tcp_thread = threading.Thread(target=self.tcp_server_loop)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

        # 创建定时器，每1秒打印一次状态信息
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("✅ 点检测节点已启动（服务器模式）。")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 从四元数转换为欧拉角（只取 yaw）
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # 弧度

        # 打印 map 坐标 + 角度（°）
        self.get_logger().info(
            f'小车在map坐标系下的坐标值: x = {x:.2f}, y = {y:.2f}, 朝向 = {math.degrees(yaw):.1f}°'
        )

        self.robot_in_zone = (
            self.detect_zone['x_min'] <= x <= self.detect_zone['x_max'] and
            self.detect_zone['y_min'] <= y <= self.detect_zone['y_max']
        )

    def scan_callback(self, msg):
        if not self.robot_in_zone:
            return

        try:
            transform = self.tf_buffer.lookup_transform('map', 'laser_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'[TF] 变换不可用: {e}')
            return

        detection_result = [False] * len(self.target_points)
        obstacle_positions = [None] * len(self.target_points)
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            angle += msg.angle_increment

            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.header.stamp = msg.header.stamp
            pt.point.x = x_laser
            pt.point.y = y_laser
            pt.point.z = 0.0

            try:
                pt_map = tf2_geometry_msgs.do_transform_point(pt, transform)
            except Exception:
                continue

            for i, tgt in enumerate(self.target_points):
                dist = math.hypot(pt_map.point.x - tgt['x'], pt_map.point.y - tgt['y'])
                if dist < self.detect_distance:
                    detection_result[i] = True
                    obstacle_positions[i] = (pt_map.point.x, pt_map.point.y)

        self.publish_markers(detection_result)
        # 更新最近一次检测结果和障碍物位置
        self.last_detection_result = detection_result
        self.last_obstacle_positions = obstacle_positions


    def publish_markers(self, detection_result):
        marker_array = MarkerArray()
        for i, tgt in enumerate(self.target_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tgt['x']
            marker.pose.position.y = tgt['y']
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            if detection_result[i]:
                # 检测到障碍物：橙色
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                # 无障碍物：紫色
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        
    def timer_callback(self):
        # 构建完整的状态消息
        status_msg = "当前检测状态：\n"
        for i, tgt in enumerate(self.target_points):
            point_id = i + 1
            status = "有障碍物" if self.last_detection_result[i] else "无障碍物"
            flag = 1 if self.last_detection_result[i] else 0
            pos_str = ""
            if self.last_obstacle_positions[i]:
                x, y = self.last_obstacle_positions[i]
                pos_str = f"，障碍物坐标: x = {x:.2f}, y = {y:.2f}"
            
            status_msg += f"  检测点{point_id}({tgt['x']:.2f}, {tgt['y']:.2f}): 标志位={flag}，状态={status}{pos_str}\n"
        
        self.get_logger().info(status_msg)
        
        # 发送检测结果到已连接的客户端（如果连接存在）
        self.send_detection_status()
            
    def send_detection_status(self):
        """将检测结果通过TCP发送到已连接的客户端"""
        if not self.client_connected or self.client_socket is None:
            self.get_logger().warn("无客户端连接，无法发送检测状态")
            return
        
        try:
            # 将布尔列表转换为一个字节（每个布尔值对应一个比特）
            status_byte = 0
            for i, detected in enumerate(self.last_detection_result):
                if detected:
                    status_byte |= (1 << i)
            
            # 发送数据
            self.client_socket.sendall(struct.pack('B', status_byte))
            
            # 构建详细的发送状态信息
            status_bits = [(1 if self.last_detection_result[i] else 0) for i in range(3)]
            status_info = (
                f"已向客户端 {self.client_address} 发送检测状态: 标志位=0b{status_byte:08b} (十进制{status_byte})"
                f"，各点状态: 检测点1={status_bits[0]}, 检测点2={status_bits[1]}, 检测点3={status_bits[2]}"
            )
            self.get_logger().info(status_info)
            
        except Exception as e:
            self.get_logger().error(f"发送检测状态失败: {e}")
            self.close_client_connection()
            
    def tcp_server_loop(self):
        """TCP服务器主循环：监听连接并处理客户端通信"""
        # 创建服务器套接字
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口复用
            self.server_socket.bind((self.server_ip, self.server_port))
            self.server_socket.listen(1)  # 最多允许1个客户端连接
            self.get_logger().info(f"TCP服务器已启动，监听 {self.server_ip}:{self.server_port} ...")
        except Exception as e:
            self.get_logger().error(f"TCP服务器初始化失败: {e}")
            return

        while rclpy.ok():
            try:
                # 等待客户端连接（阻塞操作）
                self.client_socket, self.client_address = self.server_socket.accept()
                self.client_connected = True
                self.get_logger().info(f"客户端 {self.client_address} 已连接")

                # 处理客户端通信（循环检测连接状态）
                self.handle_client()

            except Exception as e:
                self.get_logger().warn(f"服务器监听异常: {e}")
                time.sleep(1)

        # 关闭服务器套接字
        if self.server_socket:
            self.server_socket.close()
        self.get_logger().info("TCP服务器已停止")

    def handle_client(self):
        """处理客户端连接：维持连接并检测断开"""
        while rclpy.ok() and self.client_connected:
            try:
                # 检测客户端是否断开（尝试接收1字节，非阻塞）
                self.client_socket.settimeout(1.0)  # 超时1秒
                data = self.client_socket.recv(1)  # 非阻塞接收
                if not data:
                    # 客户端主动断开连接
                    self.get_logger().info(f"客户端 {self.client_address} 主动断开连接")
                    break
            except socket.timeout:
                # 超时属于正常情况，继续循环
                continue
            except Exception as e:
                self.get_logger().warn(f"与客户端 {self.client_address} 通信异常: {e}")
                break

        # 关闭客户端连接
        self.close_client_connection()

    def close_client_connection(self):
        """关闭客户端连接并重置状态"""
        if self.client_connected and self.client_socket:
            try:
                self.client_socket.close()
            except Exception as e:
                self.get_logger().warn(f"关闭客户端连接失败: {e}")
        self.client_connected = False
        self.client_socket = None
        self.client_address = None
        self.get_logger().info("客户端连接已关闭，等待新连接...")


# main 函数入口
def main(args=None):
    rclpy.init(args=args)
    node = PointDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
