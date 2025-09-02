#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from amr_ros_msg.msg import PoseWithTypeStamped
from cartographer_ros_msgs.srv import (
    FinishTrajectory, 
    StartTrajectory,
    GetTrajectoryStates  # 新增：轨迹状态查询服务
)
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import UInt8
from launch_ros.substitutions import FindPackageShare
import os
import time
import threading

class CartographerInitializer(Node):
    def __init__(self):
        super().__init__('cartographer_initializer')
        
        # 服务客户端（使用独立的回调组避免阻塞）
        service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 创建服务客户端（新增GetTrajectoryStates客户端）
        self.finish_traj_client = self.create_client(
            FinishTrajectory, 
            '/finish_trajectory',
            callback_group=service_cb_group
        )
        self.start_traj_client = self.create_client(
            StartTrajectory, 
            '/start_trajectory',
            callback_group=service_cb_group
        )
        # 新增：轨迹状态查询客户端
        self.get_traj_states_client = self.create_client(
            GetTrajectoryStates,
            '/get_trajectory_states',
            callback_group=service_cb_group
        )
        
        # 状态发布者和订阅器保持不变
        self.state_publisher = self.create_publisher(
            UInt8,
            '/set_init_relocate_state',
            10
        )
        
        qos_profile = qos_profile_system_default
        self.subscription = self.create_subscription(
            PoseWithTypeStamped,
            '/initial_pose',
            self.pose_callback,
            qos_profile
        )
        
        self.get_logger().info(f"节点已创建，订阅话题: {self.subscription.topic_name}")
        
        # 初始化变量
        self.initial_pose = None
        self.received_valid_pose = False
        self.pose_lock = threading.Lock()
        self.initialization_done = False
        
        self.publish_state(0)
        self.get_logger().info("Cartographer 初始位姿设置节点已启动，等待 /initial_pose 消息...")
    
    def publish_state(self, state_value):
        """发布状态值"""
        msg = UInt8()
        msg.data = state_value
        self.state_publisher.publish(msg)
        self.get_logger().info(f"发布状态: {state_value}")
    
    def pose_callback(self, msg):
        """处理接收到的初始位姿消息"""
        self.get_logger().info(f"收到消息! 类型: {msg.type}")
        
        if msg.type == 'M':  # 只处理手动重定位消息
            with self.pose_lock:
                self.initial_pose = msg.inital_pose
                self.received_valid_pose = True
            pos = msg.inital_pose.pose.position
            self.get_logger().info(f"收到有效初始位姿: x={pos.x:.1f}, y={pos.y:.1f}")
            self.initialize_cartographer()
            
        else:
            self.get_logger().info(f"忽略非手动定位消息: {msg.type}")
            
    
    # 新增：获取当前活跃的轨迹ID
    def get_current_trajectory_id(self):
        """查询并返回当前活跃的轨迹ID（状态为ACTIVE=0）"""
        if not self.get_traj_states_client.service_is_ready():
            self.get_logger().warn('/get_trajectory_states 服务不可用，无法查询轨迹状态')
            return None
    
        request = GetTrajectoryStates.Request()
        future = self.get_traj_states_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    
        if future.result() is None:
            self.get_logger().error("查询轨迹状态失败")
            return None
    
        # 获取轨迹状态结构（根据实际服务定义）
        traj_states = future.result().trajectory_states
    
        # 遍历轨迹ID和对应的状态（两个数组长度相同）
        active_ids = []
        for traj_id, state in zip(traj_states.trajectory_id, traj_states.trajectory_state):
            if state == traj_states.ACTIVE:  # ACTIVE=0
                active_ids.append(traj_id)
    
        if not active_ids:
            self.get_logger().warn("未找到活跃的轨迹")
            return None
    
        # 返回第一个活跃轨迹ID
        self.get_logger().info(f"找到活跃轨迹ID: {active_ids[0]}")
        return active_ids[0]

    def finish_current_trajectory(self, trajectory_id=None):
        """停止当前轨迹（默认使用动态获取的ID）"""
        # 发布状态2: 开始停止轨迹
        self.publish_state(2)
        
        # 如果未指定轨迹ID，则动态获取
        if trajectory_id is None:
            trajectory_id = self.get_current_trajectory_id()
            if trajectory_id is None:
                self.get_logger.error("无法获取活跃轨迹ID，停止轨迹失败")
                return False
        
        if not self.finish_traj_client.service_is_ready():
            self.get_logger().warn('/finish_trajectory 服务不可用，请确保Cartographer正在运行')
            return False
        
        request = FinishTrajectory.Request()
        request.trajectory_id = trajectory_id
        
        future = self.finish_traj_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"成功停止轨迹 ID: {trajectory_id}")
            return True
        else:
            self.get_logger().error(f"停止轨迹 ID: {trajectory_id} 失败")
            return False
    
    def start_new_trajectory(self, initial_pose):
        """开始新的轨迹并设置初始位姿（保持不变）"""
        if not self.start_traj_client.service_is_ready():
            self.get_logger().warn('/start_trajectory 服务不可用，请确保Cartographer正在运行')
            return False
    
        request = StartTrajectory.Request()
        cartographer_share_dir = FindPackageShare('cartographer_ros').find('cartographer_ros')
        request.configuration_directory = os.path.join(
            cartographer_share_dir, 
            'configuration_files'
        )        
        request.configuration_basename = 'online_localization.lua'
        request.use_initial_pose = True
        request.relative_to_trajectory_id = 0
        request.initial_pose = initial_pose.pose
    
        self.get_logger().info(f"使用初始位姿: x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y}")
    
        future = self.start_traj_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"成功开始新轨迹! 轨迹 ID: {response.trajectory_id}")
            time.sleep(2.0)
            self.publish_state(4)
            time.sleep(1.0)
            self.publish_state(0)
            return True
        else:
            self.get_logger().error("开始新轨迹失败")
            self.publish_state(15)
            return False

    def initialize_cartographer(self):
        """执行初始化流程（修改停止轨迹的调用方式）"""
        self.get_logger().info("等待订阅建立...")
        time.sleep(1.0)
        
        # 步骤1: 等待有效的初始位姿消息
        start_time = time.time()
        while True:
            with self.pose_lock:
                if self.received_valid_pose:
                    break
        
        # 步骤2: 停止当前轨迹（不指定ID，动态获取）
        self.get_logger().info("正在停止当前轨迹...")
        if not self.finish_current_trajectory():
            self.get_logger().error("无法停止当前轨迹，初始化失败")
            return False
        
        # 步骤3: 启动新轨迹
        self.get_logger().info("正在启动新轨迹...")
        return self.start_new_trajectory(self.initial_pose)

def main():
    rclpy.init()
    node = CartographerInitializer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()