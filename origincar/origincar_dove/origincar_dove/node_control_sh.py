import os
import signal
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
from threading import Thread
from origincar_msg.msg import Sign  # 导入自定义消息

class NodeManager(Node):
    def __init__(self):
        super().__init__('node_manager')
        self.get_logger().info(f'Start DoveCar node_control')
        self.subscription1 = self.create_subscription(Int32, '/sign4return', self.handle_start_node, 10)
        self.subscription2 = self.create_subscription(Sign, '/sign_switch', self.handle_stop_node, 10)
        # 节点启动命令
        self.commands = {
            'node1': ['ros2', 'launch', 'origincar_bringup', 'camera.launch.py', 'enable_nv12_node:=true', 'enable_jpeg_node:=true'],
            'node2': ['ros2', 'run', 'line_follower_resnet', 'line_follower_resnet', '--ros-args', '-p', 'model_path:=/root/dev_ws/src/origincar/line_follower_resnet/model/resnet18_224x224_nv12.bin', '-p', 'model_name:=resnet18_224x224_nv12'],
            #'node3': ['ros2', 'run', 'play_football', 'play_football'],
            'node4': ['ros2', 'launch','qr_code_detection', 'qr_code_detection.launch.py']
        }
        # 进程字典，用于存储运行中的进程
        self.processes = {key: None for key in self.commands.keys()}

    def handle_start_node(self, msg):
        if msg.data == 0:
            self.start_nodes(['node1', 
                              'node2', 
                              #'node3', 
                              'node4'
                              ])
        elif msg.data == 6:
            self.start_nodes(['node2'
                              #'node3'
                              ])
        else:
            self.get_logger().info(f'Invalid /sign4return msg')
            
    def handle_stop_node(self, msg):
        if msg.sign_data == 3 or msg.sign_data == 4:
            self.stop_nodes(['node2', 
                             #'node3', 
                             'node4'
                            ])
        else:
            self.get_logger().info(f'Invalid /sign_switch msg')

    def start_nodes(self, nodes):
        for node in nodes:
            if self.processes[node] is None or self.processes[node].poll() is not None:
                # 如果进程不存在或已经结束
                self.processes[node] = subprocess.Popen(self.commands[node], preexec_fn=os.setsid)
                self.get_logger().info(f'Started {node}')
            else:
                # 如果进程已在运行，则不重新启动
                self.get_logger().info(f'{node} is already running and will not be restarted.')

    # def stop_nodes(self, nodes):
    #     for node in nodes:
    #         if self.processes[node] is not None:
    #             # 向整个进程组发送SIGTERM信号
    #             os.killpg(os.getpgid(self.processes[node].pid), signal.SIGTERM)
    #             self.processes[node] = None
    #             self.get_logger().info(f'Stopped {node}')
    
    def stop_nodes(self, nodes):
        for node in nodes:
            if self.processes[node] is not None:
                # 向进程组发送SIGTERM信号
                os.killpg(os.getpgid(self.processes[node].pid), signal.SIGTERM)
                # 给进程一段较短的时间来优雅地关闭
                try:
                    self.processes[node].wait(timeout=1)  # 等待1秒
                except subprocess.TimeoutExpired:
                    # 如果进程仍然没有结束，发送SIGKILL信号
                    os.killpg(os.getpgid(self.processes[node].pid), signal.SIGKILL)
                    self.processes[node].wait()  # 确保进程已经结束
                self.get_logger().info(f'Stopped {node} and attempted to release resources.')
                self.processes[node] = None

                
def main(args=None):
    rclpy.init(args=args)
    node_manager = NodeManager()
    try:
        rclpy.spin(node_manager)
    finally:
        # 清理所有进程
        for node, process in node_manager.processes.items():
            if process is not None:
                # 发送SIGTERM信号，优雅地关闭进程
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                try:
                    # 给予一些时间进行清理
                    process.wait(timeout=2)  # 等待2秒
                except subprocess.TimeoutExpired:
                    # 如果进程未在指定时间内结束，发送SIGKILL信号强制关闭
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        node_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
