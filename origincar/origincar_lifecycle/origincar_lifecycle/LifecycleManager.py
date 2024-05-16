import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, Transition
from std_msgs.msg import Int32
from lifecycle_msgs.srv import ChangeState, GetState

class LifecycleManager(Node):
    def __init__(self, managed_nodes):
        super().__init__('lifecycle_manager')
        self.managed_nodes = managed_nodes
        self.init_service_clients()

        # 订阅 /sign4return 话题来接收控制指令
        self.subscription = self.create_subscription(
            Int32, '/sign4return', self.handle_signal, 10)

    def init_service_clients(self):
        self.clients = {}
        for node in self.managed_nodes:
            self.clients[node] = {
                'change': self.create_client(ChangeState, f'{node}/change_state'),
                'get_state': self.create_client(GetState, f'{node}/get_state')
            }

    def handle_signal(self, msg):
        if msg.data == 1:
            self.transition_nodes(self.managed_nodes, 'activate')
        elif msg.data == 5:
            self.transition_nodes(self.managed_nodes, 'deactivate')
        elif msg.data == 6:
            self.transition_nodes(['line_follower', 'goal_detection'], 'activate')

    def transition_nodes(self, nodes, action):
        for node in nodes:
            self.manage_node_transition(node, action)

    def manage_node_transition(self, node, action):
        get_client = self.clients[node]['get_state']
        change_client = self.clients[node]['change']
        if get_client.wait_for_service(timeout_sec=1.0) and change_client.wait_for_service(timeout_sec=1.0):
            current_state = self.get_current_state(get_client)
            if self.should_transition(current_state, action):
                self.get_logger().info(f"Start Transition: {node}")
                self.change_node_state(change_client, self.determine_transition_id(action))

    def get_current_state(self, client):
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().current_state.label if future.result() else 'unknown'

    def should_transition(self, current_state, action):
        if action == 'activate' and current_state != 'active':
            return True
        if action == 'deactivate' and current_state != 'inactive':
            return True
        return False

    def determine_transition_id(self, action):
        if action == 'activate':
            return Transition.TRANSITION_ACTIVATE
        elif action == 'deactivate':
            return Transition.TRANSITION_DEACTIVATE

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Transition successful')
        else:
            self.get_logger().error('Transition failed')

def main(args=None):
    rclpy.init(args=args)
    managed_nodes = ['qr_code_detection', 'line_follower', 'goal_detection']
    manager = LifecycleManager(managed_nodes)
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
