import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    TestOption(name="move_forward", id=1),
    TestOption(name="move_lateral", id=2),
    TestOption(name="rotate", id=3),
    TestOption(name="stop_move", id=4),
    TestOption(name="switch_gait_trot", id=5),
    TestOption(name="switch_gait_walk", id=6),
]

class Go2UserCommands(Node):
    def __init__(self):
        super().__init__('go2_user_test_commands')

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._gait_pub = self.create_publisher(Int8, '/go2_controller/gait_type', 10)

        # Client init example | Currently commented out as these services are not provided in the simulated environment
        '''
        self.stand_cli = self.create_client(Trigger, '/go2_controller/stand')
        self.stand_down_cli = self.create_client(Trigger, '/go2_controller/stand_down')

        while not self.stand_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /stand service...')
        while not self.stand_down_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /stand_down service...')
        '''
        
        self.actions = {
            1: lambda: self._move_cmd(0.3, 0, 0),
            2: lambda: self._move_cmd(0, 0.3, 0),
            3: lambda: self._move_cmd(0, 0, 0.5),
            4: self._stop,
            5: lambda: self._switch_gait(0),
            6: lambda: self._switch_gait(1),
        }

    # Service call example
    '''
    def _stand_up(self):
        self.stand_cli.call_async(request=Trigger.Request()).add_done_callback(
            lambda future: self.get_logger().info('Stand up command sent successfully.')
        ) 

    def _stand_down(self):
        self.stand_down_cli.call_async(request=Trigger.Request()).add_done_callback( 
            lambda future: self.get_logger().info('Stand down command sent successfully.')
        ) 

    '''

    def _move_cmd(self, x: float, y: float, yaw: float):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(yaw)

        self._cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Published move: x={x}, y={y}, yaw={yaw}")

    def _stop(self):
        self._move_cmd(0.0, 0.0, 0.0) # pub cmd
        self.get_logger().info("Published stop command")

    def _switch_gait(self, gait_id):
        msg = Int8()
        msg.data = gait_id

        self._gait_pub.publish(msg) # pub cmd
        self.get_logger().info(f"Switched gait to ID {gait_id}")


class Go2UserInterface:
    def __init__(self, options: list[TestOption]):
        self._options = options
        self._commands_node = Go2UserCommands()

    def _list_commands(self):
        print("Available commands:")
        for option in option_list:
            print(f"{option.id}: {option.name}")

    def _get_command(self, input_str: str) -> TestOption | None:
        try:
            input_val = int(input_str)
        except ValueError:
            input_val = input_str

        return next((o for o in option_list if o.name == input_val or o.id == input_val), None)
    
    def _execute_command(self, option: TestOption):
        action = self._commands_node.actions.get(option.id)

        if action:
            print(f"Executing command: {option.name} (ID: {option.id})")
            action()
        else:
            print(f"Command {option.name} (ID: {option.id}) is not implemented.")

    def terminal_handler(self):
        try:
            while rclpy.ok():
                input_str = input(f"Enter a command (1-{len(self._options)}, 'list', 'exit'): ").strip()
                
                rclpy.spin_once(self._commands_node, timeout_sec=0.1)  # Allow callbacks (non-blocking)

                if input_str.lower() == 'list':
                    self._list_commands()
                    continue

                if input_str.lower() == 'exit':
                    print("Exiting the command interface.")
                    break

                option = self._get_command(input_str)
                if not option:
                    print(f"Command '{input_str}' not recognized.")
                    continue

                self._execute_command(option)

        finally:
            self._commands_node.destroy_node()



def main(args=None):
    rclpy.init()

    user_interface = Go2UserInterface(option_list)
    user_interface.terminal_handler()

    rclpy.shutdown()