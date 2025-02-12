import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        
        self.subscription = self.create_subscription(
            String, 'voice_commands', self.command_callback, 10)

       
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("Robot Controller Node Started")

    def command_callback(self, msg):
        """Processes voice commands and sends movement commands"""
        command = msg.data.lower()  
        twist = Twist()

        if "move" in command in command:
            twist.linear.x = 0.5 
        elif "left" in command:
            twist.angular.z = 0.5  
        elif "right" in command:
            twist.angular.z = -0.5  
        elif "straight" in command:
            twist.linear.x = 0.5
            twist.angular.z = 0.0  

        self.publisher_.publish(twist)
        self.get_logger().info(f"Executed Command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
