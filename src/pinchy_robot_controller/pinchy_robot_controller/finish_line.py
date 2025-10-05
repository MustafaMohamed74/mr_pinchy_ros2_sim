import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RegionTrigger(Node):
    def __init__(self):
        super().__init__('region_trigger')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.target_x = 11.2
        self.target_y = 1.0
        self.tolerance = 0.2
        self.triggered = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # print(x)
        
        if (abs(x - self.target_x) < self.tolerance):
            
            if not self.triggered:
                self.get_logger().info(f"🎯 Target region reached at ({x:.2f}, {y:.2f})!")
                self.trigger_action()
                self.triggered = True  # so it doesn’t repeat

    def trigger_action(self):
        # TODO: Replace this with what you want your robot to do
        self.get_logger().info("🤖 Triggering robot arm or next movement!")

def main(args=None):
    rclpy.init(args=args)
    node = RegionTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
