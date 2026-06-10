import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Load parameters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_kp', .5)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('max_linear_speed', .3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_lost_frames', 10)
        
        self.target_distance = self.get_parameter('target_distance').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value

        self.lost_frames = 0

        self.detection_sub = self.create_subscription(
           Float32MultiArray,
            '/tracked_person',
            self.detection_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Controller node started')
    
    def detection_callback(self, msg):
        cmd = Twist()

        track_id = msg.data[0]
        angle = msg.data[1]
        distance = msg.data[2]
    
        # -1 means no target
        if track_id < 0:
            self.lost_frames += 1
            if self.lost_frames >= self.max_lost_frames:
                self.cmd_vel_pub.publish(cmd)
            return

        self.lost_frames = 0

        DISTANCE_DEAD_ZONE = 0.08

        distance_error = distance - self.target_distance
        if abs(distance_error) < DISTANCE_DEAD_ZONE:  
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = float(max(-self.max_linear_speed,
                min(self.max_linear_speed, self.linear_kp * distance_error)))

        ANGULAR_DEAD_ZONE = 0.1

        if abs(angle) < ANGULAR_DEAD_ZONE:
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = float(
                max(-self.max_angular_speed,
                    min(self.max_angular_speed,
                        self.angular_kp * angle))
            )

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()