import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from nav_msgs.msg import Odometry

class BridgeNode(Node):
    """
    Bridge node from OpenVins to ArduPilot

    Publishes position estimate at 30 Hz
    """
    
    def __init__(self):
        super.__init__('bridge_node')
        self.subscription = self.create_subscription(
            Odometry,
            'nav_msgs/Odometry',
            self.on_odom,
            10, 
        )
        return
    
    def on_odom(self, msg: Odometry):
        '''
        1. Reads the position and orientation out of the ROS message
        2. Rewrites those numbers into ArduPilot's convention
        3. Sends them as the eight fields above
        '''
        # Rotate position
        # ENU -> NED
        msg.pose.pose.position
        enu_to_ned_matrix = [
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ]
        # Rotate orientation
        # FLU/ENU = FRD/NED
        msg.pose.pose.orientation # quaternion
        
        # Convert the ROS timestamp to microseconds
        msg.header.stamp.sec
        msg.header.stamp.nanosec
        return
    


def main():
    print('Hi from vio_bridge.')


if __name__ == '__main__':
    main()
