import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from vio_bridge.frames import position_enu_to_ned, orientation_enu_flu_to_ned_frd

class BridgeNode(Node):
    """
    Bridge node from OpenVins to ArduPilot

    Publishes position estimate at 30 Hz
    """
    
    def __init__(self):
        super().__init__('bridge_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.on_odom,
            10, 
        )
        # Endpoint is a parameter because it differs per target and the wrong
        # one fails silently -- the node runs, packets leave, and ArduPilot
        # never sees them. Under sim_vehicle.py --no-mavproxy nothing listens on
        # 14550 at all (--out is a MAVProxy argument and is ignored), but SITL
        # serves SERIAL1/SERIAL2 as TCP servers on 5762/5763. On hardware this
        # becomes the Pi's serial device, e.g. /dev/serial0,921600.
        self.declare_parameter('mavlink_url', 'udpout:127.0.0.1:14550')
        url = self.get_parameter('mavlink_url').get_parameter_value().string_value
        self.get_logger().info(f'MAVLink out: {url}')
        self.mav = mavutil.mavlink_connection(url, source_system=1, source_component=191)
        self._min_period_us = 1_000_000 // 30   # 30 Hz out; odomimu arrives at ~200 Hz
        self._last_sent_us = 0
        return
    
    def on_odom(self, msg: Odometry):
        """
        Translate one OpenVINS pose into a MAVLink VISION_POSITION_ESTIMATE.

        In:  position ENU, orientation of FLU body within ENU world (REP-103).
        Out: position NED, orientation of FRD body within NED world (aerospace).

        Origin offset and yaw alignment are deliberately NOT handled here --
        that is ArduPilot's Viso Align (RCx_OPTION = 80). This node only
        changes conventions.
        """
        usec = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000

        # Downsample to 30 Hz using measurement time, not wall clock, so this
        # behaves the same under bag playback and sim time.
        if usec < self._last_sent_us:          # bag looped or clock jumped
            self._last_sent_us = 0
        if usec - self._last_sent_us < self._min_period_us:
            return
        self._last_sent_us = usec

        # Position: ENU -> NED.  N = y_enu, E = x_enu, D = -z_enu
        p = msg.pose.pose.position
        x, y, z = position_enu_to_ned(p.x, p.y, p.z)

        # Orientation: rotate the world frame on the left, the body frame on
        # the right. Both must change -- ENU->NED alone leaves the body in FLU.
        q = msg.pose.pose.orientation
        roll, pitch, yaw = orientation_enu_flu_to_ned_frd(q.x, q.y, q.z, q.w)

        self.mav.mav.vision_position_estimate_send(
            usec, x, y, z, roll, pitch, yaw
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()