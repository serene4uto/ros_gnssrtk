import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message

from queue import Queue

from pyrtcm import RTCM_MSGIDS
from pyubx2 import NMEA_PROTOCOL, RTCM3_PROTOCOL, UBX_PROTOCOL, UBXReader, protocol

from pygnssutils import VERBOSITY_DEBUG, VERBOSITY_LOW, GNSSNTRIPClient, haversine
from time import sleep
# NTRIP caster parameters - AMEND AS REQUIRED:
# Ideally, mountpoint should be <30 km from location.
IPPROT = "IPv4"  # or "IPv6"
NTRIP_SERVER = "rtk2go.com"
NTRIP_PORT = 2101
FLOWINFO = 0  # for IPv6
SCOPEID = 0  # for IPv6
MOUNTPOINT = "VN1"
NTRIP_USER = "kde1054@naver.com"
NTRIP_PASSWORD = ""
# NMEA GGA sentence status - AMEND AS REQUIRED:
GGAMODE = 1  # use fixed reference position (0 = use live position)
GGAINT = 10  # interval in seconds (-1 = do not send NMEA GGA sentences)
# Fixed reference coordinates (used when GGAMODE = 1) - AMEND AS REQUIRED:
REFLAT = 53
REFLON = -2.4
REFALT = 40
REFSEP = 0


class RTCMPubNode(Node):
    def __init__(self):
        super().__init__('rtcm_pub_node')

        self.ntrip_queue = Queue()
        self.rtcm_pub = self.create_publisher(Message, '/rtcm', 1)
        timer_period = 1
        self.rtcmpub_timer = self.create_timer(timer_period, self.onRTCMPubTimerCallBack)
        self.rtcm_ctn = 0

        self.get_logger().info(f"Starting NTRIP client on {NTRIP_SERVER}:{NTRIP_PORT}...\n")
        with GNSSNTRIPClient(None, verbosity=VERBOSITY_LOW) as gnc:
            self.streaming = gnc.run(
                ipprot=IPPROT,
                server=NTRIP_SERVER,
                port=NTRIP_PORT,
                flowinfo=FLOWINFO,
                scopeid=SCOPEID,
                mountpoint=MOUNTPOINT,
                user=NTRIP_USER,
                password=NTRIP_PASSWORD,
                reflat=REFLAT,
                reflon=REFLON,
                refalt=REFALT,
                refsep=REFSEP,
                ggamode=GGAMODE,
                ggainterval=GGAINT,
                output=self.ntrip_queue,
            )

            while self.streaming:  # run until user presses CTRL-C
                rclpy.spin_once(self)
                sleep(1)
            sleep(1)

    def onRTCMPubTimerCallBack(self):
        try:
            raw_data, parsed_data = self.ntrip_queue.get()
            if protocol(raw_data) == RTCM3_PROTOCOL:
                # self.get_logger().info("Message received: {}".format(parsed_data))
                # self.rtcm_ctn +=1
                rtcm_msg = Message()
                rtcm_msg.message = raw_data
                rtcm_msg.header.frame_id = "rtcm3_data"
                rtcm_msg.header.stamp = self.get_clock().now().to_msg()
                self.rtcm_pub.publish(rtcm_msg)
        except Exception as err:
            self.get_logger().error(f"Something went wrong in send thread {err}")        

            

def main():
    rclpy.init()
    rtcm_pub_node = RTCMPubNode()
    rclpy.spin(rtcm_pub_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()