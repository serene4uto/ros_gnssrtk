import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rtcm_msgs.msg import Message

from serial import Serial
from threading import Event, Lock, Thread
from queue import Queue
from time import sleep
from time import time

from io import BufferedReader
from pyrtcm import RTCM_MSGIDS
from pyubx2 import NMEA_PROTOCOL, RTCM3_PROTOCOL, UBX_PROTOCOL, UBXReader, protocol
from pygnssutils import haversine

SERIAL_PORT = "/dev/ttyACM0"
# BAUDRATE = 38400
BAUDRATE = 115200
TIMEOUT = 0.1

# Fixed reference coordinates (used when GGAMODE = 1) - AMEND AS REQUIRED:
REFLAT = 53
REFLON = -2.4
REFALT = 40
REFSEP = 0

class GPSPubNode(Node):
    def __init__(self):
        super().__init__('gps_pub_node')

        self.gps_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.rtcm_sub = self.create_subscription(Message, '/rtcm', self.onReceiveRTCMCallBack, 1)
        self.rate = self.create_rate(100)
        self.last_time = time()
        self.pkt_cnt = 0

        self.ntrip_queue = Queue()
        self.gnss_queue = Queue()
        self.serial_lock = Lock()
        self.stop = Event()

        try:
            self.get_logger().info(f"Opening serial port {SERIAL_PORT} @ {BAUDRATE}...\n")
            with Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT) as serial:
                self.stop.clear()
                
                print("Starting send thread...\n")
                send_thread = Thread(
                    target=self.send_gnss,
                    args=(
                        serial,
                        self.serial_lock,
                        self.stop,
                        self.ntrip_queue,
                    ),
                    daemon=True,
                )
                send_thread.start()  

                print("Starting read thread...\n")
                send_thread = Thread(
                    target=self.read_gnss,
                    args=(
                        serial,
                        self.serial_lock,
                        self.stop
                    ),
                    daemon=True,
                )
                send_thread.start()  

                while not self.stop.is_set():  # run until user presses CTRL-C
                    rclpy.spin_once(self)
                    # sleep(1)
                    self.rate.sleep()
                sleep(1)
        except KeyboardInterrupt:
            self.stop.set()

    def onReceiveRTCMCallBack(self, rtcm_msg):
        self.ntrip_queue.put(rtcm_msg.message)
        pass

    def send_gnss(self, stream, lock, stopevent, inqueue):
        """
        THREADED
        Reads RTCM3 data from message queue and sends it to receiver.
        """
        while not stopevent.is_set():
            try:
                raw_data = inqueue.get()
                if protocol(raw_data) == RTCM3_PROTOCOL:
                    lock.acquire()
                    stream.write(raw_data)
                    lock.release()
            except Exception as err:
                print(f"Something went wrong in send thread {err}")
                break

    def read_gnss(self, stream, lock, stopevent):
        """
        THREADED
        Reads and parses incoming GNSS data from receiver.
        """
        ubr = UBXReader(
            BufferedReader(stream),
            protfilter=(NMEA_PROTOCOL),
        )
    
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    lock.acquire()
                    (raw_data, parsed_data) = ubr.read()  # pylint: disable=unused-variable
                    lock.release()
                    if parsed_data:
                        idy = parsed_data.identity
                        if hasattr(parsed_data, "lat") and hasattr(parsed_data, "lon") and hasattr(parsed_data, "alt"):
                            lat = parsed_data.lat
                            lon = parsed_data.lon
                            alt = parsed_data.alt
                            # dev = haversine(lat, lon, REFLAT, REFLON) * 1000  # meters
                            # self.get_logger().info(
                            #     f"Receiver coordinates: {lat}, {lon}\r\n")
                            # self.get_logger().info(
                            #     f"Approximate deviation from fixed ref: {dev:06,f} m")
                            
                            self.pkt_cnt += 1
                            if self.pkt_cnt == 100:
                                self.get_logger().info(f"Sample Rate: {float(self.pkt_cnt / (time() - self.last_time))} Hz")
                                self.pkt_cnt = 0
                                self.last_time = time()
    
                            navsat_fix_msg = NavSatFix()
                            t = self.get_clock().now()
                            navsat_fix_msg.header.stamp = t.to_msg()
                            navsat_fix_msg.header.frame_id = "gps_sensor"
                            navsat_fix_msg.latitude = lat
                            navsat_fix_msg.longitude = lon
                            navsat_fix_msg.altitude = alt

                            self.gps_pub.publish(navsat_fix_msg)                    
                        
    
            except Exception as err:
                print(f"Something went wrong in read thread {err}")
                break


def main():
    rclpy.init()
    gps_pub_node = GPSPubNode()
    rclpy.spin(gps_pub_node)
    rclpy.shutdown()
