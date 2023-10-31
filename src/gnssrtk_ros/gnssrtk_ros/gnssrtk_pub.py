import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

from std_msgs.msg import String

from queue import Queue
from threading import Event
from time import sleep

from pygnssutils import VERBOSITY_LOW, GNSSNTRIPClient
from .gnssapp import GNSSSkeletonApp


# GNSS receiver serial port parameters - AMEND AS REQUIRED:
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
TIMEOUT = 10
# NTRIP caster parameters - AMEND AS REQUIRED:
# Ideally, mountpoint should be <30 km from location.
IPPROT = "IPv4"  # or "IPv6"
NTRIP_SERVER = "www.gnssdata.or.kr"
NTRIP_PORT = 2101
FLOWINFO = 0  # for IPv6
SCOPEID = 0  # for IPv6
MOUNTPOINT = "TEGN-RTCM32"  # leave blank to retrieve sourcetable
NTRIP_USER = "kde1054@naver.com"
NTRIP_PASSWORD = "gnss"
# NMEA GGA sentence status - AMEND AS REQUIRED:
GGAMODE = 0  # use fixed reference position (0 = use live position)
GGAINT = 60  # interval in seconds (-1 = do not send NMEA GGA sentences)
# Fixed reference coordinates (only used when GGAMODE = 1) - AMEND AS REQUIRED:
REFLAT = 51.176534
REFLON = -2.15453
REFALT = 40.8542
REFSEP = 26.1743

def hex_to_signed_24bit(hex_str):
    # Convert hex string to integer
    val = int(hex_str, 16)

    # Check if the value is negative
    if val & 0x800000:
        # Convert to signed 24-bit value
        val = val - 0x1000000

    return val


class GnssRtkPub(Node):
    def __init__(self):
        super().__init__('gnssrtk_pub')
        self.gnss_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.navpvt_pub = self.create_publisher(String, '/navpvt', 10)  # for debugging
        self.accel_pub = self.create_publisher(String, '/esfmeas/accel', 10)  # for debugging
        self.gyro_pub = self.create_publisher(String, '/esfmeas/gyro', 10)  # for debugging

        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.stop_event = Event()

        try:
            print(f"Starting GNSS reader/writer on {SERIAL_PORT} @ {BAUDRATE}...\n")
            with GNSSSkeletonApp(
                SERIAL_PORT,
                BAUDRATE,
                TIMEOUT,
                stopevent=self.stop_event,
                sendqueue=self.send_queue,
                receivequeue=self.receive_queue,
                idonly=True,
                enableubx=True,
                showhacc=True,
                verbose=False,
            ) as gna:
                gna.run()
                sleep(2)  # wait for receiver to output at least 1 navigation solution

                print(f"Starting NTRIP client on {NTRIP_SERVER}:{NTRIP_PORT}...\n")
                with GNSSNTRIPClient(gna, verbosity=VERBOSITY_LOW) as gnc:
                    streaming = gnc.run(
                        ipprot=IPPROT,
                        server=NTRIP_SERVER,
                        port=NTRIP_PORT,
                        flowinfo=FLOWINFO,
                        scopeid=SCOPEID,
                        mountpoint=MOUNTPOINT,
                        ntripuser=NTRIP_USER,  # pygnssutils>=1.0.12
                        ntrippassword=NTRIP_PASSWORD,  # pygnssutils>=1.0.12
                        reflat=REFLAT,
                        reflon=REFLON,
                        refalt=REFALT,
                        refsep=REFSEP,
                        ggamode=GGAMODE,
                        ggainterval=GGAINT,
                        output=self.send_queue,
                    )

                    idy_list = []

                    while (
                        streaming and not self.stop_event.is_set()
                    ):  # run until user presses CTRL-C
                        
                        parsed_data = self.receive_queue.get()
                        
                        if parsed_data:
                            # print(parsed_data)

                            if hasattr(parsed_data, "identity"):
                                idy = parsed_data.identity

                                # if idy not in idy_list:
                                #     idy_list.append(idy)
                                #     print(idy_list)
                                
                                if idy == 'NAV-PVT':
                                    if hasattr(parsed_data, "lat") and hasattr(parsed_data, "lon") and hasattr(parsed_data, "hMSL"):
                                        lat = parsed_data.lat
                                        lon = parsed_data.lon
                                        alt = parsed_data.hMSL / 1000.0 # convert to meters

                                        navsat_fix_msg = NavSatFix()
                                        t = self.get_clock().now()
                                        navsat_fix_msg.header.stamp = t.to_msg()
                                        navsat_fix_msg.header.frame_id = "gps_sensor"
                                        navsat_fix_msg.latitude = lat
                                        navsat_fix_msg.longitude = lon
                                        navsat_fix_msg.altitude = alt

                                        self.gnss_pub.publish(navsat_fix_msg)  

                                        # self.get_logger().info(f"hAcc: {parsed_data.hAcc} mm, vAcc: {parsed_data.vAcc} mm, sAcc: {parsed_data.sAcc} mm, pDOP: {parsed_data.pDOP}, numSV: {parsed_data.numSV}")
                                        # self.get_logger().info(f"hAcc: {parsed_data.hAcc} mm, vAcc: {parsed_data.vAcc} mm, pDOP: {parsed_data.pDOP}, numSV: {parsed_data.numSV}")

                                        navpvt_msg = String()
                                        navpvt_msg.data = str(f"{lat},{lon},{alt},{parsed_data.fixType},{parsed_data.hAcc},{parsed_data.vAcc},{parsed_data.pDOP},{parsed_data.numSV}")
                                        self.navpvt_pub.publish(navpvt_msg)
                                
                                elif idy == 'ESF-MEAS':
                                    # 16: accelX
                                    # 17: accelY
                                    # 18: accelZ
                                    meas_raw = {}
                                    for i in range(parsed_data.numMeas):
                                        data_type = eval(f'parsed_data.dataType_0{i+1}')
                                        data_val = eval(f'parsed_data.dataField_0{i+1}')
                                        if data_type == 16:
                                            meas_raw['accelX'] = float(hex_to_signed_24bit(hex(data_val))) / 1024.0
                                        if data_type == 17:
                                            meas_raw['accelY'] = float(hex_to_signed_24bit(hex(data_val))) / 1024.0
                                        if data_type == 18:
                                            meas_raw['accelZ'] = float(hex_to_signed_24bit(hex(data_val))) / 1024.0
                                        if data_type == 14:
                                            meas_raw['gyroX'] = float(hex_to_signed_24bit(hex(data_val))) / 4096.0
                                        if data_type == 13:
                                            meas_raw['gyroY'] = float(hex_to_signed_24bit(hex(data_val))) / 4096.0
                                        if data_type == 5:
                                            meas_raw['gyroZ'] = float(hex_to_signed_24bit(hex(data_val))) / 4096.0
                                        if data_type == 12:
                                            meas_raw['gyroTemp'] = float(hex_to_signed_24bit(hex(data_val))) / 100
                                    
                                    if meas_raw.keys() == {'accelX', 'accelY', 'accelZ'}:
                                        accel_msg = String()
                                        accel_msg.data = str(f"{meas_raw['accelX']},{meas_raw['accelY']},{meas_raw['accelZ']}")
                                        self.accel_pub.publish(accel_msg)
                                    elif meas_raw.keys() == {'gyroX', 'gyroY', 'gyroZ', 'gyroTemp'}:
                                        gyro_msg = String()
                                        gyro_msg.data = str(f"{meas_raw['gyroX']},{meas_raw['gyroY']},{meas_raw['gyroZ']},{meas_raw['gyroTemp']}")
                                        self.gyro_pub.publish(gyro_msg)
                                    pass
                                
                                elif idy == 'NAV-SAT':
                                    pass

                                elif idy == 'NAV-DOP':
                                    # print(parsed_data)
                                    pass


                        # else: 
                        #     sleep(0.5)

                    sleep(1)
        except KeyboardInterrupt:
            self.stop_event.set()
            print("Terminated by user")


def main(args=None):
    rclpy.init(args=args)
    gnss_rtk_node = GnssRtkPub()
    rclpy.spin(gnss_rtk_node)
    gnss_rtk_node.destroy_node()
    rclpy.shutdown()