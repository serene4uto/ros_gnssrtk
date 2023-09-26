import haversine
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import logging
import os
from datetime import datetime
import threading
import numpy as np
import pyproj

LOG_PATH = '.log'

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s,%(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

base_coordinates = {
    'lat': 35.885136,
    'lon': 128.604986,
    'alt': 0,
}

base_distance = 0 # meters
bearing_deg = haversine.Direction.SOUTH # degrees, better to use milestone of direction such as: Direction.WEST, Direction.EAST, etc.


def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    # crs = pyproj.CRS("EPSG:32633")   # UTM Zone 33N coordinate system (for example)
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
    # crs = pyproj.CRS("EPSG:5181")  # Korea 2000 / Central Belt coordinate system (for Daegu South Korea)
    # crs = pyproj.CRS("EPSG:5179")  # Korea 2000 / Central Belt coordinate system (for Daegu South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

def dm(x):
    degrees = int(x) // 100
    minutes = x - 100*degrees

    return degrees, minutes

def decimal_degrees(degrees, minutes):
    return degrees + minutes/60 

class ErrorEval(Node):
    def __init__(self):
        super().__init__('error_eval')
        
        # ros2 params
        self.declare_parameter('log_enable', True)
        self.declare_parameter('val_plot', False)

        self.log_enable = self.get_parameter('log_enable').value
        self.val_plot = self.get_parameter('val_plot').value

        # subscribe to gps/fix topic
        self.sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_rx_callback,
            10)
        
        self.navpvt_sub = self.create_subscription(
            String,
            '/navpvt',
            self.navpvt_rx_callback,
            10)

        # calculate reference point from base coordinates and bearing
        self.sub_base = {}
        self.sub_base['lat'], self.sub_base['lon'] = haversine.inverse_haversine(
            point=(base_coordinates['lat'], base_coordinates['lon']),
            distance=base_distance,
            direction=bearing_deg,
            unit=haversine.Unit.METERS)
        
        self.sub_base['alt'] = base_coordinates['alt']

        sub_base_utm_easting, sub_base_utm_northing = get_utm_coordinates(self.sub_base['lat'], self.sub_base['lon'])
        self.sub_base['utm_easting'] = sub_base_utm_easting
        self.sub_base['utm_northing'] = sub_base_utm_northing


        self.eval_gnss = []


        # setting up logger
        os.makedirs(LOG_PATH, exist_ok=True)
        current_time = datetime.now().strftime("%Y%m%d%H%M%S")

        if self.log_enable:
            file_handler = logging.FileHandler(os.path.join(LOG_PATH, f'error_eval_{current_time}.csv'))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter('%(asctime)s,%(message)s'))
            logger.addHandler(file_handler)
        
        # stream_handler = logging.StreamHandler()
        # stream_handler.setLevel(logging.INFO)
        # stream_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
        # logger.addHandler(stream_handler)

        if self.val_plot:
            threading.Thread(target=self.plot_thread, daemon=True).start()



    def plot_thread(self):
        pass
    
    def navpvt_rx_callback(self, navpvt_msg):
        logger.info(navpvt_msg.data)
        pass

        
    def gps_rx_callback(self, navsat_msg):

        utm_easting, utm_northing = get_utm_coordinates(navsat_msg.latitude, navsat_msg.longitude)

        hpe = np.array(
        [   float(utm_easting) - float(self.sub_base['utm_easting']) ,
            float(utm_northing) - float(self.sub_base['utm_northing']) ]
        )

        vpe = abs(float(self.sub_base['alt']) - float(navsat_msg.altitude))

        self.get_logger().info(f'lat: {navsat_msg.latitude}, lon: {navsat_msg.longitude}, alt: {navsat_msg.altitude}, hpe: {hpe}, vpe: {vpe}')



def main(args=None):
    rclpy.init(args=args)
    error_eval_node = ErrorEval()
    rclpy.spin(error_eval_node)
    error_eval_node.destroy_node()
    rclpy.shutdown()