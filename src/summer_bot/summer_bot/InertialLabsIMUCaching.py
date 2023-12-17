import rclpy
from rclpy.node import Node
from inertiallabs_interfaces.msg import GnssData, GpsData, InsData, MarineData, SensorData
import os
import json
import yaml
import locale

### Need to append to a set of lists inside a JSON file:
### Each list represent a different type of IMU data.
### The JSON file needs to be created on initializing and continually appended to.

class INSCacher(Node):
    def __init__(self, save_path: str = './imu_cache'):
        super().__init__('ins_cacher')
        self.subscription1 = self.create_subscription(GnssData, '/Inertial_Labs/gnss_data', self.gnss_callback, 10)
        self.subscription2 = self.create_subscription(GpsData, '/Inertial_Labs/gps_data', self.gps_callback, 10)
        self.subscription3 = self.create_subscription(InsData, '/Inertial_Labs/ins_data', self.ins_callback, 10)
        self.subscription4 = self.create_subscription(MarineData, '/Inertial_Labs/marine_data', self.marine_callback, 10)
        self.subscription5 = self.create_subscription(SensorData, '/Inertial_Labs/sensor_data', self.sensor_callback, 10)
        
        # TODO: Check if file already exists...

        # Check if save folder exists:
        self.save_path = save_path
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        for path in [self.save_path + "/gnss.json", self.save_path + "/gps.json", self.save_path + "/ins.json", self.save_path + "/marine.json", self.save_path + "/sensor.json"]:
            if not os.path.exists(path):
                open(path, 'w').close()

        self.log_files = {
            'gnss': open(self.save_path + "/gnss.json", 'rb+'),
            'gps': open(self.save_path + "/gps.json", 'rb+'),
            'ins': open(self.save_path + "/ins.json", 'rb+'),
            'marine': open(self.save_path + "/marine.json", 'rb+'),
            'sensor': open(self.save_path + "/sensor.json", 'rb+')
        }

        for file in self.log_files.values():
            if os.stat(file.name).st_size == 0:
                file.write(bytes('[', encoding=locale.getpreferredencoding()))
            else:
                file.seek(-1, os.SEEK_END)
                file.truncate()
                file.write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def gnss_callback(self, msg: GnssData):
        gnss_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "gnss_info_1": msg.gnss_info_1,
            "gnss_info_2": msg.gnss_info_2,
            "number_sat": msg.number_sat,
            "gnss_velocity_latency": msg.gnss_velocity_latency,
            "gnss_angles_position_type": msg.gnss_angles_position_type,
            "gnss_heading": msg.gnss_heading,
            "gnss_pitch": msg.gnss_pitch,
            "gnss_gdop": msg.gnss_gdop,
            "gnss_pdop": msg.gnss_pdop,
            "gnss_hdop": msg.gnss_hdop,
            "gnss_vdop": msg.gnss_vdop,
            "gnss_tdop": msg.gnss_tdop,
            "new_gnss_flags": msg.new_gnss_flags,
            "diff_age": msg.diff_age,
            "pos_std": {
                "x": msg.pos_std.x,
                "y": msg.pos_std.y,
                "z": msg.pos_std.z,
            },
            "heading_std": msg.heading_std,
            "pitch_std": msg.pitch_std,
        }
        seri = json.dumps(gnss_data, indent=4)
        self.log_files['gnss'].write(bytes(seri, encoding=locale.getpreferredencoding()))
        self.log_files['gnss'].write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def gps_callback(self, msg: GpsData):
        gps_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "llh": {
                    "x": msg.llh.x,
                    "y": msg.llh.y,
                    "z": msg.llh.z,
                },
                "horspeed": msg.horspeed,
                "speeddir": msg.speeddir,
                "verspeed": msg.verspeed,
        }
        seri = json.dumps(gps_data, indent=4)
        self.log_files['gps'].write(bytes(seri, encoding=locale.getpreferredencoding()))
        self.log_files['gps'].write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def ins_callback(self, msg: InsData):
        ins_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "gps_ins_time": msg.gps_ins_time,
            "gps_imu_time": msg.gps_imu_time,
            "gps_msow": msg.gps_msow.data,
            "llh": {
                "x": msg.llh.x,
                "y": msg.llh.y,
                "z": msg.llh.z,
            },
            "ypr": {
                "x": msg.ypr.x,
                "y": msg.ypr.y,
                "z": msg.ypr.z,
            },
            "oriquat": {
                "x": msg.oriquat.x,
                "y": msg.oriquat.y,
                "z": msg.oriquat.z,
                "w": msg.oriquat.w,
            },
            "vel_enu": {
                "x": msg.vel_enu.x,
                "y": msg.vel_enu.y,
                "z": msg.vel_enu.z,
            },
            "solution_status": msg.solution_status.data,
            "pos_std": {
                "x": msg.pos_std.x,
                "y": msg.pos_std.y,
                "z": msg.pos_std.z,
            },
            "heading_std": msg.heading_std,
            "usw": msg.usw,
        }
        seri = json.dumps(ins_data, indent=4)
        self.log_files['ins'].write(bytes(seri, encoding=locale.getpreferredencoding()))
        self.log_files['ins'].write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def marine_callback(self, msg: MarineData):
        marine_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "heave": msg.heave,
            "surge": msg.surge,
            "sway": msg.sway,
            "heave_velocity": msg.heave_velocity,
            "surge_velocity": msg.surge_velocity,
            "sway_velocity": msg.sway_velocity,
            "significant_wave_height": msg.significant_wave_height,
        }
        seri = json.dumps(marine_data, indent=4)
        self.log_files['marine'].write(bytes(seri, encoding=locale.getpreferredencoding()))
        self.log_files['marine'].write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def sensor_callback(self, msg: SensorData):
        sensor_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "mag": {
                "x": msg.mag.x,
                "y": msg.mag.y,
                "z": msg.mag.z,
            },
            "accel": {
                "x": msg.accel.x,
                "y": msg.accel.y,
                "z": msg.accel.z,
            },
            "gyro": {
                "x": msg.gyro.x,
                "y": msg.gyro.y,
                "z": msg.gyro.z,
            },
            "temp": msg.temp,
            "vinp": msg.vinp,
            "pressure": msg.pressure,
            "barometric_height": msg.barometric_height,

        }
        seri = json.dumps(sensor_data, indent=4)
        self.log_files['sensor'].write(bytes(seri, encoding=locale.getpreferredencoding()))
        self.log_files['sensor'].write(bytes(',\n', encoding=locale.getpreferredencoding()))

    def shutdown(self):
        for file in self.log_files.values():
            file.seek(-2, os.SEEK_END)
            file.truncate()
            file.write(bytes(']', encoding=locale.getpreferredencoding()))
            file.close()

def main(args=None):
    rclpy.init(args=args)
    gps_listener = INSCacher()
    try:
        rclpy.spin(gps_listener)
    except KeyboardInterrupt:
        pass
    finally:
        gps_listener.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
