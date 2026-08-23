import datetime

from arduino.app_utils import App, Bridge
from arduino.app_bricks.dbstorage_tsstore import TimeSeriesStore

vehicle_run_db = TimeSeriesStore()
print(vehicle_run_db.bucket)
print(vehicle_run_db.token)
print(vehicle_run_db.org)
print(vehicle_run_db.url)

print("Hello world!")

def log_dict_to_db(db, channels, meas_grp_name):
    for chan_name, chan_val in channels.items():
        db.write_sample(chan_name, chan_val, measurement_name=meas_grp_name)

def log_car_health_cb(health_channels: dict):
    #will eventually turn this into a db logger and add db to a dashboard like grafana
    # print(f"Battery voltage mV: {health_channels['batt_mV']}")
    # print(f"Lidar Online: {bool(health_channels['lidar_online'])}")
    # print(f"IMU Online: {bool(health_channels['imu_online'])}")
    # print(f"MCU Time ms: {health_channels['time_ms']}")
    timestamp_ms = int(datetime.datetime.now().timestamp())
    for chan_name, chan_val in health_channels.items():
        vehicle_run_db.write_sample(chan_name, chan_val, measurement_name="health")

def log_mag_cb(mag_channels: dict):
    log_dict_to_db(vehicle_run_db,mag_channels,"imu_mag")

def log_acc_cb(acc_channels: dict):
    log_dict_to_db(vehicle_run_db,acc_channels,"imu_acc")

def log_gyro_cb(gyro_channels: dict):
    log_dict_to_db(vehicle_run_db,gyro_channels,"imu_gyro")

def log_imu_temp_cb(imu_temp_channels: dict):
    log_dict_to_db(vehicle_run_db,imu_temp_channels,"imu_temp")

# def loop():
#     """This function is called repeatedly by the App framework."""
#     # You can replace this with any code you want your App to run repeatedly.
#     time.sleep(10)

Bridge.provide("log_car_health_cb", log_car_health_cb)
Bridge.provide("log_mag_cb", log_mag_cb)
Bridge.provide("log_acc_cb", log_acc_cb)
Bridge.provide("log_gyro_cb", log_gyro_cb)
Bridge.provide("log_imu_temp_cb", log_imu_temp_cb)

print("Starting app")
# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run()
