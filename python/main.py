import time

from arduino.app_utils import App, Bridge

print("Hello world!")

def log_car_health_cb(health_channels: dict):
    #will eventually turn this into a db logger and add db to a dashboard like grafana
    print(f"Battery voltage mV: {health_channels['batt_mV']}")
    print(f"Lidar Online: {bool(health_channels['lidar_online'])}")
    print(f"IMU Online: {bool(health_channels['imu_online'])}")
    print(f"Time ms: {health_channels['time_ms']}")

# def loop():
#     """This function is called repeatedly by the App framework."""
#     # You can replace this with any code you want your App to run repeatedly.
#     time.sleep(10)

Bridge.provide("log_car_health_cb", log_car_health_cb)

print("Starting app")
# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run()
