/*------------------------------------------------------------------------------

  Garmin Lidar Car Arduino Library

  This libary defines the configuration for the roboti lidar car

  Author: Lorenzo Pedroza (8/2/26)

  ICM Software setup: https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/arduino

------------------------------------------------------------------------------*/

#include "mapper_car.h"
#include <map>
#include <Wire.h>
#include "LIDARLite.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino_RouterBridge.h>


/*------------------------------------------------------------------------------
  Constructor

  Use Mapper_Car::init to initialize.
------------------------------------------------------------------------------*/
MapperCar::MapperCar(){}

void MapperCar::init()
{
   Bridge.begin(); //inits the MCU/MPU bridge
   init_lidar();
   init_imu();
   init_batt_voltage_sensor();
   imu_online = false;
   lidar_online = false;
   battery_voltage = 0;
   lidar_error_persist_cnt = 0;
   imu_error_persist_cnt = 0;
}

void MapperCar::init_imu()
{
   //to implement
}

void MapperCar::init_lidar()
{
  //default configuration
  lidar.begin(0, true);
  //star in default modes
  lidar.configure(0);
}

void MapperCar::init_batt_voltage_sensor()
{
  //just use the ADC on anlog pin 0
  //14 bit resolution
  analogReadResolution(14);
}

int MapperCar::read_lidar_cm(bool bias_corretion)
{
  int distance_cm = lidar.distance(bias_corretion);
  if (distance_cm == -1)
    lidar_error_persist_cnt++;
  else
  {
    lidar_error_persist_cnt == 0;
    lidar_online = true;
  }
  if (lidar_error_persist_cnt > 10)
     lidar_online = false; 
  return distance_cm;
}

//note the OSEPP VOLT-01 sensor module divides voltage in by 5

float MapperCar::check_batt_V()
{
  uint32_t fourteen_bit_adc_read;
  float battery_voltage_V;
  
  fourteen_bit_adc_read = analogRead(BATT_V_SENSE_PIN);

  battery_voltage_V = fourteen_bit_adc_read * OSEPP_ADC_READ_COUNTS_TO_V;
  battery_voltage = battery_voltage_V;
  return battery_voltage_V;
  
}

//reads sensor states and transmits to MPU (to transmit via web)
void MapperCar::telemeter_health()
{
  //algorithm
  std::map<String, int> health_channels{{"time_ms", 0},{"batt_mV", 0}, {"lidar_online", 0}, {"imu_online", 0}};
  health_channels["batt_mV"] = (int) (check_batt_V() * 1000 );
  health_channels["lidar_online"] = (int) lidar_online;
  health_channels["imu_online"] = (int) imu_online;
  health_channels["time_ms"] = (int) millis();
  Bridge.notify("log_car_health_cb", health_channels);
}

float MapperCar::update_vehicle_vel(float new_accel)
{
  //we'll get back to this
  // static unsigned long last_update_time_us = 0;
  // static std::array<std::array<float, 3>, 5> last_accels {{  // note double braces
  // { 1, 2, 3, 4 },
  // { 5, 6, 7, 8 },
  // { 9, 10, 11, 12 }}};
  return -1.0;
  
}
//will likely need seperate methods to read headin, accel, etc due to diffeernet update rates

//reads sensor states and transmits to MPU
// void MapperCar::update_vehicle_mag_heading()
// {
//   // check_batt_V()
//   // Bridge.notify("vehicle_raw_state", accel_x,accel_y)

// }

