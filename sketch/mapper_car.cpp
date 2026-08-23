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
   mag_sample_ind = 0;
   temp_sample_ind = 0;
   gyro_sample_ind = 0;
   acc_sample_ind = 0;
}

void MapperCar::init_imu()
{
  //example https://github.com/adafruit/Adafruit_ICM20X/blob/master/examples/icm20948_unifiedsensors/icm20948_unifiedsensors.ino
  //startup the imu on qwiic connector
   int imu_link_attempts = 0;
   while(!icm_imu.begin_I2C(IMU_ICM_20948_I2C4_ADDR,&Wire1,0) &&
     (imu_link_attempts++ < MAX_IMU_LINK_ATTEMPTS))
     delay(100);
  //imu is online if we connected 
   imu_online = (imu_link_attempts < MAX_IMU_LINK_ATTEMPTS); 
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

//reads biased
int MapperCar::read_lidar_cm()
{
  int distance_cm = lidar.distance(false);
  if (distance_cm == -1)
    lidar_error_persist_cnt++;
  else
  {
    lidar_error_persist_cnt == 0;
    lidar_online = true;
  }
  if (lidar_error_persist_cnt >  MAX_LIDAR_LINK_ATTEMPTS)
     lidar_online = false; 
  return distance_cm;
}
//read with bias removal
int MapperCar::read_lidar_cm(int bias_corretion_n_meas)
{
  for (int i = 0; i < bias_corretion_n_meas-1; i++) {
    read_lidar_cm(); //need unbia
  }
  int distance_cm = lidar.distance(true);
  if (distance_cm == -1)
    lidar_error_persist_cnt++;
  else
  {
    lidar_error_persist_cnt == 0;
    lidar_online = true;
  }
  if (lidar_error_persist_cnt >  MAX_LIDAR_LINK_ATTEMPTS)
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

void MapperCar::updateAndTelmMagData()
{
  std::map<String, float> mag_channels{{"x_uT", 0},{"y_uT",0},{"z_uT",0},{"t_ms",0}};
  Adafruit_Sensor *icm_mag = icm_imu.getMagnetometerSensor();
  sensors_event_t mag;
  if(icm_mag->getEvent(&mag))
  {
    //update MCU local history
    mag_sample_hist[mag_sample_ind] = mag;
    mag_sample_times_us[mag_sample_ind] = micros();
    //prepare update for the MPU data logger
    mag_channels["x_uT"]=mag.magnetic.x;
    mag_channels["y_uT"]=mag.magnetic.y;
    mag_channels["z_uT"]=mag.magnetic.z;
    mag_channels["t_ms"] = ((float)mag_sample_times_us[mag_sample_ind])/1000.0;
    mag_sample_ind = (mag_sample_ind+1)%100;

    imu_error_persist_cnt = 0;
    imu_online = true;
    
    Bridge.notify("log_mag_cb", mag_channels);
  }
    //log sample miss as needed
  else //no sample available, increment error count
  {
      imu_online = imu_error_persist_cnt++ < MAX_IMU_READ_ATTEMPTS;
  }
}

//FUTURE NOTE. consider turning on accel Low Pass Filter.
void MapperCar::updateAndTelmAccData()
{
  std::map<String, float> acc_channels{{"x_m/s^2", 0},{"y_m/s^2",0},{"z_m/s^2",0},{"t_ms",0}};
  Adafruit_Sensor *icm_acc = icm_imu.getAccelerometerSensor();
  sensors_event_t acc;
  if(icm_acc->getEvent(&acc))
  {
    //update MCU local history
    acc_sample_hist[acc_sample_ind] = acc;
    acc_sample_times_us[acc_sample_ind] = micros();
    //prepare update for the MPU data logger
    acc_channels["x_m/s^2"]=acc.acceleration.x;
    acc_channels["y_m/s^2"]=acc.acceleration.y;
    acc_channels["z_m/s^2"]=acc.acceleration.z;
    acc_channels["t_ms"] = ((float)acc_sample_times_us[acc_sample_ind])/1000.0;
    acc_sample_ind = (acc_sample_ind+1)%100;

    imu_error_persist_cnt = 0;
    imu_online = true;
    Bridge.notify("log_acc_cb", acc_channels);
  }
    //log sample miss as needed
  else //no sample available, increment error count
  {
      imu_online = imu_error_persist_cnt++ < MAX_IMU_READ_ATTEMPTS;
  }
}

void MapperCar::updateAndTelmGyroData()
{
  std::map<String, float> gyro_channels{{"x_rad/s", 0},{"y_rad/s",0},{"z_rad/s",0},{"t_ms",0}};
  Adafruit_Sensor *icm_gyro = icm_imu.getGyroSensor();
  sensors_event_t gyro;
  if(icm_gyro->getEvent(&gyro))
  {
    //update MCU local history
    gyro_sample_hist[acc_sample_ind] = gyro;
    gyro_sample_times_us[acc_sample_ind] = micros();
    //prepare update for the MPU data logger

    gyro_channels["x_rad/s"]=gyro.gyro.x;
    gyro_channels["y_rad/s"]=gyro.gyro.y;
    gyro_channels["z_rad/s"]=gyro.gyro.z;
    gyro_channels["t_ms"] = ((float)gyro_sample_times_us[gyro_sample_ind])/1000.0;
    mag_sample_ind = (acc_sample_ind+1)%100;

    imu_error_persist_cnt = 0;
    imu_online = true;

    Bridge.notify("log_gyro_cb", gyro_channels);
  }
    //log sample miss as needed
  else //no sample available, increment error count
  {
      imu_online = imu_error_persist_cnt++ < MAX_IMU_READ_ATTEMPTS;
  }
}

void MapperCar::telm_IMU_Temp()
{
   std::map<String, float> imu_temp_channels{{"temp_deg_C", 0},{"t_ms",0}};
   Adafruit_Sensor *icm_temp = icm_imu.getTemperatureSensor();
   sensors_event_t temp;
   if(icm_temp->getEvent(&temp))
   {
     imu_temp_channels["temp_deg_C"]=temp.temperature;
     imu_temp_channels["t_ms"]=(float)micros()/1000.0;
     imu_error_persist_cnt = 0;
     imu_online = true;
     Bridge.notify("log_imu_temp_cb", imu_temp_channels);
   }
   else
   {
       imu_online = imu_error_persist_cnt++ < MAX_IMU_READ_ATTEMPTS;
   }
  
}

void MapperCar::update_and_telemeter_sensors()
{
    updateAndTelmMagData();
    updateAndTelmGyroData();
    updateAndTelmAccData();
    telm_IMU_Temp();
}

//will do next week if times allows 8/22
// void MapperCar::update_pose()
// {
//   float sum1[3] = {0};
//   float sum2[3] = {0};
//   float average1[3] = {0};
//   float average2[3] = {0};
//   float slope[3]={0};
//   //divide sample array and compute averages
//   if(acc_sample_ind == IMU_MEASURES_BUFF_SIZE - 1)
//   {
    
//     for (int i = 0; i < IMU_MEASURES_BUFF_SIZE/2; i++) {
//       sum1[0] += acc_sample_hist[i].acceleration.x;
//       sum1[1] += acc_sample_hist[i].acceleration.y;
//       sum1[2] += acc_sample_hist[i].acceleration.z;
//     }
   
//     for (int i = IMU_MEASURES_BUFF_SIZE/2; i < IMU_MEASURES_BUFF_SIZE; i++) {
//       sum2[0] += acc_sample_hist[i].acceleration.x;
//       sum2[1] += acc_sample_hist[i].acceleration.y;
//       sum2[2] += acc_sample_hist[i].acceleration.z;
//     }
//     for (int axis = 0; axis < 3; axis++)
//     {
//        average1[axis] = sum1[axis]/(IMU_MEASURES_BUFF_SIZE/2);
//        average2[axis] = sum2[axis]/(IMU_MEASURES_BUFF_SIZE/2);
//        slope[axis] = (average2[axis] - average1[axis]) /
//     }
//     last_pose.x_vel_m_per_s = average1[0]
      
//   }
  

  

//   if(acc_sample_ind == IMU_MEASURES_BUFF_SIZE - 1)
//   {
//     for (int i = 0; i < IMU_MEASURES_BUFF_SIZE/2; i++) {
      
    
//     }
//     for (int i = IMU_MEASURES_BUFF_SIZE/2; i < IMU_MEASURES_BUFF_SIZE; i++) {
    
//     }
//   }
  
//   if(acc_sample_ind == IMU_MEASURES_BUFF_SIZE - 1)
//   {
//     for (int i = 0; i < IMU_MEASURES_BUFF_SIZE/2; i++) {
      
    
//     }
//     for (int i = IMU_MEASURES_BUFF_SIZE/2; i < IMU_MEASURES_BUFF_SIZE; i++) {
    
//     }
//   }
// }


