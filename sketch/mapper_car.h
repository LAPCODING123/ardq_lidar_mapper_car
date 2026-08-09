/*------------------------------------------------------------------------------

  Garmin Lidar Car Arduino Library

  This libary defines the configuration for the roboti lidar car

  Author: Lorenzo Pedroza (8/2/26)

  ICM Software setup: https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/arduino

------------------------------------------------------------------------------*/

#include <Servo.h>
#include <Wire.h>
#include "LIDARLite.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino_RouterBridge.h>

#ifndef Mapper_Car_h
#define Mapper_Car_h

/*-------------------------------------
//QWIIC I2C Bus Peripherals (I2C4)
/*Peripheral         | Address */
/*ICM-20948 IMU      |   0x69       
/*PCA9685 Servo Driv |   0x40 
/---------------------------------------
//Standard I2C2
/*Peripheral                 | Address */
//Garmin Lidar Lite v3       |   0x62

#define IMU_ICM_20948_I2C4_ADDR        0x69
#define SERVO_DRIVER_PCA9685_I2C4_ADDR 0x40
#define GARMIN_LIDAR_LITE_V3_I2C2_ADDR LIDARLITE_ADDR_DEFAULT

/*Motor drive (tbd)*/

#define BATT_V_SENSE_PIN A0
#define ADC_RESOLTION_mV 0.201416015625f      //3.3/(2^14)
#define OSEPP_VOLTAGE_SENSOR_SCALE_F 5 // multiply by 5 
#define OSEPP_ADC_READ_COUNTS_TO_V 0.001007080078125F //(ADC_RESOLTION_mV/1000)*OSEPP_VOLTAGE_SENSOR_SCALE_F



class MapperCar {
  public:
    MapperCar();
    void init();
    int read_lidar_cm(bool bias_corretion = true);
    void telemeter_health();

  private:
    LIDARLite lidar;
    Adafruit_ICM20948 icm_imu;
    bool lidar_online;
    int lidar_error_persist_cnt;
    bool imu_online;
    int imu_error_persist_cnt;
    float battery_voltage;
    //will add in drivetrain next
    void init_imu();
    void init_lidar();
    void init_batt_voltage_sensor();
    float update_vehicle_vel(float new_accel);
    float check_batt_V();
    
};




#endif /*Mapper_Car_h*/ 
