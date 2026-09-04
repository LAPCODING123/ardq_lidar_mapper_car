#include <cstdint>
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
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_RouterBridge.h>

#ifndef Mapper_Car_h
#define Mapper_Car_h

/*-------------------------------------
//QWIIC I2C Bus Peripherals (I2C4) (Wire1)
/*Peripheral         | Address */
/*ICM-20948 IMU      |   0x69       
/*PCA9685 Servo Driv |   0x40 
/---------------------------------------
//Standard I2C2 (Wire)
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

#define MAX_IMU_LINK_ATTEMPTS 5
#define MAX_IMU_READ_ATTEMPTS 100
#define IMU_MEASURES_BUFF_SIZE 100
#define MAX_LIDAR_LINK_ATTEMPTS 5
#define SERVO_UPDATE_FREQ_HZ 50

#define PITCH_SERVO_DRIVER_PIN 0
#define YAW_SERVO_DRIVER_PIN 1


#define PITCH_DIGITAL_OFFSET 0
#define YAW_DIGITAL_OFFSET 0

#define YAW_MAX_RANGE 270
#define PITCH_MAX_RANGE 180

//defining stuff this way allows both actuators to be accurate within 0.1318359375 deg,
#define PITCH_TOL_DIG 3

#define YAW_TOL_DIG 2 //allow values withn 

typedef struct robot_pose
{
    float x_vel_m_per_s;
    float y_vel_m_per_s;
    float z_vel_m_per_s;
    float x_m;
    float y_m;
    float z_m;
    float mag_heading;
    float mag_pitch;
    float mag_roll;
    float heading_deg;
    float pitch_deg;
    float roll_deg;
    float time_ms; //later add lidar actuatior
    uint16_t lidar_pitch_dig;
    uint16_t lidar_yaw_dig;
} robot_pose;



class MapperCar {
  public:
    MapperCar();
    void init();
    int read_lidar_cm();
    int read_lidar_cm(int bias_corretion_n_meas);
    void telemeter_health(unsigned long updateRate_ms);
    void update_and_telemeter_sensors();
    void updateAndTelmMagData();
    void updateAndTelmAccData();
    void updateAndTelmGyroData();
    void telm_IMU_Temp();
    void update_pose();
    void lidarActuatetoTarget(double pitch_ang, double yaw_ang, double pitch_speed_dps, double yaw_speed_dps);
    void updateActuatorState(unsigned long updateRate_ms);
    void setTargetPose(robot_pose *target_pose_new);
    void telemeterLidarPose(unsigned long updateRate_ms);

  private:
    robot_pose last_pose;
    robot_pose target_pose;
    LIDARLite lidar;
    Adafruit_ICM20948 icm_imu;
    Adafruit_PWMServoDriver lidarActDriver;
    bool lidar_online;
    int lidar_error_persist_cnt;
    bool imu_online;
    int imu_error_persist_cnt;
    float battery_voltage;
    //will add in drivetrain next
    //future work may want to refactor imu data
    sensors_event_t mag_sample_hist[IMU_MEASURES_BUFF_SIZE];
    long mag_sample_times_us[IMU_MEASURES_BUFF_SIZE];
    int mag_sample_ind;
    sensors_event_t acc_sample_hist[IMU_MEASURES_BUFF_SIZE];
    long acc_sample_times_us[IMU_MEASURES_BUFF_SIZE];
    int acc_sample_ind;
    sensors_event_t gyro_sample_hist[IMU_MEASURES_BUFF_SIZE];
    long gyro_sample_times_us[IMU_MEASURES_BUFF_SIZE];
    int gyro_sample_ind;
    sensors_event_t temp_sample_hist[IMU_MEASURES_BUFF_SIZE];
    long temp_sample_times[IMU_MEASURES_BUFF_SIZE];
    int temp_sample_ind;
    void init_imu();
    void init_lidar();
    void init_batt_voltage_sensor();
    void initLidarAcuator();
    float update_vehicle_vel(float new_accel);
    float check_batt_V();
    bool setPWMPulseus(uint8_t n, double pulse);
    //sets the acuator to a position
    bool setLidarAcuatorPosition(double pitch_deg, double yaw_deg);
    bool setLidarAcuatorPosition(uint16_t pitch_digital, uint16_t yaw_digital);
    uint16_t angleToDigitalServoVal(double angle,double angle_range, double offset=0);
    double digitalServoValtoAngle(uint16_t digital_val, uint16_t max_digital_val, uint16_t offset);
    int last_lidar_distance_cm;

    
};




#endif /*Mapper_Car_h*/ 
