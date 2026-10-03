#ifndef ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
#define ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H

#include <LibAvionics.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_LIS3DH.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// m/s^2 -> g (the FSM works in g)
constexpr double USER_MS2_TO_G = 1.0 / SENSORS_GRAVITY_STANDARD;

// Temperature and humidity (BME280, AHT20)
struct SensorEnvData {
  double temperature_c;
  double humidity_rh;
};

class IMU_MPU6050 final : public SensorIMU {
protected:
  Adafruit_MPU6050 mpu;
  TwoWire         &wire;
  uint8_t          addr;
  sensors_event_t  a{}, g{}, t{};

public:
  IMU_MPU6050(TwoWire &wire, const uint8_t addr) : SensorIMU(), wire(wire), addr(addr) {
  }

  bool begin() override {
    if (!mpu.begin(addr, &wire))
      return false;
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
  }

  bool read() override {
    return mpu.getEvent(&a, &g, &t);
  }

  double acc_x() override {
    return a.acceleration.x * USER_MS2_TO_G;
  }

  double acc_y() override {
    return a.acceleration.y * USER_MS2_TO_G;
  }

  double acc_z() override {
    return a.acceleration.z * USER_MS2_TO_G;
  }

  double gyr_x() override {
    return g.gyro.x;  // rad/s
  }

  double gyr_y() override {
    return g.gyro.y;  // rad/s
  }

  double gyr_z() override {
    return g.gyro.z;  // rad/s
  }
};

class IMU_LIS3DH final : public SensorIMU {
protected:
  Adafruit_LIS3DH lis;
  uint8_t         addr;
  sensors_event_t a{};

public:
  IMU_LIS3DH(TwoWire &wire, const uint8_t addr) : SensorIMU(), lis(&wire), addr(addr) {
  }

  bool begin() override {
    if (!lis.begin(addr))
      return false;
    lis.setRange(LIS3DH_RANGE_16_G);
    lis.setDataRate(LIS3DH_DATARATE_50_HZ);
    return true;
  }

  bool read() override {
    return lis.getEvent(&a);
  }

  double acc_x() override {
    return a.acceleration.x * USER_MS2_TO_G;
  }

  double acc_y() override {
    return a.acceleration.y * USER_MS2_TO_G;
  }

  double acc_z() override {
    return a.acceleration.z * USER_MS2_TO_G;
  }

  double gyr_x() override {
    return 0.;
  }

  double gyr_y() override {
    return 0.;
  }

  double gyr_z() override {
    return 0.;
  }
};

class Altimeter_BME280 final : public SensorAltimeter {
protected:
  Adafruit_BME280 bme;
  TwoWire        &wire;
  uint8_t         addr;
  float           pressure_pa{};
  float           temp_c{};
  float           hum_rh{};

public:
  Altimeter_BME280(TwoWire &wire, const uint8_t addr) : SensorAltimeter(), wire(wire), addr(addr) {
  }

  bool begin() override {
    if (!bme.begin(addr, &wire))
      return false;
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,   // Temperature
                    Adafruit_BME280::SAMPLING_X16,  // Pressure
                    Adafruit_BME280::SAMPLING_X1,   // Humidity
                    Adafruit_BME280::FILTER_X4,
                    Adafruit_BME280::STANDBY_MS_62_5);
    return true;
  }

  bool read() override {
    temp_c      = bme.readTemperature();
    pressure_pa = bme.readPressure();
    hum_rh      = bme.readHumidity();
    return !isnan(temp_c) && !isnan(pressure_pa) && !isnan(hum_rh);
  }

  double pressure_hpa() override {
    return pressure_pa * 0.01;  // Pa -> hPa
  }

  double temperature_c() const {
    return temp_c;
  }

  double humidity_rh() const {
    return hum_rh;
  }
};

class Altimeter_LPS22 final : public SensorAltimeter {
protected:
  Adafruit_LPS22  lps;
  TwoWire        &wire;
  uint8_t         addr;
  sensors_event_t p{}, t{};

public:
  Altimeter_LPS22(TwoWire &wire, const uint8_t addr) : SensorAltimeter(), wire(wire), addr(addr) {
  }

  bool begin() override {
    if (!lps.begin_I2C(addr, &wire))
      return false;
    lps.setDataRate(LPS22_RATE_25_HZ);
    return true;
  }

  bool read() override {
    return lps.getEvent(&p, &t);
  }

  double pressure_hpa() override {
    return p.pressure;  // hPa
  }
};

class Hygrometer_AHT20 final : public SensorBase {
public:
  using Data = SensorEnvData;

protected:
  Adafruit_AHTX0  aht;
  TwoWire        &wire;
  uint8_t         addr;
  sensors_event_t h{}, t{};

public:
  Hygrometer_AHT20(TwoWire &wire, const uint8_t addr) : SensorBase(), wire(wire), addr(addr) {
  }

  bool begin() override {
    return aht.begin(&wire, 0, addr);
  }

  bool read() override {
    return aht.getEvent(&h, &t);
  }

  double temperature_c() const {
    return t.temperature;
  }

  double humidity_rh() const {
    return h.relative_humidity;
  }
};

class GNSS_UBLOX final : public SensorGNSS {
protected:
  SFE_UBLOX_GNSS gnss;
  TwoWire       &wire;
  uint8_t        addr;
  uint8_t        nav_hz;

  uint32_t epoch{};
  uint8_t  sats{};
  double   lat{}, lon{}, alt{};

public:
  GNSS_UBLOX(TwoWire &wire, const uint8_t addr, const uint8_t nav_hz) : SensorGNSS(), wire(wire), addr(addr), nav_hz(nav_hz) {
  }

  bool begin() override {
    if (!gnss.begin(wire, addr))
      return false;
    gnss.setI2COutput(COM_TYPE_UBX);
    gnss.setNavigationFrequency(nav_hz);
    gnss.setAutoPVT(true);  // Non-blocking getPVT()
    return true;
  }

  bool read() override {
    if (!gnss.getPVT())
      return false;
    epoch = gnss.getUnixEpoch();
    sats  = gnss.getSIV();
    lat   = gnss.getLatitude() * 1e-7;      // deg
    lon   = gnss.getLongitude() * 1e-7;     // deg
    alt   = gnss.getAltitudeMSL() * 1e-3;  // mm -> m
    return true;
  }

  uint32_t timestamp_epoch() override {
    return epoch;
  }

  uint8_t siv() override {
    return sats;
  }

  double latitude() override {
    return lat;
  }

  double longitude() override {
    return lon;
  }

  double altitude_msl() override {
    return alt;
  }
};

#endif  //ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
