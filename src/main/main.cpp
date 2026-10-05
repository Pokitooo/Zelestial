/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>  // Arduino Framework
// #include <File_Utility.h>  // File Utility (SD card disabled)
#include <LibAvionics.h>      // Base Avionics Library and Utilities
#include "SystemFunctions.h"  // Function Declarations
#include "custom_kalman.h"    // Kalman Quick Table

#if __has_include("STM32FreeRTOS.h")
#  define USE_FREERTOS 1
#  include "hal_rtos.h"
#endif

// #include <STM32SD.h>     // SD card disabled
// #include <STM32Servo.h>  // Servo disabled: PB6 is now SENSOR_SCL
#include <vt_lora>  // EBYTE LoRa
/* END INCLUDE SYSTEM LIBRARIES */

/* BEGIN INCLUDE USER'S IMPLEMENTATIONS */
#include "UserConfig.h"
#include "UserPins.h"     // User's Pins Mapping
#include "UserSensors.h"  // User's Hardware Implementations
#include "UserFSM.h"      // User's FSM States
/* END INCLUDE USER'S IMPLEMENTATIONS */

/* BEGIN INCLUDE MAIN */
#include "./main.h"
/* END INCLUDE MAIN */

/* BEGIN USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */
/* END USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */

/* BEGIN BUS INSTANCES */
// Wire                            // Sensor I2C (SENSOR_SDA, SENSOR_SCL)
TwoWire        WireGPS(GPS_SDA, GPS_SCL);   // GNSS I2C
HardwareSerial SerialLoRa(E22_RX, E22_TX);  // E22 UART
/* END BUS INSTANCES */

/* BEGIN SENSOR INSTANCES */
SensorIMU *imu[RA_NUM_IMU] = {
  new IMU_MPU6050(Wire, MPU6050_ADDR),  // IMU #1
  new IMU_LIS3DH(Wire, LIS3DH_ADDR),    // IMU #2
};
Altimeter_BME280 *bme280                      = new Altimeter_BME280(Wire, BME280_ADDR);
SensorAltimeter  *altimeter[RA_NUM_ALTIMETER] = {
  bme280,                                 // Altimeter #1
  new Altimeter_LPS22(Wire, LPS22_ADDR),  // Altimeter #2
};
Hygrometer_AHT20 *hygrometer        = new Hygrometer_AHT20(Wire, AHT20_ADDR);
SensorGNSS       *gnss[RA_NUM_GNSS] = {
  new GNSS_UBLOX(WireGPS, GNSS_ADDR, 1000ul / RA_INTERVAL_GNSS_READING),  // GNSS #1
};
/* END SENSOR INSTANCES */

/* BEGIN COMMUNICATION INSTANCES */
vt::lora_e22 lora(SerialLoRa, E22_M0, E22_M1);
/* END COMMUNICATION INSTANCES */

/* BEGIN SENSOR STATUSES */
struct SensorsHealth {
  SensorStatus imu[RA_NUM_IMU]{};
  SensorStatus altimeter[RA_NUM_ALTIMETER]{};
  SensorStatus gnss[RA_NUM_GNSS]{};
  SensorStatus hygrometer{};
} sensors_health;
/* END SENSOR STATUSES */

/* BEGIN PERSISTENT STATE */
UserFSM  fsm;
double   acc;
double   alt_ref;  // Altitude at ground
double   alt_agl;  // Altitude above ground
double   apogee_raw;
uint32_t seq_no{};
/* END PERSISTENT STATE */

/* BEGIN DATA MEMORY */
struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];
  SensorGNSS::Data      gnss[RA_NUM_GNSS];
  SensorEnvData         bme280;
  SensorEnvData         hygrometer;
} data;
String sd_buf;
String tx_buf;
String debug_buf;
/* END DATA MEMORY */

/* BEGIN SD CARD */
// FsUtil fs_sd;  // SD card disabled
/* END SD CARD */

/* BEGIN FILTERS */
xcore::vdt<FILTER_ORDER - 1> vdt(static_cast<double>(RA_INTERVAL_FSM_EVAL) * 0.001);
Filter1T                     filter_acc;
Filter1T                     filter_alt;
/* END FILTERS */

/* BEGIN ACTUATORS */
// Servo disabled: USER_GPIO_SERVO_A (PB6) is now SENSOR_SCL
// STM32ServoList servos(TIMER_SERVO);
float pos_a = RA_SERVO_A_LOCK;
float pos_b = 90;
/* END ACTUATORS */

/* BEGIN USER PRIVATE VARIABLES */
// hal::rtos::mutex_t mtx_sdio;  // SD card disabled
hal::rtos::mutex_t mtx_i2c;      // Sensor I2C bus (Wire)
hal::rtos::mutex_t mtx_i2c_gps;  // GNSS I2C bus (WireGPS)
hal::rtos::mutex_t mtx_cdc;
/* END USER PRIVATE VARIABLES */

/* BEGIN USER PRIVATE FUNCTIONS */
uint32_t LoggerInterval() {
  switch (fsm.state()) {
    case UserState::STARTUP:
    case UserState::IDLE_SAFE:
      return RA_SDLOGGER_INTERVAL_IDLE;

    case UserState::ARMED:
    case UserState::PAD_PREOP:
      return RA_SDLOGGER_INTERVAL_SLOW;

    case UserState::POWERED:
    case UserState::COASTING:
      return RA_SDLOGGER_INTERVAL_REALTIME;

    case UserState::DROGUE_DEPLOY:
    case UserState::DROGUE_DESCEND:
    case UserState::MAIN_DEPLOY:
    case UserState::MAIN_DESCEND:
      return RA_SDLOGGER_INTERVAL_FAST;

    case UserState::LANDED:
    case UserState::RECOVERED_SAFE:
    default:
      return RA_SDLOGGER_INTERVAL_IDLE;
  }
}
void PrintInitStatus(const char *name, const SensorStatus status) {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.print("[Init] ");
    Serial.print(name);
    Serial.print(": ");
    Serial.println(status == SensorStatus::SENSOR_OK ? "OK" : "FAILED");
  }
}

// USART2 is alive only while its APB1 clock is on and CR1.UE is set (a dead/ungated USART reads SR=CR1=0)
bool LoRaUartAlive() {
  return (RCC->APB1ENR & RCC_APB1ENR_USART2EN) && (USART2->CR1 & USART_CR1_UE);
}

// Debug: report the E22 UART state at a given point, to find what switches USART2 off
void LoRaUartProbe(const char *where) {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.print("[LoRa UART] ");
    Serial.print(where);
    Serial.print(": clk=");
    Serial.print((RCC->APB1ENR & RCC_APB1ENR_USART2EN) ? 1 : 0);
    Serial.print(" SR=0x");
    Serial.print(USART2->SR, HEX);
    Serial.print(" CR1=0x");
    Serial.println(USART2->CR1, HEX);
  }
}

// Polled, interrupt-free TX straight into the E22's USART (USART2 on PA2/PA3).
// HardwareSerial::write() relies on the TX-complete interrupt to drain its 64-byte buffer and spins
// forever when that stalls; here the task sleeps between bytes and gives up instead of hanging.
bool LoRaWrite(const uint8_t *buf, const size_t len) {
  constexpr uint32_t BYTE_TIMEOUT_MS = 50;  // One byte takes ~1 ms at 9600 baud

  for (size_t i = 0; i < len; ++i) {
    const uint32_t t0 = millis();
    while (!(USART2->SR & USART_SR_TXE)) {
      if (millis() - t0 >= BYTE_TIMEOUT_MS)
        return false;
      hal::rtos::delay_ms(1);
    }
    USART2->DR = buf[i];
  }
  return true;
}
/* END USER PRIVATE FUNCTIONS */

/* BEGIN USER SETUP */
void UserSetupGPIO() {
  if constexpr (RA_LED_ENABLED) {
    pinMode(USER_GPIO_LED, OUTPUT);
  }
}

void UserSetupActuator() {
  // Servo disabled: USER_GPIO_SERVO_A (PB6) is now SENSOR_SCL
  // servos.attach(USER_GPIO_SERVO_A, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
}

void UserSetupCDC() {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.begin();

    // Wait (bounded) for the USB host so the init messages are not lost
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < RA_USB_WAIT_TIMEOUT_MS) {
    }
  }
}

void UserSetupI2C() {
  Wire.setSDA(SENSOR_SDA);
  Wire.setSCL(SENSOR_SCL);
  Wire.begin();
  Wire.setClock(SENSOR_I2C_SPEED);

  // Release the GNSS from reset (active low)
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);

  WireGPS.begin();
  WireGPS.setClock(GPS_I2C_SPEED);
}

void UserSetupUSART() {
  // Same sequence as the vt-lora-ebyte example: config -> set_param -> query_param -> begin
  delay(1000);
  Serial.println("[LoRa] Beginning configuration mode...");
  lora.config();
  delay(1000);
  Serial.println("[LoRa] Entered configuration mode!");

  lora.set_param(E22_ADDRESS,
                 RA_LORA_NET_ID,
                 E22_BAUDRATE,
                 vt::LoRaParity::PARITY_8N1,
                 RA_LORA_AIR_DATA_RATE,
                 E22_CHANNEL,
                 RA_LORA_PACKET_LENGTH,
                 /*save_params*/ true);

  // REG3: enable fixed-point transmission (the library leaves it transparent)
  // constexpr uint8_t E22_REG3_FIXED_TX = 0x40;
  // const uint8_t     cmd_reg3[]        = {0xC0, 0x06, 0x01, E22_REG3_FIXED_TX};
  // lora.hw_serial().write(cmd_reg3, sizeof(cmd_reg3));
  delay(50);

  Serial.print("[LoRa] Module replies: ");
  lora.query_param();  // Prints every byte the module sent back; blocks if it sent nothing
  Serial.println();

  Serial.println("[LoRa] Done config!");

  // Normal (transmit) mode
  lora.begin(E22_BAUDRATE);
  LoRaUartProbe("after lora.begin");
}
/* END USER SETUP */

/* BEGIN USER THREADS */
void CB_ReadIMU(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_IMU_READING, [&]() -> void {
    mtx_i2c.exec(ReadIMU);

    const double &ax = data.imu[0].acc_x;
    const double &ay = data.imu[0].acc_y;
    const double &az = data.imu[0].acc_z;

    // Total acceleration
    acc = std::sqrt(std::abs(ax * ax) + std::abs(ay * ay) + std::abs(az * az));

    // Compensate for gravity
    acc = acc - 1.0;

    // Update KF with measurement
    filter_acc.kf.update({acc});
  });
}

void CB_ReadAltimeter(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_ALTIMETER_READING, [&]() -> void {
    mtx_i2c.exec(ReadAltimeter);

    // Update KF with measurement
    filter_alt.kf.update({data.altimeter[0].altitude_m});

    // Update altitude above ground
    alt_agl = data.altimeter[0].altitude_m - alt_ref;

    // Update apogee
    if (alt_agl > apogee_raw)
      apogee_raw = alt_agl;
  });
}

void CB_ReadHygrometer(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_HYGROMETER_READING, [&]() -> void {
    mtx_i2c.exec(ReadHygrometer);
  });
}

void CB_ReadGNSS(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_GNSS_READING, [&]() -> void {
    mtx_i2c_gps.exec(ReadGNSS);
  });
}

void CB_EvalFSM(void *) {
  uint32_t true_interval;
  hal::rtos::interval_loop(RA_INTERVAL_FSM_EVAL, true_interval, [&]() -> void {
    const uint32_t delta_interval = true_interval < RA_INTERVAL_FSM_EVAL
                                      ? RA_INTERVAL_FSM_EVAL - true_interval
                                      : true_interval - RA_INTERVAL_FSM_EVAL;

    if (true_interval != 0 &&                             // Excluding the first run
        delta_interval > RA_JITTER_TOLERANCE_FSM_EVAL) {  // If tick jitter is too much
      // Update dt for KF
      vdt.update_dt(static_cast<double>(true_interval) * 0.001);

      // Regenerate F with the new dt
      filter_acc.F = vdt.generate_F();
      filter_alt.F = vdt.generate_F();
    }

    // Predict states to "now"
    filter_acc.kf.predict();
    filter_alt.kf.predict();

    // FSM with predicted states
    EvalFSM();
  });
}

void CB_AutoZeroAlt(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_AUTOZERO, [&]() -> void {
    AutoZeroAlt();
  });
}

void CB_ConstructData(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_CONSTRUCT, [&]() -> void {
    sd_buf = "";
    csv_stream_lf(sd_buf)
      << "MFC"
      << seq_no
      << millis()
      << state_string(fsm.state())

      << data.imu[0].acc_x
      << data.imu[0].acc_y
      << data.imu[0].acc_z
      << acc
      << filter_acc.kf.state()  // ACC

      << filter_alt.kf.state_vector()[1]  // VEL
      << filter_alt.kf.state_vector()[0]  // POS
      << data.altimeter[0].altitude_m
      << data.altimeter[0].pressure_hpa
      << alt_agl
      << alt_ref
      << apogee_raw

      << data.imu[1].acc_x
      << data.imu[1].acc_y
      << data.imu[1].acc_z
      << data.imu[0].gyr_x
      << data.imu[0].gyr_y
      << data.imu[0].gyr_z
      << data.altimeter[1].altitude_m
      << data.altimeter[1].pressure_hpa
      << data.hygrometer.temperature_c
      << data.hygrometer.humidity_rh
      << data.gnss[0].timestamp_epoch
      << data.gnss[0].siv
      << String(data.gnss[0].latitude, 7)
      << String(data.gnss[0].longitude, 7)
      << data.gnss[0].altitude_msl
      //
      ;
  });
}

void CB_Telemetry(void *) {
  tx_buf.reserve(RA_LORA_PACKET_LENGTH);

  hal::rtos::interval_loop(TELEMETRY_PERIOD_MS, [&]() -> void {
    tx_buf = "";
    csv_stream(tx_buf)
      << TEAM_ID
      << seq_no
      << millis()
      << state_string(fsm.state())

      << filter_alt.kf.state_vector()[0]  // POS
      << filter_alt.kf.state_vector()[1]  // VEL
      << data.altimeter[0].pressure_hpa   // BME280
      << data.bme280.temperature_c
      << data.bme280.humidity_rh
      << data.altimeter[1].pressure_hpa  // LPS22

      << data.imu[0].acc_x  // MPU6050
      << data.imu[0].acc_y
      << data.imu[0].acc_z
      << data.imu[0].gyr_x
      << data.imu[0].gyr_y
      << data.imu[0].gyr_z
      << data.imu[1].acc_x  // LIS3DH
      << data.imu[1].acc_y
      << data.imu[1].acc_z

      << data.hygrometer.temperature_c  // AHT20
      << data.hygrometer.humidity_rh

      << data.gnss[0].siv
      << String(data.gnss[0].latitude, 7)
      << String(data.gnss[0].longitude, 7)
      << data.gnss[0].altitude_msl
      //
      ;

    Serial.println(tx_buf);
    // Transparent transmission, same bytes as SerialLoRa.println(tx_buf) but without busy-waiting
    tx_buf += "\r\n";
    static bool first_tick = true;
    if (first_tick) {
      first_tick = false;
      mtx_cdc.exec([&]() -> void {
        LoRaUartProbe("first telemetry tick");
      });
    }

    // Something switched USART2 off since setup: bring it back (also restores M0/M1 = normal mode)
    if (!LoRaUartAlive()) {
      lora.begin(E22_BAUDRATE);
      mtx_cdc.exec([&]() -> void {
        LoRaUartProbe("USART2 was off, re-initialized");
      });
    }

    const bool sent = LoRaWrite(reinterpret_cast<const uint8_t *>(tx_buf.c_str()), tx_buf.length());
    if (sent)
      ++seq_no;

    mtx_cdc.exec([&]() -> void {
      if (sent) {
        Serial.println("LORA SENT");
      } else {
        Serial.print("LORA TX STALLED, USART2 SR=0x");
        Serial.print(USART2->SR, HEX);
        Serial.print(" CR1=0x");
        Serial.println(USART2->CR1, HEX);
      }
    });
  });
}

/*
void CB_SDLogger(void *) {
  hal::rtos::interval_loop(LoggerInterval(), LoggerInterval, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file() << sd_buf;
    });
  });
}

void CB_SDSave(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file().flush();
    });
  });
}
*/

void CB_DebugLogger(void *) {
  debug_buf.reserve(1024);

  const auto line = [](const char *label, const double value, const char *unit, const unsigned digits = 2) -> void {
    debug_buf += label;
    debug_buf += ": ";
    debug_buf += String(value, digits);
    debug_buf += unit;
    debug_buf += "\r\n";
  };

  hal::rtos::interval_loop(RA_INTERVAL_DEBUG_PRINT, [&]() -> void {
    debug_buf = "";
    debug_buf += "========================================\r\n";
    debug_buf += "Packets sent: ";
    debug_buf += seq_no;
    debug_buf += " | Time: ";
    debug_buf += millis();
    debug_buf += " ms | State: ";
    debug_buf += state_string(fsm.state());
    debug_buf += "\r\n";

    line("Temperature bme", data.bme280.temperature_c, " C");
    line("Humidity bme", data.bme280.humidity_rh, " %");
    line("Pressure bme", data.altimeter[0].pressure_hpa, " hPa");
    line("Altitude bme", data.altimeter[0].altitude_m, " m");

    line("Pressure lps22", data.altimeter[1].pressure_hpa, " hPa");
    line("Altitude lps22", data.altimeter[1].altitude_m, " m");

    line("Temperature aht20", data.hygrometer.temperature_c, " C");
    line("Humidity aht20", data.hygrometer.humidity_rh, " %");

    line("Accel X mpu6050", data.imu[0].acc_x, " g", 3);
    line("Accel Y mpu6050", data.imu[0].acc_y, " g", 3);
    line("Accel Z mpu6050", data.imu[0].acc_z, " g", 3);
    line("Gyro X mpu6050", data.imu[0].gyr_x, " rad/s", 3);
    line("Gyro Y mpu6050", data.imu[0].gyr_y, " rad/s", 3);
    line("Gyro Z mpu6050", data.imu[0].gyr_z, " rad/s", 3);

    line("Accel X lis3dh", data.imu[1].acc_x, " g", 3);
    line("Accel Y lis3dh", data.imu[1].acc_y, " g", 3);
    line("Accel Z lis3dh", data.imu[1].acc_z, " g", 3);

    line("Satellites gnss", data.gnss[0].siv, "", 0);
    line("Latitude gnss", data.gnss[0].latitude, " deg", 7);
    line("Longitude gnss", data.gnss[0].longitude, " deg", 7);
    line("Altitude gnss", data.gnss[0].altitude_msl, " m");

    line("Altitude filtered", filter_alt.kf.state_vector()[0], " m");
    line("Velocity filtered", filter_alt.kf.state_vector()[1], " m/s");
    line("Altitude AGL", alt_agl, " m");

    mtx_cdc.exec([&]() -> void {
      Serial.print(debug_buf);
    });
  });
}

void CB_RetainDeployment(void *) {
  hal::rtos::interval_loop(15ul, [&]() -> void {
    RetainDeployment();
  });
}

/* END USER THREADS */

void UserThreads() {
  hal::rtos::scheduler.create(CB_EvalFSM, {.name = "CB_EvalFSM", .stack_size = 8192, .priority = osPriorityRealtime});

  hal::rtos::scheduler.create(CB_ReadIMU, {.name = "CB_ReadIMU", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.name = "CB_ReadAltimeter", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadHygrometer, {.name = "CB_ReadHygrometer", .stack_size = 4096, .priority = osPriorityAboveNormal});
  hal::rtos::scheduler.create(CB_ReadGNSS, {.name = "CB_ReadGNSS", .stack_size = 8192, .priority = osPriorityAboveNormal});

  if constexpr (RA_RETAIN_DEPLOYMENT_ENABLED)
    hal::rtos::scheduler.create(CB_RetainDeployment, {.name = "CB_RetainDeployment", .stack_size = 4096, .priority = osPriorityHigh});

  if constexpr (RA_AUTO_ZERO_ALT_ENABLED)
    hal::rtos::scheduler.create(CB_AutoZeroAlt, {.name = "CB_AutoZeroAlt", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ConstructData, {.name = "CB_ConstructData", .stack_size = 4096, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_Telemetry, {.name = "CB_Telemetry", .stack_size = 8192, .priority = osPriorityNormal});
  // hal::rtos::scheduler.create(CB_SDLogger, {.name = "CB_SDLogger", .stack_size = 8192, .priority = osPriorityNormal});

  if constexpr (RA_USB_DEBUG_ENABLED)
    hal::rtos::scheduler.create(CB_DebugLogger, {.name = "CB_DebugLogger", .stack_size = 8192, .priority = osPriorityBelowNormal});

  // hal::rtos::scheduler.create(CB_SDSave, {.name = "CB_SDSave", .stack_size = 8192, .priority = osPriorityLow});
}

void setup() {
  /* BEGIN STORAGES SETUP */
  // SD card disabled
  // SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1, USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  // SD.setCMD(USER_GPIO_SDIO_CMD);
  // SD.setCK(USER_GPIO_SDIO_CK);
  // SD.begin();
  // fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
  // fs_sd.open_one<FsMode::WRITE>();
  sd_buf.reserve(1024);
  /* END STORAGES SETUP */

  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupGPIO();
  UserSetupActuator();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupI2C();
  LoRaUartProbe("after I2C setup");
  /* END GPIO AND INTERFACES SETUP */

  /* BEGIN FILTERS SETUP */
  filter_alt.F = vdt.generate_F();
  filter_acc.F = vdt.generate_F();
  /* END FILTERS SETUP */

  /* BEGIN SENSORS SETUP */
  // IMU
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (!imu[i])
      sensors_health.imu[i] = SensorStatus::SENSOR_NO;
    else if (imu[i]->begin())
      sensors_health.imu[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.imu[i] = SensorStatus::SENSOR_ERR;
  }
  PrintInitStatus("MPU6050", sensors_health.imu[0]);
  PrintInitStatus("LIS3DH", sensors_health.imu[1]);

  // Altimeter
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (!altimeter[i])
      sensors_health.altimeter[i] = SensorStatus::SENSOR_NO;
    else if (altimeter[i]->begin())
      sensors_health.altimeter[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.altimeter[i] = SensorStatus::SENSOR_ERR;
  }
  PrintInitStatus("BME280", sensors_health.altimeter[0]);
  PrintInitStatus("LPS22", sensors_health.altimeter[1]);

  // GNSS
  for (size_t i = 0; i < RA_NUM_GNSS; ++i) {
    if (!gnss[i])
      sensors_health.gnss[i] = SensorStatus::SENSOR_NO;
    else if (gnss[i]->begin())
      sensors_health.gnss[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.gnss[i] = SensorStatus::SENSOR_ERR;
  }
  PrintInitStatus("GNSS", sensors_health.gnss[0]);

  // Hygrometer
  if (!hygrometer)
    sensors_health.hygrometer = SensorStatus::SENSOR_NO;
  else if (hygrometer->begin())
    sensors_health.hygrometer = SensorStatus::SENSOR_OK;
  else
    sensors_health.hygrometer = SensorStatus::SENSOR_ERR;
  PrintInitStatus("AHT20", sensors_health.hygrometer);
  LoRaUartProbe("after sensors setup");

  /* END SENSORS SETUP */

  /* BEGIN SYSTEM/KERNEL SETUP */
  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
  /* END SYSTEM/KERNEL SETUP */
}

void EvalFSM() {
  static uint32_t                                            state_millis_start   = 0;
  static uint32_t                                            state_millis_elapsed = 0;
  static xcore::sampler_t<2048, double>                      sampler;
  static xcore::sampler_t<RA_MAIN_OVERSPEED_SAMPLES, double> sampler_overspeed;

  switch (fsm.state()) {
    case UserState::STARTUP: {
      // <--- Next: always transfer --->
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_LED, 0);
      }
      fsm.transfer(UserState::IDLE_SAFE);
      break;
    }

    case UserState::IDLE_SAFE: {
      // <--- Next: always transfer (should wait for uplink) --->
      // <--- Wait for startup countdown instead to prevent accidental operations --->
      if (fsm.on_enter()) {  // Run once
        state_millis_start = millis();
      }

      state_millis_elapsed = millis() - state_millis_start;

      if constexpr (RA_STARTUP_COUNTDOWN_ENABLED) {
        if (state_millis_elapsed >= RA_STARTUP_COUNTDOWN)
          fsm.transfer(UserState::ARMED);
      }
      break;
    }

    case UserState::ARMED: {
      // <--- Next: always transfer (should wait for uplink) --->
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_LED, 1);
      }
      fsm.transfer(UserState::PAD_PREOP);
      break;
    }

    case UserState::PAD_PREOP: {
      // !!!! Next: DETECT launch !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LAUNCH_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LAUNCH_ACC, /*recount*/ false);
      }

      // sampler.add_sample(acc);  // Use raw acceleration, unfiltered
      sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration

      if (sampler.is_sampled() &&
          sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO) {
        if constexpr (RA_LED_ENABLED) {
          digitalWrite(USER_GPIO_LED, 0);
        }
        fsm.transfer(UserState::POWERED);
      }
      break;
    }

    case UserState::POWERED: {
      // !!!! Next: DETECT motor burnout !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_BURNOUT_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_BURNOUT_ACC, /*recount*/ false);
        state_millis_start = millis();
      }

      // sampler.add_sample(acc);  // Use raw acceleration, unfiltered
      sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::COASTING);
      break;
    }

    case UserState::COASTING: {
      // !!!! Next: DETECT apogee !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_APOGEE_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_APOGEE_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_APOGEE_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_APOGEE_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::DROGUE_DEPLOY);
      break;
    }

    case UserState::DROGUE_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      ActivateDeployment(/*index*/ 0);
      fsm.transfer(UserState::DROGUE_DESCEND);
      break;
    }

    case UserState::DROGUE_DESCEND: {
      // !!!! Next: DETECT main deployment altitude !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_MAIN_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_MAIN_ALT_COMPENSATED, /*recount*/ false);
        sampler_overspeed.reset();
        sampler_overspeed.set_threshold(RA_MAIN_OVERSPEED_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      sampler.add_sample(alt_agl);
      const double vel = filter_alt.kf.state_vector()[1];
      sampler_overspeed.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      const bool cond_timeout   = state_millis_elapsed >= RA_TIME_TO_MAIN_MAX;
      const bool cond_min_time  = state_millis_elapsed >= RA_TIME_TO_MAIN_MIN;
      const bool cond_alt_lower = sampler.is_sampled() &&
                                  sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;
      const bool cond_overspeed = sampler_overspeed.is_sampled() &&
                                  sampler_overspeed.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO;

      if ((cond_timeout) ||
          (cond_min_time && cond_alt_lower) ||
          (cond_min_time && cond_overspeed))
        fsm.transfer(UserState::MAIN_DEPLOY);

      break;
    }

    case UserState::MAIN_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      // ActivateDeployment(/*index*/ 1);
      fsm.transfer(UserState::MAIN_DESCEND);
      break;
    }

    case UserState::MAIN_DESCEND: {
      // !!!! Next: DETECT landing !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LANDED_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LANDED_VEL, /*recount*/ false);
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::LANDED);
      break;
    }

    case UserState::LANDED: {
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_LED, 1);
      }
      break;
    }

    case UserState::RECOVERED_SAFE: {
      break;
    }
  }
}

void ReadIMU() {
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (sensors_health.imu[i] != SensorStatus::SENSOR_OK ||
        !imu[i]->read())
      continue;
    data.imu[i].acc_x = imu[i]->acc_x();
    data.imu[i].acc_y = imu[i]->acc_y();
    data.imu[i].acc_z = imu[i]->acc_z();
    data.imu[i].gyr_x = imu[i]->gyr_x();
    data.imu[i].gyr_y = imu[i]->gyr_y();
    data.imu[i].gyr_z = imu[i]->gyr_z();
  }
}

void ReadAltimeter() {
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (sensors_health.altimeter[i] != SensorStatus::SENSOR_OK ||
        !altimeter[i]->read())
      continue;
    data.altimeter[i].pressure_hpa = altimeter[i]->pressure_hpa();
    data.altimeter[i].altitude_m   = altimeter[i]->altitude_m();
  }

  // BME280 also measures temperature and humidity
  if (sensors_health.altimeter[0] == SensorStatus::SENSOR_OK) {
    data.bme280.temperature_c = bme280->temperature_c();
    data.bme280.humidity_rh   = bme280->humidity_rh();
  }
}

void ReadGNSS() {
  for (size_t i = 0; i < RA_NUM_GNSS; ++i) {
    if (sensors_health.gnss[i] != SensorStatus::SENSOR_OK ||
        !gnss[i]->read())
      continue;
    data.gnss[i].timestamp_epoch = gnss[i]->timestamp_epoch();
    data.gnss[i].siv             = gnss[i]->siv();
    data.gnss[i].latitude        = gnss[i]->latitude();
    data.gnss[i].longitude       = gnss[i]->longitude();
    data.gnss[i].altitude_msl    = gnss[i]->altitude_msl();
  }
}

void ReadHygrometer() {
  if (sensors_health.hygrometer != SensorStatus::SENSOR_OK ||
      !hygrometer->read())
    return;
  data.hygrometer.temperature_c = hygrometer->temperature_c();
  data.hygrometer.humidity_rh   = hygrometer->humidity_rh();
}

void ActivateDeployment(const size_t index) {
  switch (index) {
    case 0: {  // Drogue/First Deployment
      pos_a = RA_SERVO_A_RELEASE;
      // servos[0].write(pos_a);  // Servo disabled
      break;
    }

    case 1: {  // Main/Second Deployment
      break;
    }

    default:
      break;
  }
}

void RetainDeployment() {
  // servos[0].write(pos_a);  // Servo disabled
}

void AutoZeroAlt() {
  static xcore::sampler_t<RA_AUTOZERO_SAMPLES, double> sampler;
  sampler.set_threshold(RA_AUTOZERO_VEL, /*recount*/ false);

  switch (fsm.state()) {
    case UserState::IDLE_SAFE:
    case UserState::ARMED:
    case UserState::PAD_PREOP: {
      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));
      if (sampler.is_sampled()) {
        if (sampler.under_by_over<double>() > 3.0)  // 75%
          alt_ref = data.altimeter[0].altitude_m;
        sampler.reset();
      }
      break;
    }
    default:
      break;
  }
}
