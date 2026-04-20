#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <string.h>

// ============ НАСТРОЙКИ ============
const char* ssid = "sbn-S15C";
const char* password = "umka2012";
#define SYSTEM_ID 1
#define COMPONENT_ID 1
// ============ КОНЕЦ НАСТРОЕК ============

WiFiUDP udp;

// Данные для симуляции дрона
float fake_lat = 55.7558;
float fake_lon = 37.6173;
float fake_alt = 150.0;
float fake_groundspeed = 1.0;
float fake_airspeed = 1.5;
float fake_climb = 0.1;
int fake_throttle = 50;

// Параметры батареи
float battery_voltage = 12.6;  // Вольты
float battery_current = 2.5;   // Амперы
int battery_remaining = 80;    // Проценты
int battery_consumed = 0;      // Потреблено в mAh
int16_t battery_temperature = 2500;  // Температура в cdegC (25.0°C * 100)

// Таймеры
unsigned long last_heartbeat_time = 0;
unsigned long last_sys_status_time = 0;
unsigned long last_gps_time = 0;
unsigned long last_vfr_hud_time = 0;
unsigned long last_battery2_time = 0;
unsigned long last_rc_channels_time = 0;
unsigned long last_attitude_time = 0;
unsigned long last_param_send = 0;

// Счетчики для MAVLink
uint32_t custom_mode = 0;
uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
uint8_t system_state = MAV_STATE_STANDBY;

bool armed = false;
unsigned long arming_time = 0;

// Параметры для отправки
struct ParamEntry {
  const char* name;
  float value;
  uint16_t index;
  uint8_t type;
};

// ОСНОВНЫЕ ПАРАМЕТРЫ БАТАРЕИ ДЛЯ PX4
ParamEntry params[] = {
  // Основные параметры батареи
  {"BAT_N_CELLS", 3.0f, 0, MAV_PARAM_TYPE_REAL32},
  {"BAT_V_CHARGED", 4.2f, 1, MAV_PARAM_TYPE_REAL32},
  {"BAT_V_EMPTY", 3.5f, 2, MAV_PARAM_TYPE_REAL32},
  {"BAT_CAPACITY", 3300.0f, 3, MAV_PARAM_TYPE_REAL32},
  {"BAT_V_DIV", 10.0f, 4, MAV_PARAM_TYPE_REAL32},
  {"BAT_A_PER_V", 15.0f, 5, MAV_PARAM_TYPE_REAL32},
  
  // Пороговые значения
  {"BAT_CRIT_THR", 0.10f, 6, MAV_PARAM_TYPE_REAL32},
  {"BAT_LOW_THR", 0.20f, 7, MAV_PARAM_TYPE_REAL32},
  {"BAT_EMERGEN_THR", 0.05f, 8, MAV_PARAM_TYPE_REAL32},
  {"BAT_LOW_ACT", 2.0f, 9, MAV_PARAM_TYPE_REAL32},  // 2 = RTL
  
  // Другие параметры
  {"COM_LOW_BAT_ACT", 2.0f, 10, MAV_PARAM_TYPE_REAL32},
  {"COM_DL_LOSS_EN", 0.0f, 11, MAV_PARAM_TYPE_REAL32},
  {"COM_RC_LOSS_EN", 0.0f, 12, MAV_PARAM_TYPE_REAL32},
  {"NAV_RCL_ACT", 0.0f, 13, MAV_PARAM_TYPE_REAL32},
  
  // Параметры стабилизации
  {"MC_PITCH_P", 6.5f, 14, MAV_PARAM_TYPE_REAL32},
  {"MC_ROLL_P", 6.5f, 15, MAV_PARAM_TYPE_REAL32},
  {"MPC_Z_VEL_P", 1.6f, 16, MAV_PARAM_TYPE_REAL32},
  {"MPC_XY_VEL_P", 2.0f, 17, MAV_PARAM_TYPE_REAL32},
  
  // Калибровка
  {"CAL_ACC0_ID", 1376264.0f, 18, MAV_PARAM_TYPE_REAL32},
  {"CAL_GYRO0_ID", 2293768.0f, 19, MAV_PARAM_TYPE_REAL32},
  {"CAL_MAG0_ID", 196616.0f, 20, MAV_PARAM_TYPE_REAL32},
  
  // Еще параметры для заполнения
  {"SYS_AUTOSTART", 4010.0f, 21, MAV_PARAM_TYPE_REAL32},
  {"SYS_MC_EST_GROUP", 2.0f, 22, MAV_PARAM_TYPE_REAL32},
  {"MAV_TYPE", 2.0f, 23, MAV_PARAM_TYPE_REAL32},
  {"MAV_SYS_ID", 1.0f, 24, MAV_PARAM_TYPE_REAL32},
  {"MAV_COMP_ID", 1.0f, 25, MAV_PARAM_TYPE_REAL32},
};

uint16_t total_params = sizeof(params) / sizeof(params[0]);
uint16_t param_send_index = 0;
bool params_requested = false;

// ============ ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ============

void sendToGround(uint8_t* buf, uint16_t len) {
  udp.beginPacket("255.255.255.255", 14550);
  udp.write(buf, len);
  udp.endPacket();
}

// ============ ФУНКЦИИ ОТПРАВКИ MAVLINK ============

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  uint8_t system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
  uint8_t mav_type = MAV_TYPE_QUADROTOR;
  uint8_t autopilot = MAV_AUTOPILOT_PX4;
  
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                             mav_type, autopilot, base_mode, 
                             custom_mode, system_status);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendSysStatus() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Собираем sensor flags как у реального PX4
  uint32_t onboard_control_sensors_present = 
      MAV_SYS_STATUS_SENSOR_3D_GYRO |
      MAV_SYS_STATUS_SENSOR_3D_ACCEL |
      MAV_SYS_STATUS_SENSOR_3D_MAG |
      MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
      MAV_SYS_STATUS_SENSOR_GPS |
      MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  
  uint32_t onboard_control_sensors_enabled = onboard_control_sensors_present;
  uint32_t onboard_control_sensors_health = onboard_control_sensors_present;
  
  uint16_t voltage = (uint16_t)(battery_voltage * 1000);  // mV
  int16_t current = (int16_t)(battery_current * 100);     // cA
  
  // Рассчитываем потребленную энергию
  static unsigned long last_current_time = millis();
  unsigned long now = millis();
  float dt = (now - last_current_time) / 1000.0f / 3600.0f;  // часы
  battery_consumed += (int)(battery_current * dt * 1000);    // mAh
  last_current_time = now;
  
  // Используем правильное количество аргументов для sys_status
  mavlink_msg_sys_status_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      onboard_control_sensors_present,
      onboard_control_sensors_enabled,
      onboard_control_sensors_health,
      500,                    // load (500 = 50%)
      voltage,                // voltage_battery (mV)
      current,                // current_battery (cA)
      battery_remaining,      // battery_remaining (%)
      0,                      // drop_rate_comm
      0,                      // errors_comm
      0,                      // errors_count1
      0,                      // errors_count2
      0,                      // errors_count3
      0,0,0,0);                     // errors_count4
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendBatteryStatus() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Напряжения ячеек (3S батарея)
  uint16_t voltages[10] = {0};
  voltages[0] = (uint16_t)(battery_voltage / 3 * 1000);  // cell 1
  voltages[1] = (uint16_t)(battery_voltage / 3 * 1000);  // cell 2
  voltages[2] = (uint16_t)(battery_voltage / 3 * 1000);  // cell 3
  
  int16_t current = (int16_t)(battery_current * 100);     // cA
  
  // Проверяем структуру функции в вашей версии MAVLink
  // В большинстве версий аргументы такие:
  mavlink_msg_battery_status_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      0,                            // id
      MAV_BATTERY_TYPE_LIPO,        // battery_type
      MAV_BATTERY_FUNCTION_ALL,     // battery_function
      0,                            // type
      0,          // temperature (cdegC) - уже в cdegC
      1200,                     // voltages (mV) - массив uint16_t
      7500,                      // current_battery (cA)
      battery_consumed,             // current_consumed (mAh)
      80,                           // energy_consumed (mAh)
      battery_remaining,            // battery_remaining (%)
      0,                            // time_remaining
      0,                            // charge_state
      0,                            // voltages_ext
      0                            // mode
      );                           // fault_bitmask
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendGPS() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  uint8_t fix_type = 3;  // 3D fix
  uint8_t satellites_visible = 12;
  
  mavlink_msg_gps_raw_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      millis() * 1000,           // time_usec
      fix_type,                  // fix_type
      (int32_t)(fake_lat * 1e7), // lat
      (int32_t)(fake_lon * 1e7), // lon
      (int32_t)(fake_alt * 1000),// alt (mm)
      10,                        // eph
      10,                        // epv
      100,                       // vel (cm/s)
      0,                         // cog (cdeg)
      satellites_visible,        // satellites_visible
      0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendVfrHud() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  uint16_t heading = (millis() / 100) % 360;
  
  mavlink_msg_vfr_hud_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      fake_airspeed,      // airspeed
      fake_groundspeed,   // groundspeed
      heading,            // heading
      fake_throttle,      // throttle
      fake_alt,           // alt
      fake_climb);        // climb
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendAttitude() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  float time_sec = millis() / 1000.0f;
  float roll = 0.1f * sin(time_sec * 0.5f);
  float pitch = 0.1f * cos(time_sec * 0.5f);
  float yaw = fmod(time_sec * 0.3f, 2 * PI);
  
  mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      millis() * 1000,    // time_boot_ms
      roll,               // roll
      pitch,              // pitch
      yaw,                // yaw
      0.01f,              // rollspeed
      0.01f,              // pitchspeed
      0.05f);             // yawspeed
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendRCChannels() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  uint16_t channels[18] = {0};
  channels[0] = 1500;  // roll
  channels[1] = 1500;  // pitch
  channels[2] = 1100;  // throttle (низкий)
  channels[3] = 1500;  // yaw
  channels[4] = 1000;  // mode (стабилизация)
  
  // Используем правильное количество аргументов для rc_channels_raw
  mavlink_msg_rc_channels_raw_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      millis() * 1000,    // time_boot_ms
      0,                  // port
      channels[0],        // chan1_raw
      channels[1],        // chan2_raw
      channels[2],        // chan3_raw
      channels[3],        // chan4_raw
      channels[4],        // chan5_raw
      channels[5],        // chan6_raw
      channels[6],        // chan7_raw
      channels[7],        // chan8_raw
      8);                 // rssi
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

void sendParamValue(uint16_t index) {
  if (index >= total_params) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  ParamEntry param = params[index];
  
  mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &msg,
      param.name,    // param_id
      param.value,   // param_value
      param.type,    // param_type
      total_params,  // param_count
      index);        // param_index
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendToGround(buf, len);
}

// ============ ОБРАБОТКА ВХОДЯЩИХ СООБЩЕНИЙ ============

void handleMavlinkMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(msg, &cmd);
      
      if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        armed = (cmd.param1 > 0.5);
        arming_time = millis();
        
        if (armed) {
          base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
          Serial.println(">> ARMED");
        } else {
          base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
          Serial.println(">> DISARMED");
        }
        
        // Отправляем ACK
        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
            cmd.command, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
        
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &ack);
        sendToGround(buf, len);
      }
      break;
    }
    
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      params_requested = true;
      param_send_index = 0;
      Serial.println(">> PARAM REQUEST LIST");
      break;
    }
    
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      mavlink_param_request_read_t req;
      mavlink_msg_param_request_read_decode(msg, &req);
      
      if (req.param_index != -1) {
        sendParamValue(req.param_index);
        Serial.printf(">> PARAM REQUEST INDEX: %d\n", req.param_index);
      }
      break;
    }
  }
}

void handleMavlinkMessages() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = udp.read(buffer, MAVLINK_MAX_PACKET_LEN);
    
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
        handleMavlinkMessage(&msg);
      }
    }
  }
}

// ============ ОСНОВНЫЕ ФУНКЦИИ ============

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== PX4 СИМУЛЯТОР БПЛА ESP32 ===");
  Serial.print("Подключение к Wi-Fi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n✓ Wi-Fi подключен!");
  Serial.print("IP адрес: ");
  Serial.println(WiFi.localIP());
  
  udp.begin(14550);
  Serial.println("✓ UDP сервер на порту 14550");
  Serial.print("✓ Параметров: ");
  Serial.println(total_params);
  Serial.println("===============================\n");
}

void loop() {
  unsigned long current_time = millis();
  
  // Обработка входящих сообщений
  handleMavlinkMessages();
  
  // Отправка параметров по запросу
  if (params_requested && param_send_index < total_params) {
    if (current_time - last_param_send > 100) {
      sendParamValue(param_send_index);
      param_send_index++;
      last_param_send = current_time;
      
      if (param_send_index >= total_params) {
        params_requested = false;
        Serial.println(">> Все параметры отправлены");
      }
    }
  }
  
  // Регулярная отправка телеметрии
  if (current_time - last_heartbeat_time > 1000) {
    sendHeartbeat();
    last_heartbeat_time = current_time;
  }
  
  if (current_time - last_sys_status_time > 1000) {
    sendSysStatus();
    last_sys_status_time = current_time;
  }
  
  if (current_time - last_battery2_time > 2000) {
    sendBatteryStatus();
    last_battery2_time = current_time;
    
    // Вывод в сериал
    Serial.printf("BAT: %.1fV, %.1fA, %d%%, %dmAh\n", 
                  battery_voltage, battery_current, 
                  battery_remaining, battery_consumed);
  }
  
  if (current_time - last_gps_time > 200) {
    sendGPS();
    last_gps_time = current_time;
  }
  
  if (current_time - last_vfr_hud_time > 200) {
    sendVfrHud();
    last_vfr_hud_time = current_time;
  }
  
  if (current_time - last_attitude_time > 100) {
    sendAttitude();
    last_attitude_time = current_time;
  }
  
  if (current_time - last_rc_channels_time > 500) {
    sendRCChannels();
    last_rc_channels_time = current_time;
  }
  
  // Обновление динамических данных
  float time_sec = current_time / 1000.0f;
  
  // Движение по кругу
  fake_lat = 55.7558 + 0.001 * cos(time_sec * 0.3);
  fake_lon = 37.6173 + 0.001 * sin(time_sec * 0.3);
  fake_alt = 150.0 + 20.0 * sin(time_sec * 0.4);
  
  // Скорости
  fake_groundspeed = 1.0 + 0.5 * sin(time_sec * 0.2);
  fake_airspeed = fake_groundspeed + 0.5;
  fake_climb = 2.0 * cos(time_sec * 0.4);
  fake_throttle = armed ? 50 + int(20 * sin(time_sec * 0.3)) : 0;
  
  // Разряд батареи
  if (armed && current_time - arming_time > 1000) {
    float discharge_rate = 0.05f;  // 5% в минуту при полете
    battery_remaining = max(0, 80 - int((current_time - arming_time) / 60000.0f * discharge_rate * 100));
    
    // Напряжение падает с разрядом
    battery_voltage = 12.6 - (80 - battery_remaining) * 0.03;
    if (battery_voltage < 10.5) battery_voltage = 10.5;
    
    // Ток зависит от режима
    battery_current = 2.5 + 0.5 * sin(time_sec * 0.5);
  } else if (!armed) {
    battery_voltage = 12.6;
    battery_current = 0.1;
  }
  
  delay(10);
}