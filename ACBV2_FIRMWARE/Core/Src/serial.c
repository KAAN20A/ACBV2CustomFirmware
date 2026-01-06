#include "serial.h"

void process_serial_message(char *cmd) {

  if ((uint8_t)cmd[0] < 0x20) {
    uint8_t cmd_id = (uint8_t)cmd[0];
    uint16_t arg = (uint8_t)cmd[1] | ((uint8_t)cmd[2] << 8);
    switch (cmd_id) {
    case 0x01:
      handle_set_position((float)arg);
      break;
    case 0x02:
      handle_set_velocity((float)arg);
      break;
    case 0x03:
      handle_set_torque((float)arg);
      break;
    case 0x04:
      handle_get_position();
      break;
    case 0x05:
      handle_get_velocity();
      break;
    case 0x06:
      handle_get_torque();
      break;
    case 0x07:
      handle_enable();
      break;
    case 0x08:
      handle_disable();
      break;
    case 0x09:
      handle_home();
      break;
    case 0x0A:
      handle_stop();
      break;
    case 0x0B:
      handle_reset_position();
      break;
    case 0x0C:
      handle_get_current_a();
      break;
    case 0x0D:
      handle_get_current_b();
      break;
    case 0x0E:
      handle_get_current_c();
      break;
    case 0xAB:
      handle_cmd_mode((int)arg);
      break;
    case 0xAC:
      handle_reset_config_defaults();
      break;
    case 0xAD:
      handle_reset();
      break;
    case 0xAE:
      handle_help();
      break;
    case 0xAF:
      handle_get_version();
      break;
    case 0xB0:
      handle_get_min_angle();
      break;
    case 0xB1:
      handle_set_min_angle((float)arg);
      break;
    case 0xB2:
      handle_get_max_angle();
      break;
    case 0xB3:
      handle_set_max_angle((float)arg);
      break;
    case 0xB4:
      handle_get_absolute_angle_calibration();
      break;
    case 0xB5:
      handle_set_absolute_angle_calibration((float)arg);
      break;
    }
    return;
  }

  while (*cmd == ' ')
    cmd++;

  if (strncmp(cmd, "set_position ", 13) == 0) {
    float pos = atof(cmd + 13);
    handle_set_position(pos);
    return;
  }
  if (strncmp(cmd, "set_velocity ", 13) == 0) {
    float vel = atof(cmd + 13);
    handle_set_velocity(vel);
    return;
  }
  if (strcmp(cmd, "enable") == 0) {
    handle_enable();
    return;
  }
  if (strcmp(cmd, "disable") == 0) {
    handle_disable();
    return;
  }
  if (strcmp(cmd, "get_position") == 0) {
    handle_get_position();
    return;
  }
  if (strcmp(cmd, "get_velocity") == 0) {
    handle_get_velocity();
    return;
  }
  if (strcmp(cmd, "help") == 0) {
    handle_help();
    return;
  }
}

char msg[] = "unknown command\r\n";
HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

void UART_SendFloat(float value) {
  char buffer[32];
  int len = snprintf(buffer, sizeof(buffer), "%.3f\r\n", value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void UART_SendString(const char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
void handle_set_position(float position) { FOC_Position(position); }
void handle_set_velocity(float velocity) { FOC_Velocity(velocity); }
void handle_set_torque(float torque) { FOC_Torque(torque); }
void handle_get_position(void) {

  UART_SendString("Position: ");

  UART_SendFloat(focParam.position);
}
void handle_get_velocity(void) {

  UART_SendString("Velocity: ");

  UART_SendFloat(focParam.velocity);
}

void handle_get_torque(void) {

  UART_SendString("Torque: ");

  UART_SendFloat(focParam.torque);
}

void handle_get_temperature() {
  UART_SendString("Temprature: ");

  focParam.temprature = float calculateTempreture();
  UART_SendFloat(focParam.temprature);
}
}
void handle_get_bus_voltage() {
  UART_SendString("Bus_Voltage: ");

  focParam.bus_voltage = float calculateBus_Voltage();
  UART_SendFloat(focParam.bus_voltage);
}
}
void handle_get_internal_temperature() {
  UART_SendString("Internal_Tempreture: ");

  focParam.internal_temprature = float calculateInternal_Temprature();
  UART_SendFloat(focParam.internal_tempreture);
}
}

void handle_get_current_a() {
  UART_SendString("Velocity: ");

  UART_SendFloat(focParam.velocity);
}

void handle_get_current_b() {
  UART_SendString("Velocity: ");

  UART_SendFloat(focParam.velocity);
}
void handle_get_current_c() {
  UART_SendString("Velocity: ");

  UART_SendFloat(focParam.velocity);
}

void handle_reset() { NVIC_System_Reset(); }

void handle_get_min_angle() {
  UART_SendString("Min_Angle: ");

  UART_SendFloat(focParam.min_angle);
}

void handle_set_min_angle(float min_angle) { focParam.min_angle = min_angle; }
void handle_get_max_angle() {
  UART_SendString("Max_Angle: ");

  UART_SendFloat(focParam.max_angle);
}

void handle_set_max_angle(float max_angle) { focParam.min_angle = min_angle; }
void handle_get_velocity_pid() {
  UART_SendString("VelocityP: ");
  UART_SendFloat(focParam.pid_vel_P);
  UART_SendString("VelocityI: ");
  UART_SendFloat(focParam.pid_vel_I);
  UART_SendString("VelocityD: ");
  UART_SendFloat(focParam.pid_vel_D);
}

void handle_set_velocity_pid(float p, float i, float d) {
  focParam.pid_vel_P = p;
  focParam.pid_vel_I = i;
  focParam.pid_vel_D = d;
}
void handle_get_angle_pid() {
  UART_SendString("AngleyP: ");
  UART_SendFloat(focParam.pid_angle_P);
  UART_SendString("AngleyI: ");
  UART_SendFloat(focParam.pid_angle_I);
  UART_SendString("AngleyD: ");
  UART_SendFloat(focParam.pid_angle_D);
}
void handle_set_angle_pid(float p, float i, float d) {
  focParam.pid_angle_P = p;
  focParam.pid_angle_I = i;
  focParam.pid_angle_D = d;
}
void handle_get_current_pid() {
  UART_SendString("TorqueP: ");
  UART_SendFloat(focParam.pid_torque_P);
  UART_SendString("TorqueI: ");
  UART_SendFloat(focParam.pid_torque_I);
  UART_SendString("TorqueD: ");
  UART_SendFloat(focParam.pid_torque_D)
}
void handle_set_current_pid(float p, float i, float d) {
  focParam.pid_torque_P = p;
  focParam.pid_torque_I = i;
  focParam.pid_torque_D = d;
}
void handle_calibration() { calibrate(); }
