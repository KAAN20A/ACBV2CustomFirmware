#ifndef SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

void process_serial_message(char *cmd) ;
void UART_SendFloat(float value);
void UART_SendString(const char *str);
void handle_get_velocity(void);
void handle_get_torque(void);
void handle_get_temperature();
void handle_get_bus_voltage();
void handle_get_internal_temperature();
void handle_get_current_a();
void handle_get_current_b();
void handle_get_current_c();
void handle_reset();
void handle_get_min_angle();
void handle_set_min_angle(float min_angle);
void handle_get_max_angle();
void handle_set_max_angle(float max_angle);
void handle_get_velocity_pid();
void handle_set_velocity_pid(float p, float i, float d);
void handle_get_angle_pid();
void handle_set_angle_pid(float p, float i, float d);
void handle_get_current_pid();
void handle_set_current_pid(float p, float i, float d);
void handle_calibration();

#ifdef __cplusplus
}
#endif

#endif
