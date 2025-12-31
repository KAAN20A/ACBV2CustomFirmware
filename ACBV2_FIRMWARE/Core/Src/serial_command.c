extern FOC_Parameter focparameter;

void UART_SendFloat(float value) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%.3f\r\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

// String tanımlı gönderim
void UART_SendString(const char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
void handle_set_position(float position) { FOC_Position(position); }
void handle_set_velocity(float velocity) { FOC_Velocity(velocity); }
void handle_set_torque(float torque) { FOC_Torque(torque); }
void handle_get_position(void) {
    // UART ile Başlık
    UART_SendString("Position: ");

    // focParam.position değeri global struct’tan okunuyor
    UART_SendFloat(focParam.position);
}
void handle_get_velocity(void) {
    // UART ile Başlık
    UART_SendString("Velocity: ");

    // focParam.position değeri global struct’tan okunuyor
    UART_SendFloat(focParam.velocity);
}

void handle_get_torque(void) {
    // UART ile Başlık
    UART_SendString("Torque: ");

    // focParam.position değeri global struct’tan okunuyor
    UART_SendFloat(focParam.torque);
}

void handle_get_temperature(){
    UART_SendString("Temprature: ");

    // focParam.position değeri global struct’tan okunuyor
    focParam.temprature=float calculateTempreture();
    UART_SendFloat(focParam.temprature);
}
}
   void handle_get_bus_voltage(){
       UART_SendString("Bus_Voltage: ");

       focParam.bus_voltage=float calculateBus_Voltage();
       UART_SendFloat(focParam.bus_voltage);
   }
   }
   void handle_get_internal_temperature(){
       UART_SendString("Internal_Tempreture: ");

       focParam.internal_temprature=float calculateInternal_Temprature();
       UART_SendFloat(focParam.internal_tempreture);
   }
   }

   void handle_get_current_a(){
       UART_SendString("Velocity: ");

       // focParam.position değeri global struct’tan okunuyor
       UART_SendFloat(focParam.velocity);
   }

       void handle_get_current_b(){
           UART_SendString("Velocity: ");

       // focParam.position değeri global struct’tan okunuyor
       UART_SendFloat(focParam.velocity);
   }
       void handle_get_current_c(){
           UART_SendString("Velocity: ");

           // focParam.position değeri global struct’tan okunuyor
           UART_SendFloat(focParam.velocity);
       }



       void handle_reset(){
           NVIC_System_Reset();
       }


       void handle_get_min_angle(){
           UART_SendString("Min_Angle: ");

           // focParam.position değeri global struct’tan okunuyor
           UART_SendFloat(focParam.min_angle);
       }

       void handle_set_min_angle(float min_angle){
           focParam.min_angle=min_angle;
       }
       void handle_get_max_angle(){
           UART_SendString("Max_Angle: ");

           // focParam.position değeri global struct’tan okunuyor
           UART_SendFloat(focParam.max_angle);
       }

       void handle_set_max_angle(float max_angle){
           focParam.min_angle=min_angle;
       }
       void handle_get_velocity_pid(){
           UART_SendString("VelocityP: ");
           UART_SendFloat(focParam.pid_vel_P);
           UART_SendString("VelocityI: ");
           UART_SendFloat(focParam.pid_vel_I);
           UART_SendString("VelocityD: ");
           UART_SendFloat(focParam.pid_vel_D);
       }

          void handle_set_velocity_pid(float p, float i, float d){
              focParam.pid_vel_P=p;
              focParam.pid_vel_I=i;
              focParam.pid_vel_D=d;
          }
          void handle_get_angle_pid(){
          UART_SendString("AngleyP: ");
          UART_SendFloat(focParam.pid_angle_P);
          UART_SendString("AngleyI: ");
          UART_SendFloat(focParam.pid_angle_I);
          UART_SendString("AngleyD: ");
          UART_SendFloat(focParam.pid_angle_D);}
          void handle_set_angle_pid(float p, float i, float d){
              focParam.pid_angle_P=p;
              focParam.pid_angle_I=i;
              focParam.pid_angle_D=d;
          }
          void handle_get_current_pid(){
              UART_SendString("TorqueP: ");
              UART_SendFloat(focParam.pid_torque_P);
              UART_SendString("TorqueI: ");
              UART_SendFloat(focParam.pid_torque_I);
              UART_SendString("TorqueD: ");
              UART_SendFloat(focParam.pid_torque_D)
          }
          void handle_set_current_pid(float p, float i, float d){
              focParam.pid_torque_P=p;
              focParam.pid_torque_I=i;
              focParam.pid_torque_D=d;
          }
          void handle_calibration(){
              calibrate();
          }
