typedef struct {
    // Motor gerçek zamanlı ölçümler
    float position;       // Motor konumu (elektriksel/mechanik)
    float velocity;       // Motor hızı (rad/s veya rpm)
    float torque;         // Motor tork değeri (Nm veya akım ile ilişkili)

    // FOC iç durum
    float i_d;            // D-axis current
    float i_q;            // Q-axis current
    float v_d;            // D-axis voltage (komutlanan)
    float v_q;            // Q-axis voltage (komutlanan)

    // İç kontrol durumları (opsiyonel)
    float pid_pos_P;
    float pid_pos_I;
    float pid_pos_D;
    float pid_vel_P;
    float pid_vel_I;
    float pid_vel_D;
    float pid_cur_P;
    float pid_cur_I;
    float pid_cur_D;


    float min_angle;
    float max_angle;
    float temprature;
    float bus_voltage;
    float internal_tempreture;
} FOC_Parameter;
