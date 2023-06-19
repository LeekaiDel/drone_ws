class PidReg {
private:
    float P = 0.0;
    float I = 0.0;
    float D = 0.0;
    float err_f = 0.0;
    float last_err_f = 0.0;

public:
    float Kp, Ki, Kd;
    
    void setK(float new_Kp, float new_Ki, float new_Kd) {
        Kp = new_Kp;
        Ki = new_Ki;
        Kd = new_Kd;
    }

    float calcRegSignal(float dt, float f, float last_f) {
        float err_f = f - last_f;

        P = Kp * err_f; 
        I = I + Ki * err_f * dt;
        D = Kd * (err_f - last_err_f) / dt; 

        last_err_f = err_f;

        float u = P + I + D;

        return u;
    }
};