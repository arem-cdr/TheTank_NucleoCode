#include "SpeedPIDController.h"

void printVallaray(std::valarray<double> array)
{
    // Serial.print("$ ");
    // Serial.print(array[0]);
    // Serial.print(" ");
    // Serial.print(array[1]);
    // Serial.print(" ");
    // Serial.print(array[2]);
    // Serial.print(" ");
    // Serial.print(array[3]);
    // Serial.println(" ;");

}

SpeedPIDController::SpeedPIDController(uint32_t input_rate_ms) : Kp(4,0),  Ki(4,0),  Kd(4,0),
e(4,0),  ei(4,0), ed(4,0), rate_ms(input_rate_ms), lastCall_date_ms(millis()), Ku(4,0), Tosc(4,0),setpoint_speed(4,0)
{
    delta = ((double)rate_ms)/1000.0;
    // Ku = {0.12,0.12,0.12,0.12};
    // Tosc = {2*delta,2*delta,2*delta,2*delta};

    Kd = {0.0,0.0,0.0,0.0};
    Ki = {0.0,0.0,0.0,0.0};
    Kp = {0.1,0.1,0.1,0.1};

    // Ki = 1.2*(Ku*delta)/(Tosc);
    // Kd = 0.3*(Ku*Tosc)/(4*delta);
    // Kp = 0.6*Ku-0.5*Ki;
    
    motors = new BlocMoteurs();
    encoders = new Encoder4Mot();
    motors->motors_stop();
    


    
}


SpeedPIDController::SpeedPIDController(BlocMoteurs* motors_ptr, Encoder4Mot* encoders_ptr, uint32_t input_rate_ms) : Kp(4,0),  Ki(4,0),  Kd(4,0),
e(4,0),  ei(4,0), ed(4,0), rate_ms(input_rate_ms), lastCall_date_ms(millis()), Ku(4,0), Tosc(4,0), setpoint_speed(4,0)
{
    delta = ((double)rate_ms)/1000.0;
    Ku = {0.135,0.1326,0.1675,0.127};
    Tosc = {2*delta,2*delta,2*delta,2*delta};

    // Kd = {0.0,0.0,0.0,0.0};
    // Ki = {0.0,0.0,0.0,0.0};
    // Kp = {0.135,0.1326,0.1675,0.127};

    Ki = 1.2*(Ku*delta)/(Tosc);
    Kd = 0.3*(Ku*Tosc)/(4*delta);
    Kp = 0.6*Ku-0.5*Ki;
    
    if(motors_ptr == NULL)
    {
        motors = new BlocMoteurs();
    }
    else
    {
        motors = motors_ptr;
    }
    if(encoders_ptr == NULL)
    {
        encoders = new Encoder4Mot();
    }
    else
    {
        encoders=encoders_ptr;
    }
    motors->motors_stop();


    


    
}

void SpeedPIDController::define_setpoint(double w_M1_FD, double w_M2_FG, double w_M3_BD, double w_M4_BG)
{
    setpoint_speed = { w_M1_FD, w_M2_FG, w_M3_BD, w_M4_BG};
    // e = {0.0,0.0,0.0,0.0};
    // ei = {0.0,0.0,0.0,0.0};
    // ed = {0.0,0.0,0.0,0.0};
    motors->motors_on();
    
};

bool SpeedPIDController::update_controller(bool updateEnco, bool waitLoop)
{
    if(millis() > lastCall_date_ms + rate_ms || waitLoop == false)
    {
        std::vector<double> readings_vector;
        if(updateEnco)
        {
            readings_vector = encoders->Encoder4MotUpdate();
        }
        else
        {
            readings_vector = encoders->GetSpeeds();
        }

        std::valarray<double> readings = {readings_vector[0], readings_vector[1], readings_vector[2], readings_vector[3] };

        ed = (setpoint_speed - readings) - e;
        e = setpoint_speed - readings;
        ei+=delta*e;
        std::valarray<double> u = Kp*e + Kd*ed + Ki*ei;
       
        motors->commande_vitesses(u[0],u[1],u[2],u[3]);
        lastCall_date_ms = millis();
        return true;       

    }
    else
    {
        return false;
    }
}

