#include "SpeedController_PP.h"

void printVallaray(std::valarray<double> array)
{
    Serial.print("$ ");
    Serial.print(array[0]);
    Serial.print(" ");
    Serial.print(array[1]);
    Serial.print(" ");
    Serial.print(array[2]);
    Serial.print(" ");
    Serial.print(array[3]);
    Serial.println(" ;");

}

SpeedController_PP::SpeedController_PP() : K(4,0),z0(4,0),
lastCall_date_ms(millis()),setpoint_speed(4,0),
motors_input(4,0),motors_input_last(4,0),motors_input_last_last(4,0),
e(4,0),e_last(4,0),e_last_last(4,0)
{
    rate_ms = RATE_MS;
    delta = ((double)rate_ms)/1000.0;
    //K = {0.086262837,0.100777876,0.102160288,0.098122332};
    K = {0.053828569,0.046058035,0.055070336,0.0458791};
    z0 = {0.727259923,0.668161207,0.7290917,0.679194871};

    zr = 0.5;

    motors_input = {0.0,0.0,0.0,0.0};
    motors_input_last = {0.0,0.0,0.0,0.0};
    motors_input_last_last = {0.0,0.0,0.0,0.0};
    e =  {0.0,0.0,0.0,0.0};
    e_last = {0.0,0.0,0.0,0.0};
    e_last_last = {0.0,0.0,0.0,0.0};
    motors = new BlocMoteurs();
    encoders = new Encoder4Mot();
    motors->motors_stop();
    


    
}


SpeedController_PP::SpeedController_PP(BlocMoteurs* motors_ptr, Encoder4Mot* encoders_ptr) :K(4,0),z0(4,0),
lastCall_date_ms(millis()),setpoint_speed(4,0),
motors_input(4,0),motors_input_last(4,0),motors_input_last_last(4,0),
e(4,0),e_last(4,0),e_last_last(4,0)
{
    rate_ms = RATE_MS;
    delta = ((double)rate_ms)/1000.0;
    //K = {0.086262837,0.100777876,0.102160288,0.098122332};
    K = {0.053828569,0.046058035,0.055070336,0.0458791};
    z0 = {0.727259923,0.668161207,0.7290917,0.679194871};

    zr = 0.5;
    

    motors_input = {0.0,0.0,0.0,0.0};
    motors_input_last = {0.0,0.0,0.0,0.0};
    motors_input_last_last = {0.0,0.0,0.0,0.0};
    e =  {0.0,0.0,0.0,0.0};
    e_last = {0.0,0.0,0.0,0.0};
    e_last_last = {0.0,0.0,0.0,0.0};

    
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

void SpeedController_PP::define_setpoint(double w_M1_FD, double w_M2_FG, double w_M3_BD, double w_M4_BG)
{
    setpoint_speed = { w_M1_FD, w_M2_FG, w_M3_BD, w_M4_BG};
    motors->motors_on();
    
};

bool SpeedController_PP::update_controller(bool updateEnco, bool waitLoop)
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


        e_last_last = e_last;
        e_last = e;
        e = setpoint_speed-readings;
       
        

        motors_input_last_last = motors_input_last;
        motors_input_last = motors_input;    
        motors_input = ((1+zr)*motors_input_last)-(zr*motors_input_last_last)+(K*e_last)-((z0*K)*e_last_last);
        
        motors->commande_vitesses(motors_input[0],motors_input[1],motors_input[2],motors_input[3]);
        lastCall_date_ms = millis();
        return true;       

    }
    else
    {
        return false;
    }
}

