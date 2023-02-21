#include "SpeedController_M.h"

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

SpeedController_M::SpeedController_M() : K(4,0),
lastCall_date_ms(millis()),setpoint_speed(4,0),
motors_input(4,0),motors_input_last(4,0),
e(4,0),e_last(4,0),
c1(4,0),c2(4,0)
{
    rate_ms = RATE_MS;
    delta = ((double)rate_ms)/1000.0;
    c1 = {0.153356155,0.179160668,0.18161829,0.174439702};
    c2 = {0.119433894,0.146684349,0.14869648,0.142819148};



    motors_input = {0.0,0.0,0.0,0.0};
    motors_input_last = {0.0,0.0,0.0,0.0};
   
    e =  {0.0,0.0,0.0,0.0};
    e_last = {0.0,0.0,0.0,0.0};
   
    motors = new BlocMoteurs();
    encoders = new Encoder4Mot();
    motors->motors_stop();
    


    
}


SpeedController_M::SpeedController_M(BlocMoteurs* motors_ptr, Encoder4Mot* encoders_ptr) :K(4,0),
lastCall_date_ms(millis()),setpoint_speed(4,0),
motors_input(4,0),motors_input_last(4,0),
e(4,0),e_last(4,0),
c1(4,0),c2(4,0)
{
    rate_ms = RATE_MS;
    delta = ((double)rate_ms)/1000.0;

    c1 = {0.153356155,0.179160668,0.18161829,0.174439702};
    c2 = {0.119433894,0.146684349,0.14869648,0.142819148};
    
    motors_input = {0.0,0.0,0.0,0.0};
    motors_input_last = {0.0,0.0,0.0,0.0};
   
    e =  {0.0,0.0,0.0,0.0};
    e_last = {0.0,0.0,0.0,0.0};
    

    
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

void SpeedController_M::define_setpoint(double w_M1_FD, double w_M2_FG, double w_M3_BD, double w_M4_BG)
{
    setpoint_speed = { w_M1_FD, w_M2_FG, w_M3_BD, w_M4_BG};
    motors->motors_on();
    
};

bool SpeedController_M::update_controller(bool updateEnco, bool waitLoop)
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


 
        e_last = e;
        e = setpoint_speed-readings;
       
        

  
        motors_input_last = motors_input;    
        motors_input = motors_input_last + c1*e -c2*e_last;
        
        motors->commande_vitesses(motors_input[0],motors_input[1],motors_input[2],motors_input[3]);
        lastCall_date_ms = millis();
        return true;       

    }
    else
    {
        return false;
    }
}

