#pragma once
#include "BlocMoteurs.h"
#include "Encoder4Mot.h"
#include <algorithm>
#include <vector>
#include <valarray>
#include "Reglages.h"

#define RATE_MS 30


class SpeedController_M // Classe pour asservir la vitesse des moteurs Avec methode de placement de pôles
// a utiliser avec rate = 30_ms uniquement
{
    private:
        std::valarray<double> K; // Kp du correcteur PID 
        std::valarray<double> motors_input;
        std::valarray<double> motors_input_last;
        std::valarray<double> e;
        std::valarray<double> e_last;
        std::valarray<double> c1;
        std::valarray<double> c2;

        std::valarray<double> setpoint_speed; // vitesse de consigne des moteurs en rad/s 
        uint32_t rate_ms; // periode d'echantilonnage en ms
        double delta; // periode d'echantilonnage en s
        uint32_t lastCall_date_ms; // date du dernier appel de update
        BlocMoteurs* motors; // moteurs
        Encoder4Mot* encoders; // encodeurs

        
    public:
        SpeedController_M(BlocMoteurs* motors_ptr, Encoder4Mot* encoders_ptr) ; // Constructeur 
        SpeedController_M();  // Constructeur creeant motors et encoders
        ~SpeedController_M();
        void define_setpoint(double w_M1_FD, double w_M2_FG, double w_M3_BD, double w_M4_BG); // definit la vitesse des moteurs de consigne
        bool update_controller(bool updateEnco = true, bool waitLoop = true); // update le controlleur :
        // updateEnco : true : fait appel a l'update de l'enco, utilie les fonctions de get sinon : en cas d'utilisation concurrente de enco avec odometrie
        // wait Loop : atteint que input_rate_ms ce soit ecoule : en cas d'utilisation d'une autre classe qui cadencerait deja l'execution de l'update 
        void stop_mot() // stop le moteur 
        { 
            setpoint_speed-=setpoint_speed;
            motors->motors_stop();
        };
        std::valarray<double> getmotors_input(){return motors_input;};


        
};


