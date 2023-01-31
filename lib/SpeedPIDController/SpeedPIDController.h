#pragma once
#include "BlocMoteurs.h"
#include "Encoder4Mot.h"
#include <algorithm>
#include <vector>
#include <valarray>
#include "Reglages.h"


class SpeedPIDController // Classe pour asservir la vitesse des moteurs 
{
    private:
        std::valarray<double> Kp; // Kp du correcteur PID 
        std::valarray<double> Ki; // Ki du correcteur PID 
        std::valarray<double> Kd; // Kd du correcteur PID 
        std::valarray<double> e; // erreur
        std::valarray<double> ei; // erreur intégrale
        std::valarray<double> ed; // erreur dérivé
        std::valarray<double> Ku; // K pompage (oscillation) -> methode de ziegler Nichols/Takahashi (numérique)
        std::valarray<double> Tosc; // periode pompage (os) -> methode de ziegler Nichols/Takahashi (numérique)
        std::valarray<double> setpoint_speed; // vitesse de consigne des moteurs en rad/s 
        uint32_t rate_ms; // periode d'echantilonnage en ms
        double delta; // periode d'echantilonnage en s
        uint32_t lastCall_date_ms; // date du dernier appel de update
        BlocMoteurs* motors; // moteurs
        Encoder4Mot* encoders; // encodeurs

        
    public:
        SpeedPIDController(BlocMoteurs* motors_ptr, Encoder4Mot* encoders_ptr, uint32_t input_rate_ms = 30) ; // Constructeur 
        SpeedPIDController(uint32_t input_rate_ms = 30);  // Constructeur creeant motors et encoders
        ~SpeedPIDController();
        void define_setpoint(double w_M1_FD, double w_M2_FG, double w_M3_BD, double w_M4_BG); // definit la vitesse des moteurs de consigne
        bool update_controller(bool updateEnco = true, bool waitLoop = true); // update le controlleur :
        // updateEnco : true : fait appel a l'update de l'enco, utilie les fonctions de get sinon : en cas d'utilisation concurrente de enco avec odometrie
        // wait Loop : atteint que input_rate_ms ce soit ecoule : en cas d'utilisation d'une autre classe qui cadencerait deja l'execution de l'update 
        void stop_mot() // stop le moteur 
        { 
            setpoint_speed-=setpoint_speed;
            motors->motors_stop();
        };
        void set_calib(double ku_1, double ku_2, double ku_3, double ku_4)
        {
            Kd = {0.0,0.0,0.0,0.0};
            Ki = {0.0,0.0,0.0,0.0};
            Kp = {ku_1,ku_2,ku_3,ku_4};
        };
        void unset_calib(double ku_1, double ku_2, double ku_3, double ku_4)
        {
            Ku = {ku_1,ku_2,ku_3,ku_4};
            Ki = 1.2*(Ku*delta)/(Tosc);
            Kd = 0.3*(Ku*Tosc)/(4*delta);
            Kp = 0.6*Ku-0.5*Ki;
        };
        void set_rate(uint32_t new_rate_ms) {rate_ms = new_rate_ms;};
};


