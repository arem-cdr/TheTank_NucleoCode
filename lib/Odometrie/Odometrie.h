#pragma once


#include <Arduino.h>
#include "Reglages.h"
#include "EtatDynamiqueRobot.h"
#include "Encoder4Mot.h"

//Classe gerant le calcul de la position du robot a partir des encodeurs
class Odometrie
{
    public:
        Odometrie(uint32_t min_update_period_us, Encoder4Mot* myencodeurs);
        bool update(); // permet d'efectuer la mise a jour de la position, renvoie le booleen si la mise a jour a été faite
        // la mise a jour s'effectue au plus toute les min_update_period_us
        
        double getThetaRadian() const; 
        double getX() const;
        double getY() const;
        double getV() const;
        double getVX() const; // get VX avec derivation de la position
        double getVY() const; // get Vy avec derivation de la position
        double getVTheta() const; // get VTheta avec derivation de l'angle

        double deltaAngleTo(double initial, double target) const; // permet d'avoir l'angle entre 2 angles, securisé pour retourner un angle entre -PI et PI

        double getVXEnco()const{return VXEnco;}; // get Vx calculé a partir des encodeurs
        double getVYEnco()const{return VYEnco;}; // get Vy calculé a partir des encodeurs
        double getVThetaEnco()const{return VThetaEnco;}; // get Vtheta calculé a partir des encodeurs

       
        
        void setX(double x);
        void setY(double y);
        void setTheta(double theta);

        // Fonctions pour obtenir les vitesses du robot dans son réferentiel en fonction 
        // des vitesses des roues et vice versa
        void compute_encoders_to_robot(const double *w1, const double *w2, const double *w3, const double *w4, 
                                        double *vx, double *vy, double *wz);
        void compute_robot_to_encoders(const double *vx, const double *vy, const double *wz, 
                                        double *w1, double *w2, double *w3, double *w4);

        void setRayonRoues(double newRayon); // modifie le rayon des roues théoriques pour les calculs de la position
        void setL1pL2(double newL1pL2); // modifier la valeur de L1 + L2 théoriques pour les calculs de positions
        double getRayonRoues(); // get rayon rayon des roues théoriques pour les calculs de la position 
        double getL1pl2(); //get valeur de L1 + L2 théorique pour les calculs de positions
        uint32_t get_min_update_period_us(); // get min_uptate_periode_us
        uint32_t set_min_update_period_us(uint32_t new_period){_min_update_period_us = new_period;};
    

    private:


        Encoder4Mot *encodeurs;

        EtatDynamiqueRobot etat; // position du robot gere par une instance de la classe EtatDynamiqueRobot

        uint32_t _min_update_period_us; // periode d'uptade minimale
        uint32_t _last_update_etat; // il s'agit de la date de la dernière mise à jour de position/rotation/vitesse

        double RayonRoues; // rayon des roues théoriques pour les calculs de la position
        double L1pL2; // valeur de L1 + L2 théorique pour les calculs de positions

        double VXEnco;
        double VYEnco;
        double VThetaEnco;

 
        

};



