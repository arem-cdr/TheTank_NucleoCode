#pragma once



#include <Arduino.h>


//Classe pour enregistrer la position (x,y,theta) du robot.
//Effectue des calculs de vitesse par derivation de la position
class EtatDynamiqueRobot
{
    public:
        EtatDynamiqueRobot(double x, double y, double theta_i);

        
        //Indiquer une nouvelle position, calcule les vitesses
        void versPosition(double n_x, double n_y, double n_theta, uint32_t dt);

        //definit une nouvelle position, vitesses init a 0
        void setPosition(double n_x, double n_y, double n_theta);

        //decaler la position sans modifier les vitesses
        void translatePosition(double n_x, double n_y, double n_theta);


        double getThetaRadian() const;
        double getX() const;
        double getY() const;
        double getV() const;
        double getVTheta() const;
        double getVX() const;
        double getVY() const;

        // permet d'obtenir l'angle entre initial et target, fait appel a containTheta
        //pour avoir un angle entre PI et -PI
        double deltaAngleTo(double initial, double target) const ; 


        void containTheta(double & theta_i) const{
            if(theta_i > PI || theta_i < -PI )
            {
                int k = theta_i/(2.0*PI);
                theta_i = theta_i - k*2.0*PI;
            }
        }  ;

    private:
        double x; //position selon x
        double y; //position selon y
        double theta; //angle
        
        double v_xy; // norme du vecteur vitesse
        double v_theta; // vitesse angulaire

        double v_x; //vitesse selon x
        double v_y; //vitesse selon y

     
};

