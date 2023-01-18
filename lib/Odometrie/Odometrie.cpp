#include "Odometrie.h"




Odometrie::Odometrie(uint32_t min_update_period_us, Encoder4Mot* myencodeurs) : _min_update_period_us(min_update_period_us), etat(INIT_X, INIT_Y, INIT_THETA)
{
    
    encodeurs = myencodeurs;


    // pour mettre les angles à zéro au début (pas sur que necessaire sur la version mecanom des encodeurs)

    encodeurs->Encoder4MotUpdate();

    
    RayonRoues = RAYON_ROUE;
    L1pL2 = L1 + L2;

    _last_update_etat = micros();



}




bool Odometrie::update()
{
    uint32_t date = micros();
    // Limitation de la fréquence d'update de l'odométrie (pas de limite si _min_update_period_us = 0)
    if(date - _last_update_etat > _min_update_period_us)
    {
        // (On suppose les vitesses constantes pendant t, la trajectoire decrite par le robot est alors un cercle)
        // on manipule des entiers le plus possible, c'est pourquoi l'unité est le micrometre
        uint32_t ellapsed = date - _last_update_etat;
        double ellapsed_s = ((double)ellapsed)/(1000000.0);
        uint32_t date = micros();

        // position et orientation actuelle du robot
        double x = etat.getX();
        double y = etat.getY();
        double theta = etat.getThetaRadian();

        // nouvelle position et orientation du robot
        double n_x = 0;
        double n_y = 0;
        double n_theta = 0;

        // ///////////////////////////////////////////////////////////////////////////////////////
        // Si cette fonction update est appellée a intervalles réguliers, les distances peuvent
        // aussi être considérées comme homogènes à des vitesses (car dt se simplifie partout).

        // Récuperation des ticks parcourue par chaque encodeur (entiers de 0 à 4095)

        std::vector<double> speeds = encodeurs->Encoder4MotUpdate();    


        double wRoueFD = speeds[0];
        double wRoueFG = speeds[1];
        double wRoueBD = speeds[2];
        double wRoueBG = speeds[3];

        //////////////////////////////////////////////////////////////////////////////////////////
        // calcul du déplacement du robot ////////////////////////////////////////////////////////

        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;

        // Calcul des déplacements en x et y et theta relativement au robot
        // C'est dans ce calcul qu'interviennent les dimensions des roues et du chassis
        compute_encoders_to_robot(&wRoueFG, &wRoueFD, &wRoueBG, &wRoueBD, &vx, &vy, &wz);
        VXEnco = vx;
        VYEnco = vy;
        VThetaEnco = wz;

        // Calcul des déplacements dans le référentiel du terrain
        n_theta = theta + wz*ellapsed_s;
        n_x = x + vx*ellapsed_s*cos(theta + (wz*ellapsed_s)/2 ) - vy*ellapsed_s*sin(theta + (wz*ellapsed_s)/2 );
        n_y = y + vx*ellapsed_s*sin(theta + (wz*ellapsed_s)/2 ) + vy*ellapsed_s*cos(theta + (wz*ellapsed_s)/2 );
      
        etat.versPosition(n_x, n_y, n_theta, ellapsed);
       
        
        _last_update_etat = date;
        
        return true;
    }
    return false;
}

double Odometrie::getThetaRadian() const
{
    return etat.getThetaRadian();
}

double Odometrie::getX() const
{
    return etat.getX();
}

double Odometrie::getY() const
{
    return etat.getY();
}

double Odometrie::getV() const
{
    return etat.getV();
}

double Odometrie::getVTheta() const
{
    return etat.getVTheta();
}

double Odometrie::deltaAngleTo(double initial, double target) const
{
    return etat.deltaAngleTo(initial,target);
}



void Odometrie::setX(double x)
{
    etat.translatePosition(x, etat.getY(), etat.getThetaRadian());
}

void Odometrie::setY(double y)
{
    etat.translatePosition(etat.getX(), y, etat.getThetaRadian());
}

void Odometrie::setTheta(double theta)
{
    etat.translatePosition(etat.getX(), etat.getY(), theta);

    
}

void Odometrie::compute_encoders_to_robot(const double *w1, const double *w2, const double *w3, const double *w4, 
                                        double *vx, double *vy, double *wz)
{
    //                                                               |w1|
    // |vx|         |    1            1           1        1     |   |w2|
    // |vy| = R/4 * |    1           -1          -1        1     | x |w3|
    // |wz|         |-1/(L1+L2)  1/(L1+L2)  -1/(L1+L2)  1/(L1+L2)|   |w4|

    *vx = ((double)RayonRoues/4.0) * (*w1 + *w2 + *w3 + *w4);
    *vy = -((double)RayonRoues/4.0) * (*w1 - *w2 - *w3 + *w4);
    *wz = ((double)RayonRoues/(4.0*(L1pL2))) * (-*w1 + *w2 - *w3 + *w4);
}

void Odometrie::compute_robot_to_encoders(const double *vx, const double *vy, const double *wz, 
                                        double *w1, double *w2, double *w3, double *w4)
{
    // |w1|         |1      1     -(L1+L2)|   |vx|
    // |w2|         |1     -1     -(L1+L2)|   |vy|
    // |w3| = 1/R * |1     -1     -(L1+L2)| x |wz|
    // |w4|         |1      1     -(L1+L2)|

    *w1 = (1.0/((double)RayonRoues)) * (*vx + *vy + (L1pL2)*(*wz));
    *w2 = (1.0/((double)RayonRoues)) * (*vx - *vy - (L1pL2)*(*wz));
    *w3 = (1.0/((double)RayonRoues)) * (*vx - *vy + (L1pL2)*(*wz));
    *w4 = (1.0/((double)RayonRoues)) * (*vx + *vy - (L1pL2)*(*wz));
}

void Odometrie::setRayonRoues(double newRayon)
{
    RayonRoues = newRayon;
}
void Odometrie::setL1pL2(double newL1pL2)
{
    L1pL2 = newL1pL2;

}
double Odometrie::getRayonRoues()
{
    return RayonRoues;

}
double Odometrie::getL1pl2()
{
    return L1pL2;
}

uint32_t Odometrie::get_min_update_period_us()
{
    return _min_update_period_us;
}

double Odometrie::getVX() const
{
    return etat.getVX();

}
        
double Odometrie::getVY() const
{
    return etat.getVY();
}




