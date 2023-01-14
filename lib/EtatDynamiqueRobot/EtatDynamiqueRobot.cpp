#include "EtatDynamiqueRobot.h"

EtatDynamiqueRobot::EtatDynamiqueRobot(double x, double y, double theta_i) : x(x), y(y), theta(theta_i)
{
    // On s'assure que les angles soient bien dans ]-PI, PI]
    containTheta(theta);

    v_xy = 0;
    v_theta = 0;
    v_x = 0;
    v_y=0;

    
}

void EtatDynamiqueRobot::versPosition(double n_x, double n_y, double n_theta, uint32_t dt)
{
    // On s'assure que les angles soient bien dans ]-PI, PI]
    containTheta(n_theta);

    double dt_s = ((double)dt)/(1000000.0);
    v_xy = sqrt( pow(n_x - x, 2) + pow(n_y - y, 2)) / dt_s;
    v_x = (n_x - x)/dt_s; 
    v_y = (n_y - y)/dt_s;
    v_theta = (n_theta - theta) / dt_s;

    x = n_x;
    y = n_y;
    theta = n_theta;

    
}

void EtatDynamiqueRobot::setPosition(double n_x, double n_y, double n_theta)
{
    // On s'assure que les angles soient bien dans ]-PI, PI]
    containTheta(n_theta);

    x = n_x;
    y = n_y;
    theta = n_theta;

    v_xy = 0;
    v_theta = 0;
    v_x = 0;
    v_y = 0;
}

void EtatDynamiqueRobot::translatePosition(double n_x, double n_y, double n_theta)
{
    // On s'assure que les angles soient bien dans ]-PI, PI]
    containTheta(n_theta);

    x = n_x;
    y = n_y;
    theta = n_theta;
}

/**
 * @brief Calcul l'angle le plus petit pour atteindre la cible depuis l'angle initial. Le résultat est garanti dans ]-PI, PI].
 */
double EtatDynamiqueRobot::deltaAngleTo(double initial, double target) const
{
    // On s'assure que les angles soient bien dans ]-PI, PI]
    containTheta(initial);
    containTheta(target);

    // Chemin naïf vers l'objectif
    double delta = (target - initial);
    
    // Dans le cas ou il serait plus simple de partir dans l'autre sens
    containTheta(delta);

    return delta;
    
}

double EtatDynamiqueRobot::getThetaRadian() const
{
    return theta;
}

double EtatDynamiqueRobot::getX() const
{
    return x;
}

double EtatDynamiqueRobot::getY() const
{
    return y;
}

double EtatDynamiqueRobot::getV() const
{
    return v_xy;
}

double EtatDynamiqueRobot::getVTheta() const
{
    return v_theta;
}

double EtatDynamiqueRobot::getVX() const
{
    return v_x;

}
double EtatDynamiqueRobot::getVY() const
{
    return v_y;
}
