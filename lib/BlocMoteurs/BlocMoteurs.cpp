#include "BlocMoteurs.h"



BlocMoteurs::BlocMoteurs()
{
    moteurs_arret = 0; // Par defaut le moteur est a l'arret
    saturation = false;


    for(int i=3;i<9;i++)
    {
       pinMode(i,OUTPUT); //Initialisation des PINS moteurs
    }
    for(int i=11;i<13;i++)
    {
        pinMode(i,OUTPUT); //Initialisation des PINS moteurs
    }


   



  


}

BlocMoteurs::~BlocMoteurs()
{
    
}

void BlocMoteurs::commande_vitesses(float vitesse_normalisee_FD, float vitesse_normalisee_FG, float vitesse_normalisee_BD, float vitesse_normalisee_BG )
{ 
    saturation = false;
    /////////////////////////////////////////////////////////////
    // Assurance que les parametres sont entre -1 et 1 ////////////////
    vitesse_normalisee_FG = min(1.0f, vitesse_normalisee_FG);
    vitesse_normalisee_FG = max(-1.0f, vitesse_normalisee_FG);

    vitesse_normalisee_FD = min(1.0f, vitesse_normalisee_FD);
    vitesse_normalisee_FD = max(-1.0f, vitesse_normalisee_FD);

    vitesse_normalisee_BG = min(1.0f, vitesse_normalisee_BG);
    vitesse_normalisee_BG = max(-1.0f, vitesse_normalisee_BG);

    vitesse_normalisee_BD = min(1.0f, vitesse_normalisee_BD);
    vitesse_normalisee_BD = max(-1.0f, vitesse_normalisee_BD);
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // Détermination du signe des vitesses (logique inversée entre les deux moteurs car placés symétriquement sur le robot et branchement identique)
    DirectionMotor sens_FG = vitesse_normalisee_FG >= 0.0f ? DirectionMotor::BWD : DirectionMotor::FWD;
    DirectionMotor sens_FD = vitesse_normalisee_FD >= 0.0f ? DirectionMotor::BWD : DirectionMotor::FWD;
    DirectionMotor sens_BG = vitesse_normalisee_BG >= 0.0f ? DirectionMotor::BWD : DirectionMotor::FWD;
    DirectionMotor sens_BD = vitesse_normalisee_BD >= 0.0f ? DirectionMotor::BWD : DirectionMotor::FWD;


    // Détermination des valeurs de vitesses réelles à envoyer au shield
    int vitesse_FG = abs(vitesse_normalisee_FG * COEF_FG) * MAX_PWM_VALUE_THEORETICAL;
    int vitesse_FD = abs(vitesse_normalisee_FD * COEF_FD) * MAX_PWM_VALUE_THEORETICAL;
    int vitesse_BG = abs(vitesse_normalisee_BG * COEF_BG) * MAX_PWM_VALUE_THEORETICAL;
    int vitesse_BD = abs(vitesse_normalisee_BD * COEF_BD) * MAX_PWM_VALUE_THEORETICAL;

    if( vitesse_FD > MAX_PWM_VALUE_SAFETY || vitesse_FD > MAX_PWM_VALUE_SAFETY ||
        vitesse_BD > MAX_PWM_VALUE_SAFETY || vitesse_FG > MAX_PWM_VALUE_SAFETY)
    {
        saturation = true;
    }
    
    set_vitesse_moteur_FG(vitesse_FG , sens_FG);
    set_vitesse_moteur_FD(vitesse_FD , sens_FD);
    set_vitesse_moteur_BG(vitesse_BG , sens_BG);
    set_vitesse_moteur_BD(vitesse_BD , sens_BD);

   



}

void BlocMoteurs::set_vitesse_moteur_FG(int vitesse, DirectionMotor dir)
{
    // verification de la validité de la vitesse a envoyer

    vitesse = abs(vitesse);
    vitesse = min(vitesse, (int)MAX_PWM_VALUE_SAFETY);
    //vitesse = vitesse > 10 ? vitesse : 0;

    if (!moteurs_arret)
    {
        digitalWrite(M2,dir);
        analogWrite(E2,vitesse);
    }
    else
    {
       analogWrite(E2,0); 
    }

    

}

void BlocMoteurs::set_vitesse_moteur_FD(int vitesse, DirectionMotor dir)
{
    // verification de la validité de la vitesse a envoyer
    
    vitesse = abs(vitesse);
    vitesse = min(vitesse, (int)MAX_PWM_VALUE_SAFETY);
    //vitesse = vitesse > 10 ? vitesse : 0;

    if (!moteurs_arret)
    {
        digitalWrite(M1,dir);
        analogWrite(E1,vitesse);
    }
    else
    {
       analogWrite(E1,0); 
    }

    
    
}


void BlocMoteurs::set_vitesse_moteur_BG(int vitesse, DirectionMotor dir)
{
    // verification de la validité de la vitesse a envoyer
    
    vitesse = abs(vitesse);
    vitesse = min(vitesse, (int)MAX_PWM_VALUE_SAFETY);
    //vitesse = vitesse > 10 ? vitesse : 0;

    if (!moteurs_arret)
    {
        digitalWrite(M4,dir);
        analogWrite(E4,vitesse);
    }
    else
    {
       analogWrite(E4,0); 
    }

    
}

void BlocMoteurs::set_vitesse_moteur_BD(int vitesse, DirectionMotor dir)
{
    // verification de la validité de la vitesse a envoyer
    
    vitesse = abs(vitesse);
    vitesse = min(vitesse, (int)MAX_PWM_VALUE_SAFETY);
    //vitesse = vitesse > 10 ? vitesse : 0;

    if (!moteurs_arret)
    {
        digitalWrite(M3,dir);
        analogWrite(E3,vitesse);
    }
    else
    {
       analogWrite(E3,0); 
    }


    
}



void BlocMoteurs::motors_on() // il faut activer les moteurs pour qu'il puisse recevoir des commandes vitesse.
{
    moteurs_arret = 0;
}

void BlocMoteurs::motors_stop() // coupe les moteurs et les rends libres.
{
    moteurs_arret = 1;
    analogWrite(E1,0);
    analogWrite(E2,0);
    analogWrite(E3,0);
    analogWrite(E4,0);
    

}






