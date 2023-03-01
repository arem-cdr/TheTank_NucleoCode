#pragma once






#define PPR 330 // Nombre d'impulsions codeurs par révolutions

#define MAX_PWM_VALUE_THEORETICAL 255 //Correspond a une PWM de rapport cyclique 1, correspond à 319-255 RPM selon la charge
#define MAX_PWM_VALUE_SAFETY 200 // saturation de la PWM pour préserver le moteur

#define COEF_FG 1.0 // Coefficients pour baisser spécifiquement la vitesse d'un moteur (legacy)
#define COEF_BG 1.0
#define COEF_FD 1.0
#define COEF_BD 1.0

#define INIT_X 0.0 
#define INIT_Y 0.0 
#define INIT_THETA 0.0


#define TICKS_PAR_TOUR 1320 // correspond à la précision de l'encodeur magnétique
#define RAYON_ROUE 48.5 // Placeholder, a mesurer // en mm


#define L1 365 // A MESURER // en mm ; Largeur entre le centre du robot et le milieu d'une roue (milieu vers gauche par rapport à l'avant)
#define L2 252 // A MESURER // en mm ; Longeur entre le centre du robot et le milieu d'une roue (milieu vers avant par rapport à l'avant)

#define L1PL2 308.5

extern float global_L1pL2;
extern float global_RayonRoues; 