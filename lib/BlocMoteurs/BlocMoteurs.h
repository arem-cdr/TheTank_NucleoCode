#pragma once

#include <Arduino.h>
#include "Reglages.h"


typedef enum direction
{
	FWD,
	BWD
}DirectionMotor; // Type pour gérer la direction (FWD=0 , BWD=1) des moteurs



class BlocMoteurs // Gere les moteurs de maniere securise
{
	public:
		// Constructeur
		BlocMoteurs();
		~BlocMoteurs();
		
		// Pour envoyer une consigne de vitesse aux moteurs
		// les paramètres sont des flottants entre -1 et 1
		// (si la vitesse max est trop faible, il faut modifier la constante MAX_VITESSE dans Reglages.h)
		// (ne pas oublier d'utiliser la méthode motors_on si les moteurs ont été arrêtés avant)
		void commande_vitesses(float vitesse_normalisee_FD, float vitesse_normalisee_FG, float vitesse_normalisee_BD, float vitesse_normalisee_BG );

		// Methodes pour bloquer les moteurs
		// (il faut appeler la méthode motors_on pour pouvoir de nouveau envoyer des consignes
		// de vitesse aux moteurs)
		void motors_stop();
		
		
		// Methode pour autoriser la rotation des moteurs
		void motors_on();

		//Permet d'obtenir l'etat des moteurs : arret ou on
		bool get_moteurs_arret(){return moteurs_arret;};

		bool getSaturation(){return saturation;}; // permet d'enregistrer jusqu'a la consigne suivante si une consigne trop grande a ete demande

	private:
		// Methodes pour définir la vitesse affectée à chaque moteur
		void set_vitesse_moteur_FG(int vitesse, DirectionMotor dir); //frontleft
		void set_vitesse_moteur_FD(int vitesse, DirectionMotor dir); //frontright
		void set_vitesse_moteur_BG(int vitesse, DirectionMotor dir); //backleft
		void set_vitesse_moteur_BD(int vitesse, DirectionMotor dir); //backright
		
		bool moteurs_arret;
		const int E1 = 3; ///<Motor1 Speed
		const int E2 = 11;///<Motor2 Speed
		const int E3 = 5; ///<Motor3 Speed
		const int E4 = 6; ///<Motor4 Speed

		const int M1 = 4; ///<Motor1 Direction
		const int M2 = 12;///<Motor2 Direction
		const int M3 = 8; ///<Motor3 Direction
		const int M4 = 7; ///<Motor4 Direction

		// Un coefficient entre 0 et 1.0 pour limiter facilement la vitesse pendant un mouvement particulier
		bool saturation;

	

};





