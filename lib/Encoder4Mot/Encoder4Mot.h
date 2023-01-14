#pragma once


#include <vector>
#include <iostream>
using namespace std;
#include <Arduino.h>

#define MAX_ISR 4


typedef enum _motorPlacement
{
    FD, //Moteur avant droit (F = Front) -> deso pour le melange anglais francais
    FG, //Moteur avant gauche
    BD, //Moteur arriere droit (B = Back) -> deso pour le melange anglais francais
    BG  //Moteur arriere gauche
}motorPlacement; // Type pour gerer / enregistrer la position d'un moteur/encodeur

class Encoder // Classe pour gerer un encodeur
{
    private:

        uint8_t myISRIdA; // Id d'interruption relatif au channel A
        uint8_t myISRIdB; // Id d'interruption relatif au channel B
        // Chaque instance de classe possede un id d'interrutption, l'id est necessaire pour etablir une correspondance entre les fonctions d'interrutpions declarés en statiques et les instances d'encodeurs

        uint32_t pinIntA; // Pin d'interruption du channel A
        uint32_t pinIntB; // Pin d'interruption du channel B        
                 
        volatile int32_t count; // Compteur de l'encodeur

        int32_t ellapsedCount; // Ticks d'encodeur entre 2 appels successifs de update
        
        static uint8_t ISRUsedA;    // Keep track of which ISRs (channel A) are used (global bit field)
        static uint8_t ISRUsedB;    // Keep track of which ISRs (channel B) are used (global bit field)
        
        static Encoder* myInstanceA[]; // Callback instance for the ISR of Channel A to reach instanceISR()
        static Encoder* myInstanceB[];

        void instanceISRA(void);    // Instance ISR handler called from static ISR globalISRx
        void instanceISRB(void); 

        bool stateA; // etat du channel A de l'encodeur (HIGH,LOW)
        bool stateB; // etat du channel B de l'encodeur (HIGH,LOW)

        bool _inversePolarity; // permet d'inverser le sens de comptage de l'encodeur (Positif/negatif selon le sens)

        uint32_t timer; // Timer pour compter le temp

        int32_t lastCount; // Valeur du compteur au dernier appel de update

        uint32_t ellapsedTime; // temps ecoule entre le dernier appel de update et celui le precedant

        double lastSpeed; // valeur de la vitesse en tick/s avant l'appel de update 
        double speed;  // valeur de la vitesse en tick/s mesuré au dernier appel de update


        // declare all the [MAX_ISR] encoder ISRs
        static void globalISR0_A(void);
        static void globalISR1_A(void);
        static void globalISR2_A(void);
        static void globalISR3_A(void);
        static void globalISR0_B(void);
        static void globalISR1_B(void);
        static void globalISR2_B(void);
        static void globalISR3_B(void);

        motorPlacement placementEncoder; // Placement de l'encodeur

    public:
        Encoder(PinName pin_Name_CH_A, PinName pin_Name_CH_B, motorPlacement placement, bool inversePolarity );
        ~Encoder();
        int32_t getCount() {return count;};
        int32_t getLastCount() {return lastCount;};
        int32_t getEllapsedTick() { return ellapsedCount;};
        uint32_t getEllapsedTime() {return ellapsedTime;};
        double getSpeed() { return speed;};
        double getLastSpeed() { return lastSpeed;};
        double update(); // permet de mettre a jour les vitesses mesurés


};


class Encoder4Mot
{
private:
    std::vector<double> speeds; // Vitesses des encodeurs FD->BG en rad/s au dernier appel de update
    std::vector<double> last_speeds; // Vitesses des encodeurs FD->BG en rad/s avant le dernier appel de update
    std::vector<Encoder*> encoders; // Encodeurs FD->BG

public:
    Encoder4Mot();
    ~Encoder4Mot();
    const std::vector<double> & Encoder4MotUpdate(); // fonction d'update, retourne speeds en rad/s

    const std::vector<double> & GetSpeeds()const{ return speeds;};
    const std::vector<double> & GetLastSpeeds()const{ return last_speeds;};
    


};

