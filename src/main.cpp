#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BlocMoteurs.h"
#include "SpeedPIDController.h"
#include "EtatDynamiqueRobot.h"
#include "Encoder4Mot.h"
#include "Odometrie.h"
#include <cstdio>
#include <SpeedController_PP.h>

#include <global.h>






uint32_t timerstart=0;
int8_t updateparam = 0;

bool reset_odo = false;
bool last_reset_odo = false;
bool tuning_mode = false;
std::vector<std::vector<double>> toprint_vector;
std::vector<std::vector<double>> toprint_vector_2;


Encoder4Mot* encoder;
BlocMoteurs* mot;
Odometrie* odo;
uint32_t timer;
int printVar;
SpeedController_PP* control;


int rate_ms = 30;



float tab[100][4];













void setup() {
    

  Serial.begin(115200);

  timerstart = millis();



  mot = new BlocMoteurs();
  encoder =  new Encoder4Mot();
  odo = new Odometrie(1000*rate_ms,encoder);
  control = new SpeedController_PP(mot,encoder);

  delay(5000);

 



  // mot->motors_on();
  // mot->commande_vitesses(0.3,0.3,0.3,0.3);
  // delay(1000);
  // int i = 0;
  // timer = millis();
  // while( i < 10)
  // {
  //   if(millis() > timer + 30)
  //   {
  //     timer = millis();
  //     std::vector<double> enco = encoder->Encoder4MotUpdate();
  //     for(int j = 0; j< 4; j++)
  //     {
  //       tab[i][j] = enco[j];

  //     }
  //     i+=1;
  //   }
  // }
  // mot->commande_vitesses(0.9,0.9,0.9,0.9);
  // while( i < 100)
  // {
  //   if(millis() > timer + 30)
  //   {
  //     timer = millis();
  //     std::vector<double> enco = encoder->Encoder4MotUpdate();
  //     for(int j = 0; j< 4; j++)
  //     {
  //       tab[i][j] = enco[j];

  //     }
  //     i+=1;
  //   }

  // }
  // mot->motors_stop();

  
  // for(int j = 0; j<100; j++)
  // {


  //   for(int k = 0; k<4; k++)
  //   {
  //     Serial.print(tab[j][k]);
  //     Serial.print(";");
  //   }
  //   if(j<10)
  //   {
  //     Serial.print("1;");
  //   }
  //   else
  //   {
  //     Serial.print("10;");
  //   }
  //   Serial.println(j);
    
    
  // }

 
  control->define_setpoint(3.0,3.0,3.0,3.0);
  timer = millis();
  while(millis() < timer + 5000)
  {
    if(control->update_controller())
    {
      std::valarray<double> toprint = control->getmotors_input();
      std::vector<double> toadd {toprint[0],toprint[1],toprint[2],toprint[3]};
      toprint_vector.push_back(toadd);
      toprint_vector_2.push_back(encoder->GetSpeeds());

    }

  }
  mot->motors_stop();
  for(int i = 0; i<toprint_vector.size();i++)
  {
    for(int k = 0; k<4; k++)
    {
      Serial.print(toprint_vector[i][k]);
      Serial.print(";");
    }
    for(int k = 0; k<4; k++)
    {
      Serial.print(toprint_vector_2[i][k]);
      Serial.print(";");
    }
    Serial.println(i);
    
  }


}

void loop() {


 

   
}












