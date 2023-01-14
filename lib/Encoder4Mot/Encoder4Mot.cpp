#include "Encoder4Mot.h"
#include "Reglages.h"


uint8_t Encoder::ISRUsedA = 0;           // allocation table for the globalISRx()
Encoder* Encoder::myInstanceA[MAX_ISR]; // callback instance handle for the ISR
uint8_t Encoder::ISRUsedB = 0;           // allocation table for the globalISRx()
Encoder* Encoder::myInstanceB[MAX_ISR]; // callback instance handle for the ISR 

// ISR for each myISRId
void Encoder::globalISR0_A(void) { Encoder::myInstanceA[0]->instanceISRA(); }
void Encoder::globalISR1_A(void) { Encoder::myInstanceA[1]->instanceISRA(); }
void Encoder::globalISR2_A(void) { Encoder::myInstanceA[2]->instanceISRA(); }
void Encoder::globalISR3_A(void) { Encoder::myInstanceA[3]->instanceISRA(); }
void Encoder::globalISR0_B(void) { Encoder::myInstanceB[0]->instanceISRB(); }
void Encoder::globalISR1_B(void) { Encoder::myInstanceB[1]->instanceISRB(); }
void Encoder::globalISR2_B(void) { Encoder::myInstanceB[2]->instanceISRB(); }
void Encoder::globalISR3_B(void) { Encoder::myInstanceB[3]->instanceISRB(); }


Encoder::Encoder(PinName pin_Name_CH_A, PinName pin_Name_CH_B, motorPlacement placement, bool inversePolarity): placementEncoder(placement), _inversePolarity(inversePolarity) 
{
    lastSpeed = 0.0;
    speed = 0.0;
    count = 0;
    lastCount = 0;
    ellapsedTime = 0;
    timer = micros();

    pinIntA = pinNametoDigitalPin(pin_Name_CH_A);
    pinIntB = pinNametoDigitalPin(pin_Name_CH_B);
    int32_t irqA = digitalPinToInterrupt(pinIntA);
    int32_t irqB = digitalPinToInterrupt(pinIntB);

    if (irqA != NOT_AN_INTERRUPT && irqB != NOT_AN_INTERRUPT)
    {
      pinMode(pinIntA, INPUT_PULLDOWN);
      pinMode(pinIntB, INPUT_PULLDOWN);
      stateA = digitalRead(pinIntA);
      stateB = digitalRead(pinIntA);
   
      // assign ourselves a ISR ID ...
      myISRIdA = UINT8_MAX;
      myISRIdB = UINT8_MAX;
      for (uint8_t i = 0; i < MAX_ISR; i++)
      {
        if (!(ISRUsedA & _BV(i)))    // found a free ISR Id?
        {
          myISRIdA = i;                 // remember who this instance is
          myInstanceA[myISRIdA] = this; // record this instance
          ISRUsedA |= _BV(myISRIdA);    // lock this in the allocations table
          break;
        }
      }
      // ... and attach corresponding ISR callback from the lookup table
      {
        static void((*ISRfuncA[MAX_ISR])(void)) =
        {
          globalISR0_A, globalISR1_A, globalISR2_A, globalISR3_A,
        };
   
        if (myISRIdA != UINT8_MAX)
          attachInterrupt(irqA, ISRfuncA[myISRIdA], CHANGE);
        else
          irqA = NOT_AN_INTERRUPT;
      }
      //------------------------------------------------------//

      for (uint8_t i = 0; i < MAX_ISR; i++)
      {
        if (!(ISRUsedB & _BV(i)))    // found a free ISR Id?
        {
          myISRIdB = i;                 // remember who this instance is
          myInstanceB[myISRIdB] = this; // record this instance
          ISRUsedB |= _BV(myISRIdB);    // lock this in the allocations table
          break;
        }
      }
      // ... and attach corresponding ISR callback from the lookup table
      {
        static void((*ISRfuncB[MAX_ISR])(void)) =
        {
          globalISR0_B, globalISR1_B, globalISR2_B, globalISR3_B,
        };
   
        if (myISRIdB != UINT8_MAX)
          attachInterrupt(irqB, ISRfuncB[myISRIdB], CHANGE);
        else
          irqB = NOT_AN_INTERRUPT;
      }
      
    }
    
}

Encoder::~Encoder()
{

    detachInterrupt(digitalPinToInterrupt(pinIntA));
    detachInterrupt(digitalPinToInterrupt(pinIntB));
    ISRUsedA &= ~_BV(myISRIdA);   // free up the ISR slot for someone else
    ISRUsedB &= ~_BV(myISRIdB);
}

void Encoder::instanceISRA(void)
{
    if(stateA == HIGH)
    {
        if(digitalRead(pinIntA)==LOW)
        {
            stateA = LOW;
            if(stateB)
            {
                if(!_inversePolarity)
                {
                    count--;
                }
                else
                {
                    count++;
                }
                
            }
            else
            {
                if(!_inversePolarity)
                {
                    count++;
                }
                else
                {
                    count--;
                }
                
            }

        }

    }
    else
    {
        if(digitalRead(pinIntA) == HIGH)
        {
            stateA = HIGH;
            if(stateB)
            {
                if(!_inversePolarity)
                    count++;
                else
                    count--;
            }
            else
            {
                if(!_inversePolarity)
                    count--;
                else
                    count++;
            }
        }

    }
}

void Encoder::instanceISRB(void)
{
    if(stateB == HIGH)
    {
        if(digitalRead(pinIntB)==LOW)
        {
            stateB = LOW;
            if(stateA)
            {
                if(!_inversePolarity)
                {
                    count++;
                }
                else
                {
                    count--;
                }
                
            }
            else
            {
                if(!_inversePolarity)
                {
                    count--;
                }
                else
                {
                    count++;
                }
                
            }

        }

    }
    else
    {
        if(digitalRead(pinIntB) == HIGH)
        {
            stateB = HIGH;
            if(stateA)
            {
                if(!_inversePolarity)
                    count--;
                else
                    count++;
            }
            else
            {
                if(!_inversePolarity)
                    count++;
                else
                    count--;
            }
        }

    }
}

double Encoder::update()
{
    ellapsedTime = micros() - timer;
    timer = micros();
    ellapsedCount = count - lastCount;
    lastCount = count;
    lastSpeed = speed;
    speed = (1000000.0*ellapsedCount)/((double)ellapsedTime);
    return speed;

}




Encoder4Mot::Encoder4Mot()
{
    for(int i = 0; i<4; i++)
    {
        speeds.push_back(0.0);
        last_speeds.push_back(0.0);
    }
    
    
    
  encoders.push_back(new Encoder(PA_0,PA_1,motorPlacement::FD,true));
  encoders.push_back(new Encoder(PC_10,PC_11,motorPlacement::FG,false));
  encoders.push_back(new Encoder(PB_13,PB_14,motorPlacement::BD,true));
  encoders.push_back(new Encoder(PC_5,PC_6,motorPlacement::BG,false));


}

Encoder4Mot::~Encoder4Mot()
{
}

const std::vector<double> & Encoder4Mot::Encoder4MotUpdate()
{

    last_speeds.assign(speeds.begin(),speeds.end());

    for(int i = 0; i<4; i++)
    {
        speeds[i] = (encoders[i]->update()*2*PI)/(4*PPR);
    }
    // Serial.print(speeds[0]);
    // Serial.print(" ");
    // Serial.print(speeds[1]);
    // Serial.print(" ");
    // Serial.print(speeds[2]);
    // Serial.print(" ");
    // Serial.println(speeds[3]);
    return speeds;
}

