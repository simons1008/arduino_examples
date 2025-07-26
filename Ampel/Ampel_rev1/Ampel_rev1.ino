/***********************************
* Zustandsautomat zum Lernen
* Taste1 schaltet Zustände: ROT - GELB - GRUEN - GRUEN ...
* Taste2 schaltet den Zustandsübergang GRUEN - ROT
* In allen Zuständen wird 1x geblinkt
* Einsatz der Bibliothek StateMachine
* Quellen: https://github.com/hobbyelektroniker/StateMachine sm7
*          https://github.com/jrullan/StateMachine
* Erstellt: 11.08.2022 (Si)
***********************************/
#include <StateMachine.h>

const int STATE_DELAY = 1000;

// Pin - Definitionen
const int PIN_LED_GRUEN = 12;
const int PIN_LED_GELB = 11;
const int PIN_LED_ROT = 10;

const int PIN_TASTE1 = 7;
const int PIN_TASTE2 = 6;

// Konstruktor erzeugt das Objekt machine 
StateMachine machine = StateMachine();

//// Zustände definieren ////
State* KEINE = machine.addState(&keine);
State* ROT = machine.addState(&rot);
State* GELB = machine.addState(&gelb);
State* GRUEN = machine.addState(&gruen);

//// Aktionen in den Zuständen ////
void keine()
{
  digitalWrite(PIN_LED_GRUEN,LOW);
  digitalWrite(PIN_LED_GELB,LOW);
  digitalWrite(PIN_LED_ROT,LOW);
}

void rot()
{
  if(machine.executeOnce)
  {
    digitalWrite(PIN_LED_ROT,!digitalRead(PIN_LED_ROT));
    delay(100);
  }
}

void gelb()
{
  if(machine.executeOnce)
  {
    digitalWrite(PIN_LED_GELB,!digitalRead(PIN_LED_GELB));
    delay(100);
  }
}

void gruen()
{
  digitalWrite(PIN_LED_GRUEN,!digitalRead(PIN_LED_GRUEN));
  delay(100);
}

//// Bedingungen für Zustandsübergänge ////
bool taste1_gedrueckt()
{
  if (digitalRead(PIN_TASTE1) == LOW) 
  {
    delay(300);
    return true; 
  }
  else
  {
    return false; 
  }
}

bool taste2_gedrueckt()
{
  if (digitalRead(PIN_TASTE2) == LOW) 
  {
    delay(300);
    return true; 
  }
  else
  {
    return false; 
  }
}

//// Initialisierung ////
void setup()
{
  //// Pins vorbereiten ////
  pinMode(PIN_LED_GRUEN,OUTPUT);
  pinMode(PIN_LED_GELB,OUTPUT);
  pinMode(PIN_LED_ROT,OUTPUT);
  
  pinMode(PIN_TASTE1,INPUT_PULLUP);
  pinMode(PIN_TASTE2,INPUT_PULLUP);
 
  //// Zustandsübergänge definieren ////
  ROT->addTransition(&taste1_gedrueckt, GELB);
  GELB->addTransition(&taste1_gedrueckt, GRUEN);
  GRUEN->addTransition(&taste1_gedrueckt, ROT);
  
  ROT->addTransition(&taste2_gedrueckt, KEINE);
  GELB->addTransition(&taste2_gedrueckt, KEINE);
  GRUEN->addTransition(&taste2_gedrueckt, KEINE);

  KEINE->addTransition(&taste1_gedrueckt, ROT);
}

//// Aktionen ////
void loop() 
{
  machine.run();
}
