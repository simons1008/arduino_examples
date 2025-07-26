// Ampel_rev2.ino

// Zustandsautomat mit den Zuständen
// GRUEN
// GELB
// ROT
// Zustandsübergänge mit Taste1 und Taste2
// Quellen: https://github.com/hobbyelektroniker/StateMachine sm7
//          https://github.com/jrullan/StateMachine
// Erstellt: 12.08.2022 (Si)

// Bibliotheken einbinden
#include <StateMachine.h>	// Steuert den Zustandsautomaten
#include <neotimer.h>		  // Timer für die Tasten und das Blinken

// Definition der Pins der Ampel LEDs
const int PIN_LED_GRUEN = 12;
const int PIN_LED_GELB = 11;
const int PIN_LED_ROT = 10;

// Definition der Pins der Tasten
const int PIN_TASTE1 = 7;
const int PIN_TASTE2 = 6;

// Timer für Blinken und Debouncing
Neotimer blinkTimer = Neotimer(500);
Neotimer debounceTimer = Neotimer(250);

// Zustandsautomat erzeugen
StateMachine machine = StateMachine();

// Aktionen in den Zuständen
// Die Ampel blinkt grün
void gruen()
{
  digitalWrite(PIN_LED_ROT,LOW);
  digitalWrite(PIN_LED_GELB,LOW);
  if(blinkTimer.repeat())
  {
    digitalWrite(PIN_LED_GRUEN,!digitalRead(PIN_LED_GRUEN));
  }
  if(machine.executeOnce)
  {
    Serial.println("Die Ampel blinkt grün");
  }
}
// Die Ampel ist gelb
void gelb()
{
  digitalWrite(PIN_LED_ROT,LOW);
  digitalWrite(PIN_LED_GELB,HIGH);
  digitalWrite(PIN_LED_GRUEN,LOW);
}
// Die Ampel ist rot
void rot()
{
  digitalWrite(PIN_LED_ROT,HIGH);
  digitalWrite(PIN_LED_GELB,LOW);
  digitalWrite(PIN_LED_GRUEN,LOW);
}
// Keine Ampel LED leuchtet
void keine()
{
  digitalWrite(PIN_LED_ROT,LOW);
  digitalWrite(PIN_LED_GELB,LOW);
  digitalWrite(PIN_LED_GRUEN,LOW);
}

// Funktionen der Zustandsübergänge
// Taste1 gedrückt?
bool taste1_gedrueckt()
{
  if (debounceTimer.debounce(digitalRead(PIN_TASTE1) == 0))
  {
    return true; 
  }
  else
  {
    return false; 
  }
}
// Taste2 gedrückt?
bool taste2_gedrueckt()
{
  if (debounceTimer.debounce(digitalRead(PIN_TASTE2) == 0))
  {
    return true; 
  }
  else
  {
    return false; 
  }
}

// Definition der Zustände
// Initial ist der erste Zustand
State* GRUEN = machine.addState(&gruen);
State* GELB = machine.addState(&gelb);
State* ROT = machine.addState(&rot);
State* KEINE = machine.addState(&keine);

// Initialisierung
void setup()
{
  // Pins vorbereiten 
  pinMode(PIN_LED_GRUEN,OUTPUT);
  pinMode(PIN_LED_GELB,OUTPUT);
  pinMode(PIN_LED_ROT,OUTPUT);
  pinMode(PIN_TASTE1,INPUT_PULLUP);
  pinMode(PIN_TASTE2,INPUT_PULLUP);
  
  // Debugging
  Serial.begin(115200);
 
  // Definition der Zustandsübergänge durch Taste1
  GRUEN->addTransition(&taste1_gedrueckt, GELB);
  GELB->addTransition(&taste1_gedrueckt, ROT);
  ROT->addTransition(&taste1_gedrueckt, GRUEN);
  KEINE->addTransition(&taste1_gedrueckt, GRUEN);

  // Definition der Zustandsübergänge durch Taste2
  GRUEN->addTransition(&taste2_gedrueckt, KEINE);
  GELB->addTransition(&taste2_gedrueckt, KEINE);
  ROT->addTransition(&taste2_gedrueckt, KEINE);

  // Der Debounce Timer muss gestartet werden!!
  debounceTimer.start();
}

// Loop
void loop()
{
  machine.run();
}
