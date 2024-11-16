//Code fuer die Kommandozentrale

//Bibliotheken
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Definition des Radiomodules
RF24 radio(8, 7);

//Kommunikation: Adresse fuer Befehle
const byte commandaddress[][6] = {"monor", "convr"};


//Fuer den Input
int currentstate;
//Zahl, welche zum Stoppen geschickt wird
int st = -602223;

//Wartezeit, damit man nicht mit Nachrichten ueberschwemmt wird
int wait = 1250;

void setup() {
  Serial.begin(9600);
  Serial.println("Mit dem willst du Kommunikation aufnehmen?");
  Serial.println("0. kreiselstabilisierte Einschienenbahn");
  Serial.println("1. herkoemmliche Eisenbahn");

  currentstate = Serial.available();
  //Warten, bis Antwort kommt
  while(Serial.available() ==  currentstate){
    //Do nothing
  }
  int choice = Serial.parseInt();

  //Je nach dem, was man gewaehlt hat, kommt diese Antwort
  switch(choice){
    case 0:
      Serial.println("Du hast die kreiselstabilisierte Einschienenbahn gewaehlt");
      break;
    case 1:
      Serial.println("Du hast die herkoemmliche Eisenbahn gewaehlt");
      break;
    default:
      Serial.println("Ungueltige Eingabe, setzte automatisch auf die kreiselstabilisierte Einschienenbahn");
      choice = 0;
      break;
  }

  delay(wait);

  //Geschwindigkeitsrohwert eingeben
  Serial.println("Welche Geschwindigkeit willst du einstellen? (Zahl zwischen 1 und 255)");

  currentstate = Serial.available();

  //Wieder warten, bis eine Antort kommt
  while(Serial.available() == currentstate){
    //Do nothing
  }
  
  int setspeed = Serial.parseInt();
  if(0 > setspeed || 255 < setspeed){
    Serial.println("Invalide Geschwindigkeitsangabe, setze Geschwindigkeit automatisch auf 255");
    setspeed = 255;
    delay(wait);
  }

  Serial.print("Du hast dich fuer ");
  Serial.print(setspeed);
  Serial.println(" entschieden");
  delay(wait);
  
  //Verbindung wird mit der Bahn hergestellt
  radio.begin();
  radio.openWritingPipe(commandaddress[choice]);
  //radio.openReadingPipe(1, speedaddress[choice]);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  //Pruefen, ob eine Verbindung hergestellt werden kann
  bool success = radio.write(&setspeed, sizeof(setspeed));
  if(success){
    Serial.println("Geschwindigkeit wurde gesendet");
    delay(wait);
    Serial.println("Gib das Zeichen zum Start");
  }else{
    Serial.println("Keine Verbindung, bitte pruefe die Bahn");
  }

  //Zeichen fuer den Start
  currentstate = Serial.available();
  while(Serial.available() == currentstate){
    //wait
  }

  radio.write(&setspeed, sizeof(setspeed));
  Serial.println("Die Bahn faehrt");
  delay(500);
  Serial.println("Gib das Zeichen zum Anhalten");
  
  //Warten, bis neues Zeichen kommt
  currentstate = Serial.available();
  
  while(Serial.available() == currentstate){
    //wait
  }

  /*
  Serial.println("Gib das Zeichen zum streamen");
  
  //Warten, bis neues Zeichen kommt
  currentstate = Serial.available();
  
  while(Serial.available() == currentstate){
    //wait
  }

  radio.write(&setspeed, sizeof(setspeed));

  //Faengt an, Daten zu erhalten
  radio.startListening();
  bool stop = false;
  
  currentstate = Serial.available();

  //Schleife zum Daten lesen/stoppen
  while(!stop){

    if (radio.available()) {
      float streamed = 0;
      radio.read(&streamed, sizeof(streamed));
      Serial.println(streamed, 7);
      //delay(500);
    }

    if(Serial.available() != currentstate){
      stop = true;
    }
  }
  */

  Serial.print("Stopp");
  radio.stopListening();

}

void loop() {
  //Sicherstellen, dass die Bahn den Befehl nicht verpasst hat
  radio.stopListening();
  radio.write(&st, sizeof(st));
  
}