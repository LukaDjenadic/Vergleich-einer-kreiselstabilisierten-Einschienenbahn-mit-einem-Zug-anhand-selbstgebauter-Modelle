//Code fuer die herkoemmliche Eisenbahn

//Bibliotheken fuer das Transceivermodul
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Pins und Variablen definieren:

//Laufwerk
#define forwards 4
#define backwards 5
#define speed 3
int setspeed = 0;

//Variablen fuer das Beschleunigen in ms
int dt = 40;

//Kommunikation
RF24 radio(8, 7); //Definition des Moduls


const byte conventionalrail[6] = "convr"; //Adresse zum Erhalten von Befehlen


void setup() {
  Serial.begin(9600);
  
  //Kommunikationseinstellungen
  radio.begin();
  radio.openWritingPipe(speeddual);
  radio.openReadingPipe(1, dualrail);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //Laufwerk
  pinMode(speedmeasure, INPUT);

  pinMode(forwards, OUTPUT);
  pinMode(backwards, OUTPUT);
  pinMode(speed, OUTPUT);

  digitalWrite(forwards, LOW);
  digitalWrite(backwards, LOW);
  analogWrite(speed, 0);

  //************************************************
  //Fahren

  //Warten, bis man den PWM-Wert bekommt
  while(!radio.available()){
    //warten
  }
  //Geschwindigkeit empfangen
  radio.read(&setspeed, sizeof(setspeed));
  //Serial.println(setspeed);

  //Warten auf das Startsignal
  while(!radio.available()){
    //warten
  }
  digitalWrite(forwards, HIGH);
  accelerate(setspeed);
  radio.flush_rx();
  radio.flush_tx();

  //Warten, bis man das Signal zum Stoppen bekommt
  
  while(!radio.available()){
    //warten
  }
  //Stoppen
  digitalWrite(forwards, LOW);
  analogWrite(speed, 0);
}

void loop() {
  //Sicherstellen, dass nicht mehr gefahrt wird
  digitalWrite(forwards, LOW);
  analogWrite(speed, 0);
}

//Beschleunigungsfunktion
void accelerate(int wishedspeed){
  int i = 0;
  while(i < wishedspeed){
    analogWrite(speed, i);
    Serial.println(i);
    i = i + 20;
    delay(dt);
  }
  analogWrite(speed, wishedspeed);
}