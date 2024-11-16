//Code fuer den Zeitmesser:

//Startsensor:
#define trigstart 8
#define echostart 9

//Endsensor:
#define trigend 6
#define echoend 7

//Zeit
int t0 = 0;
int t1 = 0;

float minimumdistance = 10;

void setup(){

  Serial.begin(9600);
  //Konfiguration
  pinMode(trigstart, OUTPUT);
  pinMode(echostart, INPUT);
  pinMode(trigend, OUTPUT);
  pinMode(echoend, INPUT);

}
/*
void loop(){
  Serial.print(startm());
  Serial.print(", ");
  Serial.println(endm());
}
*/

void loop() {
  while(startm() > minimumdistance){
    //warten
  }
  t0 = millis();
  while(endm() > minimumdistance){
    //warten
  }
  t1 = millis();
  Serial.println((t1 - t0));
  
}


//Funktionen zum Messen der Distanz:

float startm(){
  digitalWrite(trigstart, LOW);
  delay(5);

  digitalWrite(trigstart, HIGH);
  delay(10);
  digitalWrite(trigstart, LOW);

  return pulseIn(echostart, HIGH) * 0.034 / 2;
}

float endm(){
  digitalWrite(trigend, LOW);
  delay(5);

  digitalWrite(trigend, HIGH);
  delay(10);
  digitalWrite(trigend, LOW);

  return pulseIn(echoend, HIGH) * 0.034 / 2;
}