//Code fuer die kreiselstabilisierte Einschienenbahn

//Bibliotheken fuer den Neigungsmesser
#include <Adafruit_MPU6050.h>
#include <Kalman.h>

//Bibliothek fuer den Servomotor und den ESC
#include <Servo.h>

//Bibliothek fuer den PID-Regler
#include <PID_v1.h>

//Bibliotheken fuer das Transceivermodul
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Pins und Variablen definieren

//MPU6050
Adafruit_MPU6050 mpu;
double AccX, AccY, AccZ;
double GyroX;
double accAngleX;
Kalman kalmanX;
double elapsedTime, currentTime, previousTime; //Um die vergangene Zeit zu messen

double rollError = -0.20; //Offsetwert


//Servomotor des Kreisels
Servo gyro;
const int gyroposition = 90; //Dieser Wert muss dem Wert ensprechen, bei dem der Kreisel parallel zur Basis verlaeuft
int currentposition = gyroposition;
int maxrotation = 50; //Maximale Rotation des Kreisels, damit er nicht in die Basis reinfraest
Servo flywheel; //Auf Pin 6 (Der ESC wird als Servomotor definiert, da er dasselbe Signal braucht)


//Stabilisation:
double setpoint; //Nullposition, die angestrebt wird
double input; //Die gemessene Neigung des MPU fuer den PID
double output; //Winkel des Servomotors, gesteuert vom PID
double dynamicsetpoint; //dynamische Nullposition
double curoll; //aktuelle Neigung
int mill; //Variable zum Zeitmessen

//Konstanten fuer die Stabilisation
double Pk1 = 1.35;
double Ik1 = 32;
double Dk1 = 0.135;

//Definition des PID-Reglers
PID stabiliser(&input, &output, &setpoint, Pk1, Ik1 , Dk1, DIRECT);

//Laufwerk
#define forwards 4
#define backwards 5
#define speed 3

int setspeed = 0;


//Kommunikation
RF24 radio(8, 7); //Definition des Moduls

const byte monorail[6] = "monor"; //Adresse zum Erhalten von Befehlen
const byte data[6] = "data_"; //Adresse zum Schicken der Neigung


void setup() {
  Serial.begin(9600);
  
  //MPU6050
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  kalmanX.setAngle(0);

  //ESC und Servomotor
  flywheel.attach(6, 1000, 2000);
  gyro.attach(9);

  //PID
  stabiliser.SetMode(AUTOMATIC);
  stabiliser.SetOutputLimits(-60, 60); //Begrenzung des Winkels
  stabiliser.SetSampleTime(25);
  gyro.write(gyroposition);
  flywheel.write(0);
  delay(3000);

  //Kommunikationseinstellungen
  radio.begin();
  radio.openWritingPipe(data);
  radio.openReadingPipe(1, monorail);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //Fahrwerk
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
    currentRoll(); //Messung der Neigung
  }
  //Geschwindigkeit empfangen
  radio.read(&setspeed, sizeof(setspeed));
  //Serial.println(setspeed);
  delay(4000); //kurze Pause

  //Kreiselbeschleunigung
  mill = millis();
  while(millis()-mill<20000){
    flywheel.write(20);
    //Serial.println(currentRoll());
  }

  //Warten auf das Startsignal
  while(!radio.available()){
  
    stabilise();
    //Die Schnipsel, die es braucht, um zu stabilisieren
    radio.stopListening();
    radio.write(&curoll, sizeof(curoll));
    radio.startListening();
    if(radio.available()){
      //nichts
    }  
  }

  digitalWrite(forwards, HIGH);
  accelerate(setspeed);
  radio.flush_rx();
  radio.flush_tx();

  //Warten, bis man das Singal zum Stoppen bekommt
  
  while(!radio.available()){
    stabilise();
    //Die Schnipsel, die es braucht, um zu stabilisieren
    radio.stopListening();
    radio.write(&curoll, sizeof(curoll));
    radio.startListening();
    if(radio.available()){
      //nichts
    } 
  }


  digitalWrite(forwards, LOW);
  analogWrite(speed, 0);
}

void loop() {
  //Sicherstellen, dass nicht mehr gefahrt wird
  digitalWrite(forwards, LOW);
  analogWrite(speed, 0);
  flywheel.write(0);
}

//Stabilisierungsfunktion
void stabilise(){
  curoll = currentRoll();
  //Serial.println(curoll);

  //Berechnung des dynamischen Mittelpunktes
  dynamicsetpoint = dynamicsetpoint + output;
  dynamicsetpoint = constrain(dynamicsetpoint, -0.1, 0.1);
  setpoint = dynamicsetpoint;
  input = curoll;
  //Serial.println(dynamicsetpoint);
  stabiliser.Compute();


  output = constrain(output, -maxrotation, maxrotation);


  //Servomotor rotieren
  currentposition = gyroposition - output;
  gyro.write(gyroposition - output);
}


//Aktuelle Neigung
float currentRoll(){
  double roll;
  //Rohdaten werden vom MPU6050 entnommen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Accelerometerdaten
  AccX = a.acceleration.x/9.81;  // X-Achse
  AccY = a.acceleration.y/9.81;  // Y-Achse
  AccZ = a.acceleration.z/9.81;  // Z-Achse


  //Mathematische Berechnung fuer den Neigungswinkel
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);


  previousTime = currentTime;                           //Vorherige Zeit wird gespeichert
  currentTime = micros();                               //Neue Zeit in Microsekunden
  elapsedTime = (currentTime - previousTime) / 1000000; // Zeitaenderung in Sekunden

  //Gyroskopdaten
  GyroX = g.gyro.x * 57.295779578552; //von Radian in Grad

  roll = kalmanX.getAngle(accAngleX, GyroX, elapsedTime) - rollError; //Kalmanfilter berechnet die Neigung minus den Offset

  return roll;
}

//Beschleunigen
void accelerate(int wishedspeed){
  int i = 0;
  while(i < wishedspeed){
    analogWrite(speed, i);
    i = i + 25;

    stabilise();
    radio.stopListening();
    radio.write(&curoll, sizeof(curoll));  
    radio.startListening();

    if(radio.available()){
      //Warten
    }
  }
  analogWrite(speed, wishedspeed);
}