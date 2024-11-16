//Code fuer die MPU6050-Kalibration

//Bibliotheken fuer den Neigungsmesser
#include <Adafruit_MPU6050.h>
#include <Kalman.h>

//MPU6050
Adafruit_MPU6050 mpu;
double AccX, AccY, AccZ;
double GyroX;
double accAngleX;
Kalman kalmanX;
float curoll = 0;
double rollError = 0;

double elapsedTime, currentTime, previousTime;




void setup() {
  Serial.begin(9600);
  //IMU
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  kalmanX.setAngle(0);

}


void loop(){

  curoll = currentRoll();
  Serial.println(curoll); //Wert kopieren und als Offset eingeben
}




float currentRoll(){
  double roll;
  //Rohdaten werden vom MPU6050 entnommen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Accelerometerdaten
  AccX = a.acceleration.x/9.81;  // X-Achse
  AccY = a.acceleration.y/9.81;  // Y-Achse
  AccZ = a.acceleration.z/9.81;  // Z-Achse


  //Mathematische Berechnung für den Neigungswinkel
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);


  previousTime = currentTime;                           //Vorherige Zeit wird gespeichert
  currentTime = micros();                               //Neue Zeit in Microsekunden
  elapsedTime = (currentTime - previousTime) / 1000000; // Zeitänderung in Sekunden

  //Gyroskopdaten
  GyroX = g.gyro.x * 57.295779578552; //von Radian in Grad

  roll = kalmanX.getAngle(accAngleX, GyroX, elapsedTime) - rollError; //Kalmanfilter berechnet die Neigung minus den Offset

  return roll;
}
