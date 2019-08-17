#include <Wire.h>
#include <I2Cdev.h>
#include <ADXL345.h>
#include <L3G4200D.h>
#include <HMC5883L.h>
#include <BMP085.h>
#include <MahonyAHRS.h>

//create sensor instances
L3G4200D gyro;
ADXL345 accel;
HMC5883L mag;
BMP085 barometer;
Mahony filter;

int16_t gx, gy, gz;
int16_t ax, ay, az;
int16_t mx, my, mz;
float temperature, pressure, altitude;
float roll, pitch, heading;
unsigned long previousMillis  = 0;
unsigned long currentMillis   = 0;
int interval = 40; //25 Hz

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  filter.begin(25);
  
  //initialize device
  gyro.initialize();
  gyro.setOutputDataRate(100);
  gyro.setFullScale(2000);
   
  accel.initialize();
  accel.setRange(3);
  accel.setFullResolution(1);
  accel.setOffsetX(-1);
  accel.setOffsetY(1);
  accel.setOffsetZ(3);

  mag.initialize();
  mag.setDataRate(6);

  barometer.initialize();
}

void loop(void){
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval == true ) {

    //read data
    gyro.getAngularVelocity(&gx, &gy, &gz);
    accel.getAcceleration(&ax, &ay, &az);
    mag.getHeading(&mx, &my, &mz);
  
    //convert raw data to g
    float accelx = ax/256.0;
    float accely = ay/256.0;
    float accelz = az/256.0;

    //calibrate accel
    accelx = (accelx - 0.035)/1.04;
    accely = (accely - 0.0)/1.03;
    accelz = (accelz + 0.12)/0.99;
  
    //gyro offset and convert to dps
    float gyrox = (gx - 21.09)*0.07;
    float gyroy = (gy + 13.81)*0.07;
    float gyroz = (gz - 11.61)*0.07;
    
    //convert and calibrate raw data to uT
    float magx = mx/1100.0; //raw to Gauss
    float magy = my/1100.0; //raw to Gauss
    float magz = mz/980.0; //raw to Gauss
    
    magx *= 100000.0; //Gauss to nT
    magy *= 100000.0; //Gauss to nT
    magz *= 100000.0; //Gauss to nT
    
    magx += 16125.634877; 
    magy += 18995.378933;
    magz += 14877.398463;
    
    float magcalx = 1.170281*magx - 0.000184*magy + 0.006579*magz;
    float magcaly = -0.000184*magx + 1.130717*magy - 0.012456*magz;
    float magcalz = 0.006579*magx - 0.012456*magy + 1.137240*magz; 
  
    magcalx *= 0.001; //nT to uT (Library expect this format)
    magcaly *= 0.001; //nT to uT (Library expect this format)
    magcalz *= 0.001; //nT to uT (Library expect this format)

    barometer.setControl(BMP085_MODE_TEMPERATURE);
    temperature = barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    pressure = barometer.getPressure();   
    altitude = barometer.getAltitude(pressure);
/*
    Serial.print(currentMillis);
    Serial.print(" ");
    Serial.print(gyrox);
    Serial.print(" ");
    Serial.print(gyroy);
    Serial.print(" ");
    Serial.print(gyroz);
    Serial.print(" ");
    Serial.print(accelx);
    Serial.print(" ");
    Serial.print(accely);
    Serial.print(" ");
    Serial.print(accelz);
    Serial.print(" ");
    Serial.print(magcalx);
    Serial.print(" ");
    Serial.print(magcaly);
    Serial.print(" ");
    Serial.print(magcalz);
    Serial.print(" ");
    Serial.print(temperature);
    Serial.print(" ");
    Serial.print(pressure);
    Serial.print(" ");
    Serial.println(altitude);
*/

    //filter.update(gyrox, gyroy, gyroz, accelx, accely, accelz, magcalx, magcaly, magcalz);
    filter.updateIMU(gyrox, gyroy, gyroz, accelx, accely, accelz);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print(heading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);
    
    previousMillis = currentMillis;
  }
}
