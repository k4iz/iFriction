#include <Wire.h>

  //LDR definitions
  int pin_sensor_ini = 1;
  int pin_sensor_end = 2;
  float ini_reading;
  float end_reading;
  
  //constants
  unsigned long time_1, time_2;
  float distance = 0.2;
  double measured_time_microsec;
  double measured_time_sec;
  double calculated_speed;
  double calculated_acceleration;
  float gravity = 9.81;


  //angle constants
  const int MPU_addr=0x68; 
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

  int minVal=265; 
  int maxVal=402;

  double x; 
  double y; 
  double z;

  float x_rad;
  float static_friction;
  float kinetic_friction;

void setup() 
{   
    //angle I2C address
    Wire.begin(); 
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
 
    //define speed sensors pins
    pinMode(pin_sensor_ini, INPUT);
    pinMode(pin_sensor_end, INPUT);

    //Initialize serial
    Serial.begin(9600);
}


void loop() 
{
    //initializing sensors
    Serial.println("Waiting for readings...");

    ini_reading = 100000/analogRead(pin_sensor_ini);
    end_reading = 100000/analogRead(pin_sensor_end);

    //get readings loop
/*----------------------------------------------------------------*/
    while(ini_reading > 200)
    {
        ini_reading = 100000/analogRead(pin_sensor_ini);
    }

    if(ini_reading <= 200)
    {
        //measure angle
        Wire.beginTransmission(MPU_addr); 
        Wire.write(0x3B); 
        Wire.endTransmission(false); 
        Wire.requestFrom(MPU_addr,14,true); 
        
        AcX=Wire.read()<<8|Wire.read(); 
        AcY=Wire.read()<<8|Wire.read(); 
        AcZ=Wire.read()<<8|Wire.read(); 
        
        int xAng = map(AcX,minVal,maxVal,-90,90); 
        int yAng = map(AcY,minVal,maxVal,-90,90); 
        int zAng = map(AcZ,minVal,maxVal,-90,90);
    
        x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); 
        y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); 
        z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

        x_rad = x*3.14/180;
        static_friction = tan(x_rad);
        
        Serial.print("Static friction = ");
        Serial.println(static_friction);
    }
    
    while(ini_reading <= 200)
    {
        time_1 = micros();
        ini_reading = 100000/analogRead(pin_sensor_ini);
    }

    while(end_reading > 200)
    {
        end_reading = 100000/analogRead(pin_sensor_end);
    }

    while(end_reading <= 200)
    {
        time_2 = micros();
        end_reading = 100000/analogRead(pin_sensor_end);
    }
 /*--------------------------------------------------------*/

    //calculate speed
    measured_time_microsec = (time_2 - time_1);
    measured_time_sec = measured_time_microsec/1000000;
    calculated_speed = distance/measured_time_sec;

    //calculate acceleration
    calculated_acceleration = 2*distance/pow(measured_time_sec, 2);
   
    //calculate kinetic friction
    kinetic_friction = tan(x_rad) - calculated_acceleration/(gravity*cos(x_rad));

    //show measured info
    Serial.print("Angle in rad: "); Serial.println(x_rad);
    Serial.print("Time 1: "); Serial.println(time_1);
    Serial.print("Time 2: "); Serial.println(time_2);
    Serial.print("Measured time [micro sec]: "); Serial.println(measured_time_microsec);
    Serial.print("Measured time [sec]: "); Serial.println(measured_time_sec);
    Serial.print("Calculated speed: "); Serial.println(calculated_speed);
    Serial.print("Calculated acceleration: "); Serial.println(calculated_acceleration);
    Serial.print("Kinetic friction: "); Serial.println(kinetic_friction);
    Serial.println("");
    
    Serial.print("AngleX= "); Serial.println(x);
    Serial.print("AngleY= "); Serial.println(y);
    Serial.print("AngleZ= "); Serial.println(z);
    Serial.println("");
    
}
