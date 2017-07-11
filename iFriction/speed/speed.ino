int pin_sensor_ini = 1;
int pin_sensor_end = 2;
unsigned long time_1, time_2;
float measured_time, measured_speed;
float ini_reading;
float end_reading;

void setup() 
{
    Serial.begin(9600);
    pinMode(pin_sensor_ini, INPUT);
    pinMode(pin_sensor_end, INPUT);
}

void loop() 
{
    Serial.println("Waiting for readings...");

    ini_reading = 100000/analogRead(pin_sensor_ini);
    end_reading = 100000/analogRead(pin_sensor_end);

    while(ini_reading > 200)
    {
        ini_reading = 100000/analogRead(pin_sensor_ini);
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
       
    measured_time = time_2 - time_1;
    measured_speed = 1000000/measured_time;

    Serial.println(measured_speed);
    
}
