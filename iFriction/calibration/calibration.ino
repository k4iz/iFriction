int pin_sensor_ini = 1;
int pin_sensor_end = 2;

float ini_reading;
float end_reading;

void setup()
{
    Serial.begin(9600);
    pinMode (pin_sensor_ini, INPUT);
    pinMode (pin_sensor_end, INPUT);
}

void loop()
{
    ini_reading = 100000/analogRead(pin_sensor_ini);
    end_reading = 100000/analogRead(pin_sensor_end);

    Serial.println("First Sensor");
    Serial.println(ini_reading);

    Serial.println("Second Sensor");
    Serial.println(end_reading);

    delay (1000);
}
