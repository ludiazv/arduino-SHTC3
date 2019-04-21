# A minimial arduino library fro SHTC3 Temperature & Humidity sensor

A bare minimal lib for this sensor for low power applications based on official [SHT libraries](https://github.com/Sensirion/arduino-sht).

## Usage
 The library is designed for low power applications that will read sensor periodically. When the sensor is not reading
 it is keep spleep consuming less than uA.


```cplusplus

 
    SHTC3 sensor(Wire);

    void setup() {
            Wire.begin();
            sensor.begin();
    }
    void loop() {
            s.sample();
            Serial.print(F("[SHTC3] T:"));
            Serial.print(s.readTempC());
            Serial.print(F(" Cº  /   H: "));
            Serial.print(s.readHumidity());
            Serial.println(F(" %"));
            delay(2000);
    }


```

## Compilation options:
 if __ARDUINO_SHTC3_NOFLOAT__ is defined no float computation is done. Temperature and Humidity will return a 16bit integer. This optimization will reduce the code size more at expense of precision in the reading.


