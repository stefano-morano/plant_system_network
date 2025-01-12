#ifndef I2CSENSORS_H
#define I2CSENSORS_H

#include "mbed.h"
#include <cstdint>

class I2CSensor {
public:

    DigitalOut light;
    uint16_t clear, red, green, blue;
    float ax, ay, az, humidity, temperature;
    bool color_alarm = false, temperature_alarm = false, humidity_alarm = false, accellerometer_alarm = false, 
    color_working = false, temperature_working = false, humidity_working = false, accellerometer_working = false, 
    need_flash = false;
    
    I2CSensor(PinName sda, PinName scl, PinName digitalPin);    
    void read_i2c();
    void changeLED(int mode);

private:

    I2C i2c;
    float total_acc = 0;

    bool readAccRegs(int addr, uint8_t *data, int len);
    float getAccAxis(uint8_t addr);
    int writeColorRegister(uint8_t reg, uint8_t value);
    
    //RGB sensor functions
    int readColorRegister(uint8_t reg);
    int dominant(int red, int green, int blue);
    float readHumidity();
    float readTemperature();

    // Color sensor functions
    void initializeColorSensor();
    void readColorData(uint16_t &clear, uint16_t &red, uint16_t &green, uint16_t &blue);

     // Accelerometer functions
    bool initializeAccelerometer();
    float calculateTotalAcceleration(float x, float y, float z);

};

#endif
