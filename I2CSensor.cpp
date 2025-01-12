#include "I2CSensor.h"
#include "mbed.h"

// Accelerometer address
#define ACC_I2C_ADDRESS 0x1D << 1
#define ACC_CTRL_REG 0x2A
#define REG_OUT_X_MSB 0x01
#define REG_OUT_Y_MSB 0x03
#define REG_OUT_Z_MSB 0x05
#define UINT14_MAX 16383

// Color sensor definitions
#define TCS34725_ADDRESS 0x29 << 1  //register address for the rgb sensor
#define TCS34725_ENABLE 0x00        //register address to enable the rgb sensor
#define TCS34725_ENABLE_PON 0x01
#define TCS34725_ENABLE_AEN 0x02
#define TCS34725_ATIME 0x01
#define TCS34725_CONTROL 0x0F
#define TCS34725_COMMAND_BIT 0x80
#define TCS34725_CDATAL 0x14        //register for clear data
#define TCS34725_RDATAL 0x16        //register for red data
#define TCS34725_GDATAL 0x18        //register for green data
#define TCS34725_BDATAL 0x1A        //register for blue data
#define MAX_COLOR_VALUE 65535

// Temperature and Humidity sensor definitions
#define SI7021_ADDRESS 0x40 << 1
#define CMD_MEASURE_HUMIDITY 0xF5
#define CMD_MEASURE_TEMPERATURE 0xF3
#define THRESHOLD 1.0

I2CSensor::I2CSensor(PinName sda, PinName scl, PinName digitalPin) : i2c(sda, scl), light(digitalPin) {}

// --- Accelerometer Functions ---
bool I2CSensor::initializeAccelerometer() {
    uint8_t data[2] = {ACC_CTRL_REG, 0x01};
    return i2c.write(ACC_I2C_ADDRESS, (char *)data, 2) == 0;
}

bool I2CSensor::readAccRegs(int addr, uint8_t *data, int len) {
    char t[1] = {(char) addr};
    if (i2c.write(ACC_I2C_ADDRESS, t, 1, true) != 0)
        return false;
    return i2c.read(ACC_I2C_ADDRESS, (char *) data, len) == 0;
}

float I2CSensor::getAccAxis(uint8_t addr) {
    int16_t axisValue;
    uint8_t res[2];
    if (!readAccRegs(addr, res, 2)){
        return 0.0;
    }
    axisValue = (res[0] << 6) | (res[1] >> 2);
    if (axisValue > UINT14_MAX/2)
        axisValue -= UINT14_MAX;
    return float(axisValue) / 4096.0;
}

// --- Color Sensor Functions ---
void I2CSensor::initializeColorSensor() {
    if (writeColorRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON) < 0){
        return;
    }
    ThisThread::sleep_for(3ms);
    if (writeColorRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) < 0 || 
    writeColorRegister(TCS34725_ATIME, 0xFF) < 0 || writeColorRegister(TCS34725_CONTROL, 0x01) < 0){
        return;
    }
}

int I2CSensor::writeColorRegister(uint8_t reg, uint8_t value) {
    char data[2] = {(char)(TCS34725_COMMAND_BIT | reg), (char)value}; 
    return i2c.write(TCS34725_ADDRESS, data, 2);
}

int I2CSensor::readColorRegister(uint8_t reg) {
    char cmd = (TCS34725_COMMAND_BIT | reg);
    char data[2];
    if (i2c.write(TCS34725_ADDRESS, &cmd, 1) != 0)
        return -1;
    if (i2c.read(TCS34725_ADDRESS, data, 2) != 0)
        return -1;
    return (data[1] << 8) | data[0];
}

void I2CSensor::readColorData(uint16_t &clear, uint16_t &red, uint16_t &green, uint16_t &blue) {
   
    clear = readColorRegister(TCS34725_CDATAL);
    red = readColorRegister(TCS34725_RDATAL);
    green = readColorRegister(TCS34725_GDATAL);
    blue = readColorRegister(TCS34725_BDATAL);

}

// --- Temperature and Humidity Sensor Functions ---
float I2CSensor::readHumidity() {
    char cmd[1] = { CMD_MEASURE_HUMIDITY };
    char data[2] = { 0 };
    if (i2c.write(SI7021_ADDRESS, cmd, 1) != 0){
        return 0.0;
    }
    ThisThread::sleep_for(20ms);
    if (i2c.read(SI7021_ADDRESS, data, 2) != 0){
        return 0.0;
    }
    int humidity_raw = (data[0] << 8) | data[1];
    return ((125.0 * humidity_raw) / 65536) - 6.0;
}

float I2CSensor::readTemperature() {
    char cmd[1] = { CMD_MEASURE_TEMPERATURE };
    char data[2] = { 0 };

    if (i2c.write(SI7021_ADDRESS, cmd, 1) != 0) return 0.0;
    ThisThread::sleep_for(20ms);
    if (i2c.read(SI7021_ADDRESS, data, 2) != 0) return 0.0;
    
    int temperature_raw = (data[0] << 8) | data[1];
    return temperature_raw;
}

int I2CSensor::dominant(int red, int green, int blue){
    int max = red, color = 1;
    if (green > max) {
        max = green;
        color = 2;
    }
    if (blue > max) {
        max = blue;
        color = 3;
    }
    return color;
}

float I2CSensor::calculateTotalAcceleration(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}

void I2CSensor::read_i2c() {
    
        initializeAccelerometer();

        ax = getAccAxis(REG_OUT_X_MSB);
        ay = getAccAxis(REG_OUT_Y_MSB);
        az = getAccAxis(REG_OUT_Z_MSB);

        // Read color sensor values
        initializeColorSensor();

        readColorData(clear, red, green, blue);
    
        // Read temperature and humidity
        humidity = readHumidity();
        temperature = readTemperature();

}