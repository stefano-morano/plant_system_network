// GPS.h
#ifndef GPS_H
#define GPS_H

#include "mbed.h"
#include "rtos.h"

class GPS {
public:
 
    BufferedSerial gpsSerial;

    int num_satellites;
    float latitude, longitude, altitude;
    char measurement, gps_time[10], meridian, parallel;
    char gps_data[256], buffer[256];

    GPS(PinName txPin, PinName rxPin, int baudRateGPS);
    void read_gps(void);

private: 
    void parse_gps_data(char* nmea_sentence);
};

#endif // GPS_H
