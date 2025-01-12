#include <cstring>
#include <cstdlib>
#include <cstring>  
#include "mbed.h"
#include "GPS.h"

#define DEFAULT_LATITUDE 20.9076f
#define DEFAULT_LONGITUDE 12.4861f
#define DEFAULT_ALTITUDE 21.0f

GPS::GPS(PinName txPin, PinName rxPin, int baudRateGPS) : gpsSerial(txPin, rxPin, baudRateGPS){
    num_satellites = 0;
    latitude = 0.0;
    longitude = 0.0;
    meridian = ' ';
    parallel = ' ';
    altitude = 0.0;
    measurement = ' ';
    memset(gps_time, 0, sizeof(gps_time));
}


void GPS::parse_gps_data(char* nmea_sentence) {
    if (strstr(nmea_sentence, "$GPGGA")) {          // Find GPGGA string
        char* token = strtok(nmea_sentence, ",");   // Split the string by commas
        int fieldIndex = 0;
        int hour, minute, second;
        while (token != NULL) {
            switch (fieldIndex) {
                case 1:
                    if (token) {
                        hour = (token[0] - '0') * 10 + (token[1] - '0') + 1;
                        if (hour >= 24) 
                            hour -= 24;
                        minute = (token[2] - '0') * 10 + (token[3] - '0');
                        second = (token[4] - '0') * 10 + (token[5] - '0');
                        snprintf(gps_time, sizeof(gps_time), "%.2d:%.2d:%.2d", hour, minute, second);
                    } else snprintf(gps_time, sizeof(gps_time), "Time not available!");
                    break;

                case 2: latitude = atof(token) / 100; break;        // Latitude
                case 3: parallel = token[0]; break;                 // N/S Indicator
                case 4: longitude = atof(token) / 100; break;       // Longitude
                case 5: meridian = token[0]; break;                 // E/W Indicator
                case 7: num_satellites = atoi(token); break;        // Number of Satellites
                case 9: altitude = atof(token); break;              // Altitude
                case 10: measurement = token[0] + 32; break;        // Measurement Unit (converted to lowercase)
            }
            token = strtok(NULL, ",");
            fieldIndex++;
        }
        const char par = parallel;
        const char mer = meridian;
        if (num_satellites == 0){
            latitude = DEFAULT_LATITUDE;
            longitude = DEFAULT_LONGITUDE;
            altitude = DEFAULT_ALTITUDE;
        } else{
            if (par == 'S') latitude = -latitude;
            if (mer == 'W') longitude = -longitude;
        } 
    }
}

void GPS::read_gps(void) {
    while(true){
        uint32_t flags = ThisThread::flags_wait_any(0x1, true);
        if (gpsSerial.readable()) {
            int bytesRead = gpsSerial.read(buffer, sizeof(buffer)); 
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';
                parse_gps_data(buffer);    
            }
        } else {
            latitude = DEFAULT_LATITUDE;
            longitude = DEFAULT_LONGITUDE;
            altitude = DEFAULT_ALTITUDE;
        }
    }
}