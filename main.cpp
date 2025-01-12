#include "I2CSensor.h"
#include "mbed.h"
#include <cstdio>
#include <cstdint>
#include <cstdio>

#include "mbed_version.h"

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"
#include "GPS.h"
#include "I2CSensor.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;
using namespace std::chrono_literals;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.

uint8_t rx_buffer[30];
#define TX_BUFFER_SIZE 30


/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10s

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Default and configured device EUI, application EUI and application key
 */
static const uint8_t DEFAULT_DEV_EUI[] = {0x40, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t DEV_EUI[] = {0x82, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = {0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0x4d};
static uint8_t APP_KEY[] = {0xf3, 0x1c, 0x2e, 0x8b, 0xc6, 0x71, 0x28, 0x1d,
                            0x51, 0x16, 0xf0, 0x8f, 0xf0, 0xb7, 0x92, 0x8f};

static uint8_t OFF_COMMAND[] = {0x4f, 0x46, 0x46};
static uint8_t GREEN_COMMAND[] = {0x47, 0x72, 0x65, 0x65, 0x6e};
static uint8_t RED_COMMAND[] = {0x52, 0x65, 0x64};

#define WHITE 0     //0,0,0
#define YELLOW 1    //0,0,1
#define PURPLE 2    //0,1,0
#define RED 3       //0,1,1
#define CYAN 4      //1,0,0
#define GREEN 5     //1,0,1
#define BLUE 6      //1,1,0
#define OFF 7       //1,1,1

#define GPS_TX_PIN PA_9
#define GPS_RX_PIN PA_10
#define I2C_SDA PB_9
#define I2C_SCL PB_8
#define LIGHT_RGB_PIN PB_12
#define LIGHT_PIN PA_0
#define MOISTURE_PIN PA_4

BufferedSerial pc(USBTX, USBRX, 115200);
AnalogIn light(LIGHT_PIN), moisture(MOISTURE_PIN);
BusOut led_rgb(PB_13, PB_14, PB_15);
uint8_t tx_buffer[TX_BUFFER_SIZE];
Thread gpsThread(osPriorityNormal, 1024);

GPS gps(GPS_TX_PIN, GPS_RX_PIN, 9600);
I2CSensor i2c(I2C_SDA, I2C_SCL, PB_12);



struct __attribute__((packed)) frame_data_t {
 uint8_t light;   //1
 uint8_t moisture;   //1
 uint8_t accelleration;  //1
 uint8_t humidity;  //1
 uint16_t ax_value;  //2
 uint16_t ay_value;  //2
 uint16_t az_value;  //2
 uint16_t red;  //2
 uint16_t green;  //2
 uint16_t blue;  //2
 uint16_t clear; //2
 uint16_t temperature;  //2
 uint16_t altitude; //2
 float latitude; //4
 float longitude;   //4
} frame_data_t;

struct frame_data_t tx_struct_buffer;

float calculateTotalAcceleration(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}

void read_sensor(){
    gpsThread.flags_set(0x1);

    tx_struct_buffer.light = static_cast<uint8_t>(light.read()*100);
    tx_struct_buffer.moisture = static_cast<uint8_t>(moisture.read()*100);

    i2c.read_i2c();
    
    float x_axis = (i2c.ax * 9.81  + 200) * 100;
    float y_axis = (i2c.ay * 9.81 + 200) * 100;
    float z_axis = (i2c.az * 9.81 + 200) * 100;

    tx_struct_buffer.ax_value = static_cast<uint16_t>(x_axis);
    tx_struct_buffer.ay_value = static_cast<uint16_t>(y_axis);
    tx_struct_buffer.az_value = static_cast<uint16_t>(z_axis);
    tx_struct_buffer.accelleration = static_cast<uint8_t>(calculateTotalAcceleration(i2c.ax, i2c.ay, i2c.az));

    tx_struct_buffer.red = static_cast<uint16_t>(i2c.red);
    tx_struct_buffer.green = static_cast<uint16_t>(i2c.green);
    tx_struct_buffer.blue = static_cast<uint16_t>(i2c.blue);
    tx_struct_buffer.clear = static_cast<uint16_t>(i2c.clear);

    tx_struct_buffer.humidity = static_cast<uint8_t>(i2c.humidity);
    tx_struct_buffer.temperature = static_cast<uint16_t>(i2c.temperature);

    tx_struct_buffer.latitude = gps.latitude;
    tx_struct_buffer.longitude = gps.longitude;
    tx_struct_buffer.altitude = static_cast<uint16_t>(gps.latitude);

    printf("\nLight: %d\n", tx_struct_buffer.light);
    printf("Temperature: %d\n", tx_struct_buffer.temperature);
    printf("Humidity: %d\n", tx_struct_buffer.humidity);
    printf("Moisture: %d\n", tx_struct_buffer.moisture);
    printf("Red: %d\tGreen: %d\tBlue: %d\tClear: %d\t\n", tx_struct_buffer.red, tx_struct_buffer.green, tx_struct_buffer.blue, tx_struct_buffer.clear);
    printf("X_axis: %d\tY_axis: %d\tZ_axis: %d\tTotal: %d\t\n", tx_struct_buffer.ax_value, tx_struct_buffer.ay_value, tx_struct_buffer.az_value, tx_struct_buffer.accelleration);
    printf("Latitude: %.3f\tLongitude: %.3f\tAltitude: %d\t\n", tx_struct_buffer.latitude, tx_struct_buffer.longitude, tx_struct_buffer.altitude);
}

/**
 * Sends a message to the Network Server
 */
static void send_message(){
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;

    read_sensor();

    memcpy(tx_buffer, &tx_struct_buffer, sizeof(tx_struct_buffer));

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, sizeof(tx_struct_buffer),
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n") : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message(){
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }

    if (!memcmp(OFF_COMMAND, rx_buffer, sizeof(OFF_COMMAND))) led_rgb = OFF;
    if (!memcmp(RED_COMMAND, rx_buffer, sizeof(RED_COMMAND))) led_rgb = RED;
    if (!memcmp(GREEN_COMMAND, rx_buffer, sizeof(GREEN_COMMAND))) led_rgb = GREEN;

    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}


int main(){

    // setup tracing
    setup_trace();

    led_rgb = OFF;

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;
    gpsThread.start(callback(&gps, &GPS::read_gps));

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER) != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n", CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data rate (ADR) - Enabled \r\n");

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;

    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event) {
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}
