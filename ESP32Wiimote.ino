#include "ESP32Wiimote.h"
#include <HardwareSerial.h> // library for serial and UART commnication

static const int UART_TX_PIN = 17; // Transmit (TX) ESP32
static const int UART_RX_PIN = 16; // Receive (RX) ESP32
static const uint32_t UART_BAUD = 115200; // Baud rate for both

#define UART_PORT Serial2

ESP32Wiimote wiimote;
static bool logging = true;
static long last_ms = 0;
static int num_run = 0, num_updates = 0;

static void send_wii_state(ButtonState buttons, const AccelState& accel, const NunchukState& n) {
    uint32_t b = static_cast<uint32_t>(buttons);
    const uint8_t header[2] = {0xAA, 0x55}; // Start bits
    uint8_t p[4] = {0}; // packet will be 12 bytes
    p[0] = uint8_t(b); // LSB 8 bits
    p[1] = uint8_t(b >> 8); // next 8 bits
    p[2] = uint8_t(b >> 16); // and so on
    p[3] = uint8_t(b >> 24);
    // p[4] = accel.xAxis; // acceleration components
    // p[5] = accel.yAxis; 
    // p[6] = accel.zAxis;
    // p[7] = n.xAxis; // nunchuck acceleration components
    // p[8] = n.yAxis;
    // p[9] = n.zAxis;
    // p[10] = n.xStick; // nunchuck analog values
    // p[11] = n.yStick;
    
    UART_PORT.write(header[0]); // send first header/ start bit
    UART_PORT.write(header[1]); // send second bit
    UART_PORT.write(p, sizeof(p)); // send packet
}

void setup()
{
    Serial.begin(115200);
    UART_PORT.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); // start UART2

    Serial.println("ESP32Wiimote");
    wiimote.init();
    if (!logging) wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);
    Serial.println("Started");
}

void loop()
{
    wiimote.task();
    wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL); 
    if (wiimote.available() > 0)
    {
        send_wii_state(
            wiimote.getButtonState(),      // no cast
            wiimote.getAccelState(),
            wiimote.getNunchukState()
        );
    }

    delay(10);
}
