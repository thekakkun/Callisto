#include "config_server.h"
#include "init.h"

void init_brightness()
{
    /* Define pulse wave modulation (PWM) output pin for boost converter and input
   * for light dependent resistor (LDR)
   * Max and min brightness is set from the LDR at this point so that the
   * brightness range is not zero during later calculations
   */

    constexpr int PWM_PIN{32};
    constexpr int PWM_FREQ{31250};
    constexpr int PWM_RESOLUTION{8};

    ledcSetup(BOOST_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_PIN, BOOST_CHANNEL);

    ledcSetup(BLANK_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(VFBLANK, BLANK_CHANNEL);
}

void touch_callback()
{
    /* placeholder callback for if I need it.
   */
}

void init_touch()
{
    /* Set touch pad pin and filter.
   * Callback needs to be set in order to use touch sensor to wake from deep
   * sleep
   */

    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_config(TOUCH_PAD_GPIO15_CHANNEL, TOUCH_THRESHOLD);
    touch_pad_filter_start(10);
    touchAttachInterrupt(TOUCH_PAD_GPIO15_CHANNEL, touch_callback,
                         TOUCH_THRESHOLD);
}

void init_spi()
{
    /* Initialize the serial peripheral interface (SPI) that will communicate
   * with the MAX9621 VFD driver chip.
   */

    constexpr int SPI_FREQ{500000};

    pinMode(VFLOAD, OUTPUT);
    digitalWrite(VFLOAD, LOW);
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
}

void init_spiffs()
{
    /* Initialize the SPIFFS, which is used to store the settings webpage data and
   * user preferences.
   */

    if (!SPIFFS.begin(true))
    {
        Serial.println("Error mounting SPIFFS");
    }
}

void init_wifi()
{
    /* Initialize the wifi. The variable WIFI_TIMEOUT will set how long before
   * wifi connection is deemed a failure.
   */

    constexpr unsigned int WIFI_TIMEOUT{10000};

    Serial.printf("Connecting to %s ", settings.ssid.c_str());
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname("Callisto");
    WiFi.begin(settings.ssid.c_str(), settings.password.c_str());

    unsigned long wifi_start{millis()};
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        unsigned long now_ms = millis();
        if (wifi_start + WIFI_TIMEOUT < now_ms)
        {
            break;
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        // TODO: Wifi sleep
        WiFi.setSleep(true);
        wifi_connected = true;
        Serial.println(" CONNECTED!");
        Serial.print("Local Address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.print("\n");
        Serial.println("Wifi timed out");
    }
}

void init_font_table()
{
    /* Create the font table and digit grid output based on which pins are being
   * used on the MAX6921 VFD driver chip.
   */

    constexpr int SEGMENT_OUT[]{
        3, // Segment A
        1, // Segment B
        6, // Segment C
        5, // Segment D
        7, // Segment E
        2, // Segment F
        0, // Segment G
        4, // Decimal point segment
    };
    constexpr int SEGMENTS[]{
        0x40, // -
        0x00, // .
        0x00, // /
        0x3F, // 0
        0x06, // 1
        0x5B, // 2
        0x4F, // 3
        0x66, // 4
        0x6D, // 5
        0x7D, // 6
        0x07, // 7
        0x7F, // 8
        0x6F, // 9
        0x00, // :
        0x00, // ;
        0x00, // <
        0x00, // =
        0x00, // >
        0x00, // ?
        0x00, // @
        0x77, // A
        0x7C, // B
        0x39, // C
        0x5E, // D
        0x79, // E
        0x71, // F
        0x3D, // G
        0x76, // H
        0x30, // I
        0x1E, // J
        0x00, // K
        0x38, // L
        0x00, // M
        0x54, // N
        0x3F, // O
        0x73, // P
        0x67, // Q
        0x50, // R
        0x6D, // S
        0x78, // T
        0x3E, // U
        0x00, // V
        0x00, // W
        0x00, // X
        0x6E, // Y
        0x00  // Z
    };
    int segment_size{sizeof(SEGMENTS) / sizeof(SEGMENTS[0])};
    for (int i{0}; i < segment_size; ++i)
    {
        for (int j{0}; j < 7; ++j)
        {
            if (bitRead(SEGMENTS[i], j))
            {
                bitSet(font_table[i], SEGMENT_OUT[j]);
            }
        }
    }

    dot = 1 << SEGMENT_OUT[7];

    constexpr int GRID_OUT[]{
        19, // Digit 9
        15, // Digit 8
        14, // Digit 7
        13, // Digit 6
        16, // Digit 5
        12, // Digit 4
        17, // Digit 3
        11, // Digit 2
        18, // Digit 1
    };
    int grid_size{sizeof(GRID_OUT) / sizeof(GRID_OUT[0])};
    for (int i{0}; i < grid_size; ++i)
    {
        bitSet(digit_table[i], GRID_OUT[i]);
    }
};

void init_sntp()
{
    /* Connect to the the simple network time protocol (SNTP) server, and retrieve
   * the current time. Also set the time zone based on user settings.
   */

    const char *const ntp_server[]{
        "pool.ntp.org",
        "0.pool.ntp.org",
        "1.pool.ntp.org",
        "2.pool.ntp.org",
        "3.pool.ntp.org",
    };

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    int server_count{sizeof(ntp_server) / sizeof(ntp_server[0])};
    for (int i{0}; i < server_count; ++i)
    {
        sntp_setservername(i, (char *)ntp_server[i]);
    }
    sntp_init();

    setenv("TZ", settings.time_zone.c_str(), 1);
    tzset();
}