
#include "config_server.h"
#include "display.h"

void adjust_brightness(int min_brightness, int max_brightness)
{
    /* Set the anode voltage based on ambient brightness.
   * Target voltages used in the calculation do not reflect actual boost
   * converter outputs for whatever reason...
   */

    // TODO: Use PWM blanking instead of boost converter voltage?
    constexpr int V_IN{9}, MIN_V{15}, MAX_V{40}, V_LIM{55};
    const int V_RANGE{MAX_V - MIN_V};
    constexpr float ETA{0.8};
    float lo_brightness_v, hi_brightness_v, brightness_v_range;
    float ambient_brightness_p, v_out;
    int duty;

    // max and min output voltage, based on user set preferences
    lo_brightness_v =
        MIN_V + (V_RANGE * static_cast<float>(settings.lo_brightness - 1) / 9);
    hi_brightness_v =
        MIN_V + (V_RANGE * static_cast<float>(settings.hi_brightness - 1) / 9);
    brightness_v_range = hi_brightness_v - lo_brightness_v;

    int brightness{analogRead(LDR_PIN)};
    ambient_brightness_p =
        static_cast<float>(brightness - min_brightness) / (max_brightness - min_brightness);
    v_out = lo_brightness_v + ambient_brightness_p * brightness_v_range;
    duty = int(255 * (V_IN * ETA / (V_LIM - v_out)));

    ledcWrite(PWM_CHANNEL, duty);
}

void set_touch_state(bool &touch_state, bool &previous_touch_state,
                     unsigned long &touch_start, unsigned long &touch_end)
{
    /* Figure out when touch starts and ends, and set the time in ms.
   */

    uint16_t touch_value;
    unsigned long now_ms{millis()};

    touch_pad_read_filtered(TOUCH_PAD_NUM3, &touch_value);

    if (touch_value < TOUCH_THRESHOLD)
    {
        touch_state = true;
        digitalWrite(LED_PIN, HIGH); // FIXME: Delete later
        if (!previous_touch_state)
        {
            touch_start = now_ms;
            Serial.println("Touch start");
        }
    }
    else
    {
        touch_state = false;
        digitalWrite(LED_PIN, LOW); // FIXME: Delete later
        if (previous_touch_state)
        {
            touch_end = now_ms;
            Serial.println("Touch end");
        }
    }
}

bool is_night()
{
    /* Figure out if night mode should be active
   */

    time_t now{time(0)};
    struct tm timeinfo = *localtime(&now);

    int current_time;
    static int night_start, night_end;

    current_time = timeinfo.tm_hour * 100 + timeinfo.tm_min;
    night_start =
        settings.night_start_h * 100 + settings.night_start_m;
    night_end = settings.night_end_h * 100 + settings.night_end_m;

    if (night_start < night_end)
    {
        if (night_start <= current_time && current_time < night_end)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (night_start <= current_time || current_time < night_end)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

int get_mode()
{
    /* Get display mode based on success flags, current time, and touch reading
   * 0: Time (everything okay)
   * 1: Sleep (at night)
   * 2: Date (touch detected)
   * 3: IP address (server is active)
   * 4: NTP syncing (wifi connected, getting time)
   * 5: Wifi error (wifi config exists, but timed out)
   * 6: AP active (AP is active)
   */

    constexpr unsigned int SHOW_FOR{5 * 1000};
    constexpr unsigned int SERVER_START{10 * 1000};
    constexpr unsigned int SERVER_TIMEOUT{10 * 60 * 1000};

    unsigned long now_ms;
    static bool touch_state{false};
    bool previous_touch_state{touch_state};
    static unsigned long touch_start{0}, touch_end{0};
    static unsigned long server_start_time;

    if (!wifi_connected)
    {
        if (credentials_saved)
        {
            return 5; // credentials saved, but timed out -> wifi error
        }
        else
        {
            return 6; // no credentials -> connect to ap
        }
    }
    else if (server_active)
    {
        now_ms = millis();
        if (now_ms - server_start_time < SERVER_TIMEOUT)
        {
            return 3; // server active, not timed out -> show IP
        }
        else
        {
            return 0; // server timed out -> show time
        }
    }
    else
    {
        set_touch_state(touch_state, previous_touch_state, touch_start, touch_end);

        time_t now = time(0);
        struct tm timeinfo = *localtime(&now);

        now_ms = millis();
        if (touch_state && now_ms - touch_start > SERVER_START)
        {
            server_start_time = millis();
            return 3; // held for long time -> start server
        }
        else if (!time_synced)
        {
            if (timeinfo.tm_year > 70)
            {
                time_synced = true;
            }
            return 4; // time syncing -> show boot
        }
        else if (is_night())
        {
            if (now_ms - touch_end < SHOW_FOR + 5000)
            {
                // touched at night -> show time for longer
                // (to accomodate for waking from deep sleep)
                return 0;
            }
            else
            {
                return 1; // night -> clock off
            }
        }
        else
        {
            if (now_ms - touch_end < SHOW_FOR)
            {
                return 2; // touched during day -> show date
            }
            else
            {
                return 0; // day -> show time
            }
        }
    }
}

void set_text(int disp_mode, char *disp_text)
{
    /* Set display data based on display mode
   */

    switch (disp_mode)
    {
        time_t now;
        struct tm timeinfo;
        int current_time, night_end, deep_sleep_sec;

    case 0: // Time
        digitalWrite(VFBLANK, LOW);
        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_DEFAULT);
        esp_sleep_enable_touchpad_wakeup();

        if (server_active)
        {
            server.end();
            server_active = false;
        }

        now = time(0);
        timeinfo = *localtime(&now);

        if (settings.t_format == 0 || settings.t_format == 1)
        {
            if (settings.t_pad == 0)
            {
                if (settings.t_divider == 0)
                {
                    strftime(disp_text, 10, " %l %M %S", &timeinfo);
                }
                else
                { // t_divider == 1
                    strftime(disp_text, 10, " %l-%M-%S", &timeinfo);
                }
            }
            else
            { // t_pad == 1
                if (settings.t_divider == 0)
                {
                    strftime(disp_text, 10, " %I %M %S", &timeinfo);
                }
                else
                { // t_divider == 1
                    strftime(disp_text, 10, " %I-%M-%S", &timeinfo);
                }
            }
        }
        else
        { // t_format == 2
            if (settings.t_pad == 0)
            {
                if (settings.t_divider == 0)
                {
                    strftime(disp_text, 10, " %k %M %S", &timeinfo);
                }
                else
                { // t_divider == 1
                    strftime(disp_text, 10, " %k-%M-%S", &timeinfo);
                }
            }
            else
            { // t_pad == 1
                if (settings.t_divider == 0)
                {
                    strftime(disp_text, 10, " %H %M %S", &timeinfo);
                }
                else
                { // t_divider == 1
                    strftime(disp_text, 10, " %H-%M-%S", &timeinfo);
                }
            }
        }
        break;

    case 1:
    { // off
        digitalWrite(VFBLANK, HIGH);

        now = time(0);
        timeinfo = *localtime(&now);

        current_time, night_end, deep_sleep_sec;
        current_time = timeinfo.tm_hour * 100 + timeinfo.tm_min;
        night_end = settings.night_end_h * 100 + settings.night_end_m;

        if (current_time < night_end)
        {
            deep_sleep_sec = (settings.night_end_h - timeinfo.tm_hour) * 60 * 60 +
                             (settings.night_end_m - timeinfo.tm_min) * 60 +
                             (0 - timeinfo.tm_sec);
        }
        else
        {
            deep_sleep_sec =
                (settings.night_end_h - timeinfo.tm_hour + 24) * 60 * 60 +
                (settings.night_end_m - timeinfo.tm_min) * 60 +
                (0 - timeinfo.tm_sec);
        }

        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
        esp_sleep_enable_touchpad_wakeup();
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        esp_deep_sleep(deep_sleep_sec * 1e6);

        break;
    }

    case 2: // date
        digitalWrite(VFBLANK, LOW);

        now = time(0);
        timeinfo = *localtime(&now);

        if (settings.d_format == 0)
        {
            if (settings.d_pad == 0)
            {
                if (settings.d_divider == 0)
                {
                    strftime(disp_text, 10, " %m%e%Y", &timeinfo);
                }
                else if (settings.d_divider == 1)
                {
                    strftime(disp_text, 10, " %m-%e-%y", &timeinfo);
                }
                else
                { // d_divider == 2
                    strftime(disp_text, 10, " %m %e %y", &timeinfo);
                }
                if (disp_text[1] == '0')
                {
                    disp_text[1] = ' ';
                }
            }
            else
            { // d_pad == 1
                if (settings.d_divider == 0)
                {
                    strftime(disp_text, 10, " %m%d%Y", &timeinfo);
                }
                else if (settings.d_divider == 1)
                {
                    strftime(disp_text, 10, " %m-%d-%y", &timeinfo);
                }
                else
                { // d_divider == 2
                    strftime(disp_text, 10, " %m %d %y", &timeinfo);
                }
            }
        }
        else
        { // d_format == 1
            if (settings.d_pad == 0)
            {
                if (settings.d_divider == 0)
                {
                    strftime(disp_text, 10, " %e%m%Y", &timeinfo);
                    if (disp_text[3] == '0')
                    {
                        disp_text[3] = ' ';
                    }
                }
                else if (settings.d_divider == 1)
                {
                    strftime(disp_text, 10, " %e-%m-%y", &timeinfo);
                    if (disp_text[4] == '0')
                    {
                        disp_text[4] = ' ';
                    }
                }
                else
                { // d_divider == 2
                    strftime(disp_text, 10, " %e %m %y", &timeinfo);
                    if (disp_text[4] == '0')
                    {
                        disp_text[4] = ' ';
                    }
                }
            }
            else
            {
                if (settings.d_divider == 0)
                {
                    strftime(disp_text, 10, " %d%m%Y", &timeinfo);
                }
                else if (settings.d_divider == 1)
                {
                    strftime(disp_text, 10, " %d-%m-%y", &timeinfo);
                }
                else
                { // d_divider == 2
                    strftime(disp_text, 10, " %d %m %y", &timeinfo);
                }
            }
        }

        break;

    case 3: // IP
    {
        digitalWrite(VFBLANK, LOW);
        if (!server_active)
        {
            init_server();
        }
        sprintf(disp_text, "%9d", WiFi.localIP()[3]);
        break;
    }

    case 4: // NTP syncing
        digitalWrite(VFBLANK, LOW);
        sprintf(disp_text, " %-8s", "Callisto");
        break;

    case 5: // Wifi error
        digitalWrite(VFBLANK, LOW);
        sprintf(disp_text, " %-8s", "net err");

        if (!ap_active)
        {
            ssid_options = get_ssids();
            init_ap();
        }
        dns_server.processNextRequest();
        break;

    case 6: // AP active
        digitalWrite(VFBLANK, LOW);
        sprintf(disp_text, " %-8s", "connect");

        if (!ap_active)
        {
            ssid_options = get_ssids();
            init_ap();
        }
        dns_server.processNextRequest();
        break;
    }
}

void dot_scroll(int *dots)
{
    /* Generate array for scrolling dots
   */

    unsigned long now_ms{millis()};

    dots[(now_ms / 500) % 8 + 1] = 1;
}

void set_dots(int disp_mode, int *dots)
{
    /* Set decimal point display based on display mode and settings.
   */

    switch (disp_mode)
    {
        time_t now;
        struct tm timeinfo;

        char am_pm[3];

    case 0: // Time
        if (settings.t_format == 0)
        {
            now = time(0);
            timeinfo = *localtime(&now);
            strftime(am_pm, 3, "%p", &timeinfo);

            if (strcmp(am_pm, "PM") == 0)
            {
                dots[0] = 1;
            }
        }
        break;

    case 1: // off
        break;

    case 2: // date
        if (settings.d_divider == 0)
        {
            dots[2] = 1;
            dots[4] = 1;
        }
        break;

    case 3: // IP
        dots[5] = 0;
        break;

    case 4: // ntp
        dot_scroll(dots);
        break;

    case 5: // no wifi
        break;
    }
}

int get_spi_data(int current_digit, char *disp_text, int *dots)
{
    /* Based on character string and current digit, generate data to be sent via
   * SPI
   */

    unsigned char chr{disp_text[current_digit]};
    int spi_data;

    if (chr <= (unsigned char)' ') // If character is space, show dot or nothing at all
    {
        spi_data = dots[current_digit] * (dot + digit_table[current_digit]);
        return spi_data;
    }

    else // Else, retrieve data based on font table
    {
        if (chr > (unsigned char)'Z') // If lower case, convert to upper case
        {
            chr -= 32;
        }
        if (chr > (unsigned char)'Z') // If still after 'Z', return -1
        {
            return 0;
        }
    }
    spi_data = font_table[chr - '-'];
    spi_data += dots[current_digit] * dot;
    spi_data += digit_table[current_digit];

    return spi_data;
}

void send_spi_data(int spi_data)
{
    /* Send data via SPI
   */

    digitalWrite(VFLOAD, LOW);
    for (int i{0}; i < 3; ++i)
    {
        int tube_out = (unsigned int)spi_data >> (8 * (2 - i)) & 0xff;
        SPI.transfer(tube_out);
    }
    digitalWrite(VFLOAD, HIGH);
}

void brightness_utilities(int *min_brightness, int *max_brightness)
{
    /* Various utilities regarding brightness
   * Set min and max brightness, based on what was seen
   * Shrink brightness range every 2 days, in order to account for brightness
   * outliers.
   */

    int brightness{analogRead(LDR_PIN)};

    if (*max_brightness < brightness)
    {
        *max_brightness = brightness;
    }
    if (brightness < *min_brightness)
    {
        *min_brightness = brightness;
    }

    // every 2 days
    constexpr unsigned int UTILITY_FREQUENCY{1000 * 60 * 60 * 24 * 2};
    constexpr float DAMP_P{.02};
    static unsigned long now_ms = millis();
    unsigned long last_run = 0;

    if (now_ms - last_run >= UTILITY_FREQUENCY)
    {
        last_run = now_ms;
        int brightness_mean = (*max_brightness + *min_brightness) / 2;
        *max_brightness = max(int(*max_brightness * (1 - DAMP_P)), brightness_mean);
        *min_brightness = min(int(*max_brightness * (1 + DAMP_P)), brightness_mean);
    }
}