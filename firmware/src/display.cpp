#include "config_server.h"
#include "display.h"

void adjust_brightness(int min_brightness, int max_brightness)
{
    /* Set the anode voltage based on ambient brightness.
   * Target voltages used in the calculation do not reflect actual boost
   * converter outputs for whatever reason...
   */

    // Gives max tube voltage of 48-ish volts. Increase value for higher voltage at risk of decreased tube life.
    constexpr int BOOST_DUTY{160};
    constexpr int PWM_LIM_HI{0}, PWM_LIM_LO{240}; // Absolute limits for PWM.
    const int PWM_LIM_RANGE{PWM_LIM_LO - PWM_LIM_HI};
    float pwm_hi, pwm_lo;
    float brightness_p{};
    int blank_duty;

    // PWM limit based on user-set brightness range.
    pwm_hi = PWM_LIM_HI + (PWM_LIM_RANGE * static_cast<float>(settings.lo_brightness - 1) / 9);
    pwm_lo = PWM_LIM_HI + (PWM_LIM_RANGE * static_cast<float>(settings.hi_brightness - 1) / 9);

    int brightness{analogRead(LDR_PIN)};
    int brightness_avg{ldr_reading.reading(brightness)};
    brightness_p =
        static_cast<float>(brightness_avg - min_brightness) / (max_brightness - min_brightness);
    blank_duty = static_cast<int>(pwm_hi + (1 - brightness_p) * (pwm_lo - pwm_hi));

    ledcWrite(BOOST_CHANNEL, BOOST_DUTY);
    ledcWrite(BLANK_CHANNEL, blank_duty);
}

void set_touch_state(bool &touch_state, bool &previous_touch_state,
                     unsigned long &touch_start, unsigned long &touch_end)
{
    /* Figure out when touch starts and ends, and set the time in ms.
   */

    uint16_t touch_value{};
    unsigned long now_ms{millis()};

    touch_pad_read_filtered(TOUCH_PAD_GPIO15_CHANNEL, &touch_value);

    if (touch_value < TOUCH_THRESHOLD)
    {
        touch_state = true;

        if (!previous_touch_state)
        {
            touch_start = now_ms;
            Serial.println("Touch start");
        }
    }
    else
    {
        touch_state = false;

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

    int current_time_digital{};
    static int night_start_digital{}, night_end_digital{};

    current_time_digital = timeinfo.tm_hour * 100 + timeinfo.tm_min;
    night_start_digital =
        settings.night_start_h * 100 + settings.night_start_m;
    night_end_digital = settings.night_end_h * 100 + settings.night_end_m;

    if (night_start_digital == night_end_digital)
    {
        return false;
    }
    else if (night_start_digital < night_end_digital)
    {
        if (night_start_digital <= current_time_digital && current_time_digital < night_end_digital)
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
        if (night_start_digital <= current_time_digital || current_time_digital < night_end_digital)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

void go_to_sleep()
{
    time_t now;
    struct tm timeinfo;
    int time_digital, night_end_digital, deep_sleep_sec;

    Serial.println("entering sleep");

    now = time(0);
    timeinfo = *localtime(&now);

    time_digital = timeinfo.tm_hour * 100 + timeinfo.tm_min;
    night_end_digital = settings.night_end_h * 100 + settings.night_end_m;

    if (time_digital < night_end_digital)
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

    esp_deep_sleep(deep_sleep_sec * 1e6);
}

Mode get_mode()
{
    /* Get display mode based on success flags, current time, and touch reading
   */

    constexpr unsigned int SHOW_FOR{5 * 1000};
    constexpr unsigned int SERVER_START{10 * 1000};
    constexpr unsigned int SERVER_TIMEOUT{10 * 60 * 1000};

    unsigned long now_ms{};
    static bool touch_state{false};
    bool previous_touch_state{touch_state};
    static unsigned long touch_start{0}, touch_end{0};
    static unsigned long server_start_time{};

    if (!wifi_connected)
    {
        if (credentials_saved)
        {
            return wifi_error; // credentials saved, but timed out -> wifi error
        }
        else
        {
            return connect_ap; // no credentials -> connect to ap
        }
    }
    else if (server_active)
    {
        now_ms = millis();
        if (now_ms - server_start_time < SERVER_TIMEOUT)
        {
            return ip_address; // server active, not timed out -> show IP
        }
        else
        {
            return current_time; // server timed out -> show time
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
            return ip_address; // held for long time -> start server
        }
        else if (!time_synced)
        {
            if (timeinfo.tm_year > 70)
            {
                time_synced = true;
            }
            return ntp_syncing; // time syncing -> show boot
        }
        else if (is_night())
        {
            if (now_ms - touch_wake_time < SHOW_FOR)
            {
                // touched at night -> show time
                return current_time;
            }
            else
            {
                return in_sleep; // night -> clock off
            }
        }
        else
        {
            if (now_ms - touch_start < SHOW_FOR)
            {
                return current_date; // touched during day -> show date
            }
            else
            {
                return current_time; // day -> show time
            }
        }
    }
}

void set_text(Mode disp_mode, char *disp_text)
{
    /* Set display data based on display mode
   */

    switch (disp_mode)
    {
        time_t now;
        struct tm timeinfo;

    case current_time:
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

    case in_sleep:
        ledcWrite(BLANK_CHANNEL, 255);
        go_to_sleep();
        break;

    case current_date:
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

    case ip_address:
        if (!server_active)
        {
            init_server();
        }
        sprintf(disp_text, "%9d", WiFi.localIP()[3]);
        break;

    case ntp_syncing:
        sprintf(disp_text, " %-8s", "Callisto");
        break;

    case wifi_error:
        sprintf(disp_text, " %-8s", "net err");

        if (!ap_active)
        {
            ssid_options = get_ssids();
            init_ap();
        }
        dns_server.processNextRequest();
        break;

    case connect_ap:
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

void set_dots(Mode disp_mode, int *dots)
{
    /* Set decimal point display based on display mode and settings.
   */
    char am_pm[3]{};
    switch (disp_mode)
    {
        time_t now;
        struct tm timeinfo;

    case current_time:
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

    case in_sleep:
        break;

    case current_date:
        if (settings.d_divider == 0)
        {
            dots[2] = 1;
            dots[4] = 1;
        }
        break;

    case ip_address:
        dots[5] = 0;
        break;

    case ntp_syncing:
        dot_scroll(dots);
        break;

    case wifi_error:
        break;

    case connect_ap:
        break;
    }
}

int get_spi_data(int current_digit, char *disp_text, int *dots)
{
    /* Based on character string and current digit, generate data to be sent via
   * SPI
   */

    unsigned char chr{disp_text[current_digit]};
    int spi_data{};

    // If character is space, show dot or nothing at all
    if (chr <= (unsigned char)' ')
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