#include <Arduino.h>
#include <esp32-hal-touch.h>
#include <movingAvg.h>

#include "init.h"
#include "config_server.h"
#include "display.h"

// Flags
RTC_DATA_ATTR int boot_count{0};
RTC_DATA_ATTR bool credentials_saved{false};
RTC_DATA_ATTR bool wifi_connected{false};
RTC_DATA_ATTR bool time_synced{false};
bool ap_active{false};
bool server_active{false};

// PWM and photoresistor
constexpr int LDR_PIN{36};
constexpr int BOOST_CHANNEL{0};
movingAvg ldr_reading(50);

// Touch
constexpr int TOUCH_THRESHOLD{400};
unsigned long touch_wake_time{};

// SPI interface
constexpr int VFLOAD{5};
constexpr int BLANK_CHANNEL{2};
constexpr int VFBLANK{21};

// Font Table
int font_table[46]{};
int digit_table[9]{};
int dot{};

DNSServer dns_server;
AsyncWebServer server(80);
CallistoSettings settings;
String ssid_options;

void setup()
{
  Serial.begin(115200);

  init_brightness();
  init_touch();
  init_spi();
  init_spiffs();
  init_font_table();
  settings.init();

  if (settings.ssid.compareTo("") != 0)
  {
    credentials_saved = true;
  }

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
  {
    touch_wake_time = millis();
  }
  else
  {
    if (credentials_saved)
    {
      init_wifi();
    }

    if (wifi_connected)
    {
      init_power_save();
      init_sntp();
    }
    else if (boot_count)
    {
      init_power_save();
      time_synced = true;
      wifi_connected = true;
    }
  }

  ++boot_count;
}

void loop()
{
  static int current_digit{0};
  static char disp_text[10]{};
  int dots[9]{};

  static int min_brightness{4095};
  static int max_brightness{0};

  // Get ambient light and adjust PWM
  utilities(&min_brightness, &max_brightness);
  adjust_brightness(min_brightness, max_brightness);

  // Set display text and dots based on mode
  Mode disp_mode{get_mode()};
  set_text(disp_mode, disp_text);
  set_dots(disp_mode, dots);

  //  Send digit via SPI
  int spi_data{get_spi_data(current_digit, disp_text, dots)};
  send_spi_data(spi_data);

  ++current_digit;
  current_digit %= 9;
}
