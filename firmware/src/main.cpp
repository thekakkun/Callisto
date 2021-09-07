#include <Arduino.h>
#include <esp32-hal-touch.h>
#include <esp_pm.h>
#include <esp_sleep.h>

#include "init.h"
#include "config_server.h"
#include "display.h"

// Flags
bool credentials_saved{false};
bool wifi_connected{false};
bool time_synced{false};
bool ap_active{false};
bool server_active{false};

// PWM and photoresistor
constexpr int LDR_PIN{36};
constexpr int LED_PIN{0}; // FIXME: Delete later
constexpr int PWM_CHANNEL{0};

// Touch
constexpr int TOUCH_THRESHOLD{150};

// SPI interface
constexpr int VFLOAD{5};
constexpr int VFBLANK{21};

// Font Table
int font_table[46];
int digit_table[9];
int dot;

DNSServer dns_server;
AsyncWebServer server(80);
CallistoSettings settings;
String ssid_options = "";

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);   // FIXME: Delete later
  digitalWrite(LED_PIN, LOW); // FIXME: Delete later

  // TODO: Some sort of energy saving mode
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = 160;
  pm_config.min_freq_mhz = 80;
  pm_config.light_sleep_enable = true;

  esp_pm_configure(&pm_config);

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

  if (credentials_saved)
  {
    // Connect to WiFi
    init_wifi();
    if (!wifi_connected)
    {
      Serial.print("\n");
      Serial.println("Wifi timed out");
      init_ap();
    }
  }
  else
  {
    // Start AP
    ssid_options = get_ssids();
    init_ap();
  }

  // Sync time if wifi connected
  if (wifi_connected)
  {
    init_sntp();
  }
}

void loop()
{
  static int current_digit{0};
  static char disp_text[10];
  int dots[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  static int min_brightness{2500};
  static int max_brightness{2501};

  // Get ambient light and adjust PWM
  brightness_utilities(&min_brightness, &max_brightness);
  adjust_brightness(min_brightness, max_brightness);

  // Set display text and dots based on mode
  int disp_mode{get_mode()};
  set_text(disp_mode, disp_text);
  set_dots(disp_mode, dots);

  //  Send digit via SPI
  int spi_data{get_spi_data(current_digit, disp_text, dots)};
  send_spi_data(spi_data);

  ++current_digit;
  current_digit %= 9;
}
