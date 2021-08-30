#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp32-hal-touch.h>
#include <esp_pm.h>
#include <esp_sntp.h>

#include "settings.h"

_VOID _EXFUN(tzset, (_VOID));
int _EXFUN(setenv,
           (const char* __string, const char* __value, int __overwrite));

// Flags
bool credentials_saved = false;
bool wifi_connected = false;
bool time_synced = false;
bool ap_active = false;
bool server_active = false;
bool at_night = false;

// PWM and photoresistor
const int LDR_PIN = 36;
const int LED_PIN = 0;  // FIXME: Delete later
const int PWM_CHANNEL = 0;
int min_brightness, max_brightness;

// Touch
const int TOUCH_THRESHOLD = 150;

// SPI interface
const int VFLOAD = 5;
const int VFBLANK = 21;

// Font Table
int font_table[46];
int digit_table[9];
int dot;

// Network
DNSServer dns_server;
AsyncWebServer server(80);
const char* HOSTNAME = "Callisto";
const char* AP_SSID = "callisto_config";
const char* AP_PASSWORD = "12345678";

CallistoSettings settings;

void init_brightness() {
  const int PWM_PIN = 32;
  const int PWM_FREQ = 31250;
  const int PWM_RESOLUTION = 8;

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  int brightness = analogRead(LDR_PIN);
  max_brightness = brightness;
  min_brightness = brightness - 1;
}

void init_touch() {
  // TODO: Use interrupt?
  touch_pad_init();
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  touch_pad_config(TOUCH_PAD_GPIO15_CHANNEL, TOUCH_THRESHOLD);
  touch_pad_filter_start(10);
}

void init_spi() {
  const int SPI_FREQ = 500000;

  pinMode(VFLOAD, OUTPUT);
  pinMode(VFBLANK, OUTPUT);
  digitalWrite(VFLOAD, LOW);
  digitalWrite(VFBLANK, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
}

void init_spiffs() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS");
  }
}

void init_wifi() {
  const int WIFI_TIMEOUT = 10000;

  Serial.printf("Connecting to %s ", settings.ssid.c_str());
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(settings.ssid.c_str(), settings.password.c_str());

  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    unsigned long now_ms = millis();
    if (wifi_start + WIFI_TIMEOUT < now_ms) {
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    // TODO: Wifi sleep
    WiFi.setSleep(true);
    wifi_connected = true;
    Serial.println(" CONNECTED!");
    Serial.print("Local Address: ");
    Serial.println(WiFi.localIP());
  }
}

String processor(const String& var) {
  if (var == "ssid") {
    return settings.ssid;
  } else if (var == "password") {
    return settings.password;
  }
  if (ap_active) {
    if (var == "ap_hide_begin") {
      return F("<!-- ");
    } else if (var == "ap_hide_end") {
      return F(" -->");
    }
  } else {
    if (var.indexOf("t_format_opt_") >= 0) {
      if (var == "t_format_opt_" + String(settings.t_format)) {
        return F("selected");
      }
    } else if (var.indexOf("t_pad_opt_") >= 0) {
      if (var == "t_pad_opt_" + String(settings.t_pad)) {
        return F("selected");
      }
    } else if (var.indexOf("t_divider_opt_") >= 0) {
      if (var == "t_divider_opt_" + String(settings.t_divider)) {
        return F("selected");
      }
    } else if (var.indexOf("d_format_opt_") >= 0) {
      if (var == "d_format_opt_" + String(settings.d_format)) {
        return F("selected");
      }
    } else if (var.indexOf("d_pad_opt_") >= 0) {
      if (var == "d_pad_opt_" + String(settings.d_pad)) {
        return F("selected");
      }
    } else if (var.indexOf("d_divider_opt_") >= 0) {
      if (var == "d_divider_opt_" + String(settings.d_divider)) {
        return F("selected");
      }
    } else if (var == "lo_brightness") {
      return String(settings.lo_brightness);
    } else if (var == "hi_brightness") {
      return String(settings.hi_brightness);
    } else if (var.indexOf("night_") >= 0) {
      char out[2];
      if (var == "night_start_h") {
        sprintf(out, "%02d", settings.night_start_h);
      } else if (var == "night_start_m") {
        sprintf(out, "%02d", settings.night_start_m);
      } else if (var == "night_end_h") {
        sprintf(out, "%02d", settings.night_end_h);
      } else if (var == "night_end_m") {
        sprintf(out, "%02d", settings.night_end_m);
      }
      return String(out);
    } else if (var == "time_zone") {
      return settings.time_zone;
    }
  }

  return String();
}

void on_cancel(AsyncWebServerRequest* request) {
  request->send(
      200, "text/plain",
      "Preferences closed without saving.\n\nYou may now close this page.");
}

void on_get(AsyncWebServerRequest* request) {
  request->send(200, "text/plain",
                "Preferences saved.\nCallisto will now reboot.\n\nYou may "
                "close this page.");

  int params = request->params();

  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);

    if (p->name() == "ssid" || p->name() == "password" ||
        p->name() == "time_zone") {  // Save as Strings
      settings.preferences.putString(p->name().c_str(), p->value());
    } else if (p->name() == "night_start") {
      settings.preferences.putInt("night_start_h",
                                  p->value().substring(0, 2).toInt());
      settings.preferences.putInt("night_end_m",
                                  p->value().substring(3, 5).toInt());
    } else if (p->name() == "night_end") {
      settings.preferences.putInt("night_end_h",
                                  p->value().substring(0, 2).toInt());
      settings.preferences.putInt("night_end_m",
                                  p->value().substring(3, 5).toInt());
    } else {  // Save as ints
      settings.preferences.putInt(p->name().c_str(), p->value().toInt());
    }
  }

  server.end();
  ESP.restart();
}

void on_factory_reset(AsyncWebServerRequest* request) {
  request->send(200, "text/plain",
                "All preferences reset to factory defaults.\nCallisto will now "
                "reboot.\n\nYou may "
                "close this page.");

  settings.preferences.clear();
  server.end();
  ESP.restart();
}

class CaptiveRequestHandler : public AsyncWebHandler {
 public:
  CaptiveRequestHandler() { server.on("/get", HTTP_POST, on_get); }

  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest* request) { return true; }

  void handleRequest(AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }
};

void init_ap() {
  // TODO: List visible SSIDs
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  dns_server.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);

  server.begin();
  ap_active = true;
  Serial.print("AP Address: ");
  Serial.println(WiFi.softAPIP());
}

void init_server() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/get", HTTP_POST, on_get);
  server.on("/factory_reset", HTTP_GET, on_factory_reset);
  server.on("/cancel", HTTP_GET, on_cancel);

  server.begin();
  server_active = true;
}

void init_font_table() {
  const int SEGMENT_OUT[] = {
      11,  // Segment A
      13,  // Segment B
      16,  // Segment C
      17,  // Segment D
      15,  // Segment E
      12,  // Segment F
      14   // Segment G
  };
  const int SEGMENTS[] = {
      0x40,  // -
      0x00,  // .
      0x00,  // /
      0x3F,  // 0
      0x06,  // 1
      0x5B,  // 2
      0x4F,  // 3
      0x66,  // 4
      0x6D,  // 5
      0x7D,  // 6
      0x07,  // 7
      0x7F,  // 8
      0x6F,  // 9
      0x00,  // :
      0x00,  // ;
      0x00,  // <
      0x00,  // =
      0x00,  // >
      0x00,  // ?
      0x00,  // @
      0x77,  // A
      0x7C,  // B
      0x39,  // C
      0x5E,  // D
      0x79,  // E
      0x71,  // F
      0x3D,  // G
      0x76,  // H
      0x30,  // I
      0x1E,  // J
      0x00,  // K
      0x38,  // L
      0x00,  // M
      0x54,  // N
      0x3F,  // O
      0x73,  // P
      0x67,  // Q
      0x50,  // R
      0x6D,  // S
      0x78,  // T
      0x3E,  // U
      0x00,  // V
      0x00,  // W
      0x00,  // X
      0x6E,  // Y
      0x00   // Z
  };
  for (int i = 0; i < sizeof(SEGMENTS) / sizeof(SEGMENTS[0]); i++) {
    for (int j = 0; j < 7; j++) {
      if (bitRead(SEGMENTS[i], j)) {
        bitSet(font_table[i], SEGMENT_OUT[j]);
      }
    }
  }

  const int DOT_SEGMENT_OUT = 18;  // Decimal point segment
  dot = 1 << DOT_SEGMENT_OUT;

  const int GRID_OUT[] = {
      9,  // Digit 9
      5,  // Digit 8
      3,  // Digit 7
      2,  // Digit 6
      6,  // Digit 5
      1,  // Digit 4
      7,  // Digit 3
      0,  // Digit 2
      8   // Digit 1
  };
  for (int i = 0; i < sizeof(GRID_OUT) / sizeof(GRID_OUT[0]); i++) {
    bitSet(digit_table[i], GRID_OUT[i]);
  }
};

void init_sntp() {
  const char* const ntp_server[] = {
      "pool.ntp.org",   "0.pool.ntp.org", "1.pool.ntp.org",
      "2.pool.ntp.org", "3.pool.ntp.org",
  };

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  for (int i = 0; i < sizeof(ntp_server) / sizeof(ntp_server[0]); i++) {
    sntp_setservername(i, (char*)ntp_server[i]);
  }
  sntp_init();

  setenv("TZ", settings.time_zone.c_str(), 1);
  tzset();
}

// Set brightness PWM based on ambient brightness
void adjust_brightness() {
  const int V_IN = 9, MIN_V = 15, MAX_V = 40, V_LIM = 55;
  const int V_RANGE = MAX_V - MIN_V;
  const float ETA = 0.8;

  // max and min output voltage, based on user set preferences
  float lo_brightness_v =
      MIN_V + float(V_RANGE * (settings.lo_brightness - 1) / 9);
  float hi_brightness_v =
      MIN_V + float(V_RANGE * (settings.hi_brightness - 1) / 9);
  float brightness_v_range = hi_brightness_v - lo_brightness_v;

  int brightness = analogRead(LDR_PIN);
  float ambient_brightness_p =
      float(brightness - min_brightness) / (max_brightness - min_brightness);
  float v_out = lo_brightness_v + ambient_brightness_p * brightness_v_range;
  int duty = int(255 * (V_IN * ETA / (V_LIM - v_out)));

  ledcWrite(PWM_CHANNEL, duty);
}

void set_touch_state(bool& touch_state, bool& previous_touch_state,
                     unsigned long& touch_start, unsigned long& touch_end) {
  uint16_t touch_value;
  unsigned long now_ms = millis();

  touch_pad_read_filtered(TOUCH_PAD_NUM3, &touch_value);

  if (touch_value < TOUCH_THRESHOLD) {
    touch_state = true;
    digitalWrite(LED_PIN, HIGH);  // FIXME: Delete later
    if (!previous_touch_state) {
      touch_start = now_ms;
      Serial.println("Touch start");
    }
  } else {
    touch_state = false;
    digitalWrite(LED_PIN, LOW);  // FIXME: Delete later
    if (previous_touch_state) {
      touch_end = now_ms;
      Serial.println("Touch end");
    }
  }
}

bool set_night() {
  time_t now = time(0);
  struct tm timeinfo = *localtime(&now);

  if (settings.night_start_h == timeinfo.tm_hour) {
    if (settings.night_start_m <= timeinfo.tm_min) {
      return true;
    } else {
      return false;
    }
  } else if (settings.night_start_h <= timeinfo.tm_hour &&
             timeinfo.tm_hour <= settings.night_end_h) {
    return true;
  } else if (timeinfo.tm_hour == settings.night_end_h) {
    if (timeinfo.tm_min <= settings.night_end_m) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

// Get mode based on success flags, current time, and touch reading
int get_mode() {
  const int SHOW_FOR = 5 * 1000;
  const int SERVER_START = 10 * 1000;
  const int SERVER_TIMEOUT = 10 * 60 * 1000;

  unsigned long now_ms = millis();
  static bool touch_state = false;
  bool previous_touch_state = touch_state;
  static unsigned long touch_start = 0, touch_end = 0;
  static unsigned long server_start_time;

  if (ap_active) {
    if (credentials_saved) {
      // credentials saved, but timed out -> wifi error
      return 5;
    } else {
      // no credentials -> connect to ap
      return 6;
    }

  } else if (server_active) {
    if (now_ms - server_start_time < SERVER_TIMEOUT) {
      // server active, not timed out -> show IP
      return 3;
    } else {
      // server timed out -> show time
      return 0;
    }

  } else {
    set_touch_state(touch_state, previous_touch_state, touch_start, touch_end);
    at_night = set_night();

    if (touch_state && now_ms - touch_start > SERVER_START) {
      server_start_time = millis();
      // held for long time -> start server
      return 3;

    } else if (!time_synced) {
      time_t now = time(0);
      struct tm timeinfo = *localtime(&now);

      if (timeinfo.tm_year > 70) {
        time_synced = true;
      }
      // time syncing -> show boot
      return 4;

    } else if (now_ms - touch_end < SHOW_FOR) {
      if (at_night) {
        // touched at night -> show time
        return 0;
      } else {
        // touched during day -> show date
        return 2;
      }

    } else {
      if (at_night) {
        // night -> clock off
        return 1;
      } else {
        // day -> show time
        return 0;
      }
    }
  }
}

void set_disp_text(int disp_mode, char* disp_text, int num) {
  time_t now;
  struct tm timeinfo;

  switch (disp_mode) {
    case 0:  // Time
      digitalWrite(VFBLANK, LOW);
      if (server_active) {
        server.end();
        server_active = false;
      }

      now = time(0);
      timeinfo = *localtime(&now);

      if (settings.t_format == 0 || settings.t_format == 1) {
        if (settings.t_pad == 0) {
          if (settings.t_divider == 0) {
            strftime(disp_text, 10, " %l %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %l-%M-%S", &timeinfo);
          }
        } else {  // t_pad == 1
          if (settings.t_divider == 0) {
            strftime(disp_text, 10, " %I %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %I-%M-%S", &timeinfo);
          }
        }
      } else {  // t_format == 2
        if (settings.t_pad == 0) {
          if (settings.t_divider == 0) {
            strftime(disp_text, 10, " %k %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %k-%M-%S", &timeinfo);
          }
        } else {  // t_pad == 1
          if (settings.t_divider == 0) {
            strftime(disp_text, 10, " %H %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %H-%M-%S", &timeinfo);
          }
        }
      }
      break;

    case 1:  // off
      // TODO: set to sleep mode. wake up via touch or timer? program?
      digitalWrite(VFBLANK, HIGH);
      break;

    case 2:  // date
      digitalWrite(VFBLANK, LOW);

      now = time(0);
      timeinfo = *localtime(&now);

      if (settings.d_format == 0) {
        if (settings.d_pad == 0) {
          if (settings.d_divider == 0) {
            strftime(disp_text, 10, " %m%e%Y", &timeinfo);
          } else if (settings.d_divider == 1) {
            strftime(disp_text, 10, " %m-%e-%y", &timeinfo);
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %m %e %y", &timeinfo);
          }
          if (disp_text[1] == '0') {
            disp_text[1] = ' ';
          }
        } else {  // d_pad == 1
          if (settings.d_divider == 0) {
            strftime(disp_text, 10, " %m%d%Y", &timeinfo);
          } else if (settings.d_divider == 1) {
            strftime(disp_text, 10, " %m-%d-%y", &timeinfo);
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %m %d %y", &timeinfo);
          }
        }
      } else {  // d_format == 1
        if (settings.d_pad == 0) {
          if (settings.d_divider == 0) {
            strftime(disp_text, 10, " %e%m%Y", &timeinfo);
            if (disp_text[3] == '0') {
              disp_text[3] = ' ';
            }
          } else if (settings.d_divider == 1) {
            strftime(disp_text, 10, " %e-%m-%y", &timeinfo);
            if (disp_text[4] == '0') {
              disp_text[4] = ' ';
            }
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %e %m %y", &timeinfo);
            if (disp_text[4] == '0') {
              disp_text[4] = ' ';
            }
          }
        } else {
          if (settings.d_divider == 0) {
            strftime(disp_text, 10, " %d%m%Y", &timeinfo);
          } else if (settings.d_divider == 1) {
            strftime(disp_text, 10, " %d-%m-%y", &timeinfo);
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %d %m %y", &timeinfo);
          }
        }
      }

      break;

    case 3:  // IP
    {
      digitalWrite(VFBLANK, LOW);
      if (!server_active) {
        init_server();
      }
      sprintf(disp_text, "%9d", WiFi.localIP()[3]);
      break;
    }

    case 4:  // NTP syncing
      digitalWrite(VFBLANK, LOW);
      sprintf(disp_text, " %-8s", "Callisto");
      break;

    case 5:  // Wifi error
      digitalWrite(VFBLANK, LOW);
      sprintf(disp_text, " %-8s", "net err");
      dns_server.processNextRequest();
      break;

    case 6:  // AP active
      digitalWrite(VFBLANK, LOW);
      sprintf(disp_text, " %-8s", "connect");
      dns_server.processNextRequest();
      break;
  }
}

void dot_scroll(int* dots) {
  unsigned long now_ms = millis();

  dots[(now_ms / 500) % 8 + 1] = 1;
}

void set_dots(int disp_mode, int* dots) {
  switch (disp_mode) {
    case 0:  // Time
      if (settings.t_format == 0) {
        char am_pm[3];

        time_t now = time(0);
        struct tm timeinfo = *localtime(&now);
        strftime(am_pm, 3, "%p", &timeinfo);

        if (strcmp(am_pm, "PM") == 0) {
          dots[0] = 1;
        }
      }
      break;

    case 1:  // off
      break;

    case 2:  // date
      if (settings.d_divider == 0) {
        dots[2] = 1;
        dots[4] = 1;
      }
      break;

    case 3:  // IP
      dots[5] = 0;
      break;

    case 4:  // ntp
      dot_scroll(dots);
      break;

    case 5:  // no wifi
      break;
  }
}

// Character to SPI out data via font table
int get_spi_data(int current_digit, char* disp_text, int* dots) {
  unsigned char chr = disp_text[current_digit];
  int spi_data;

  // If character is space, show dot or nothing at all
  if (chr <= (unsigned char)' ') {
    spi_data = dots[current_digit] * (dot + digit_table[current_digit]);
    return spi_data;
  }
  // Else, retrieve data based on font table
  else {
    if (chr > (unsigned char)'Z') {  // If lower case, convert to upper case
      chr -= 32;
    }
    if (chr > (unsigned char)'Z') {  // If still after 'Z', return -1
      return 0;
    }
  }
  spi_data = font_table[chr - '-'];
  spi_data += dots[current_digit] * dot;
  spi_data += digit_table[current_digit];

  return spi_data;
}

// Send data via SPI
void send_spi_data(int spi_data) {
  digitalWrite(VFLOAD, LOW);
  for (int i = 0; i < 3; i++) {
    int tube_out = (unsigned int)spi_data >> (8 * (2 - i)) & 0xff;
    SPI.transfer(tube_out);
  }
  digitalWrite(VFLOAD, HIGH);
}

void set_brightness_range() {
  int brightness = analogRead(LDR_PIN);

  if (max_brightness < brightness) {
    max_brightness = brightness;
  }
  if (brightness < min_brightness) {
    min_brightness = brightness;
  }
}
void run_utilities() {
  set_brightness_range();

  const int UTILITY_FREQUENCY = 1000 * 60 * 60 * 24 * 2;
  const float DAMP_P = .02;
  static unsigned long now_ms = millis();
  unsigned long last_run = 0;

  if (now_ms - last_run >= UTILITY_FREQUENCY) {
    last_run = now_ms;
    int brightness_mean = (max_brightness + min_brightness) / 2;
    max_brightness = max(int(max_brightness * (1 - DAMP_P)), brightness_mean);
    min_brightness = min(int(max_brightness * (1 + DAMP_P)), brightness_mean);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);    // FIXME: Delete later
  digitalWrite(LED_PIN, LOW);  // FIXME: Delete later

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

  settings.init();
  if (settings.ssid.compareTo("") == 0) {
  } else {
    credentials_saved = true;
  }

  if (credentials_saved) {
    // Connect to WiFi
    init_wifi();
    if (!wifi_connected) {
      Serial.print("\n");
      Serial.println("Wifi timed out");
      init_ap();
    }
  } else {
    // Start AP
    init_ap();
  }

  // Sync time if wifi connected
  if (wifi_connected) {
    init_font_table();
    init_sntp();
  }
}

void loop() {
  static int current_digit = 0;
  char disp_text[10];
  int dots[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Get ambient light and adjust PWM
  adjust_brightness();

  // Set display text and dots based on mode
  int disp_mode = get_mode();
  set_disp_text(disp_mode, disp_text, sizeof(disp_text));
  set_dots(disp_mode, dots);

  //  Send digit via SPI
  int spi_data = get_spi_data(current_digit, disp_text, dots);
  send_spi_data(spi_data);

  run_utilities();

  current_digit++;
  current_digit %= 9;
}
