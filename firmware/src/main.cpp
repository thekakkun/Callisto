#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp32-hal-touch.h>
#include <esp_sntp.h>

_VOID _EXFUN(tzset, (_VOID));
int _EXFUN(setenv,
           (const char* __string, const char* __value, int __overwrite));

// Flags
int credentials_saved = 0;
int wifi_connected = 0;
int time_synced = 0;
int ap_active = 0;
int server_active = 0;
int at_night = 0;

// PWM and photoresistor
const int LDR_PIN = 32;
const int PWM_CHANNEL = 0;
int min_brightness, max_brightness;

// Touch
const int TOUCH_THRESHOLD = 150;

// SPI interface
const int VFLOAD = 5;
const int VFBLANK = 27;

// Network
DNSServer dns_server;
AsyncWebServer server(80);
const char* HOSTNAME = "Callisto";
const char* AP_SSID = "callisto_config";
const char* AP_PASSWORD = "12345678";

// Config defaults
Preferences preferences;
String ssid = "";
String password = "";
int t_format = 0;   // [0]: 12 with dot, [1]: 12 without dot, [2]: 24
int t_pad = 0;      // [0]: no pad, [1]: zero pad
int t_divider = 0;  // [0]: space, [1]: hyphen
int d_format = 0;   // [0]: MMDDYYYY, [1]: DDMMYYYY
int d_pad = 0;      // [0]: No pad, [1]: Zero pad
int d_divider = 0;  // [0] dot, [1] hyphen, [2] space
int lo_brightness = 2;
int hi_brightness = 9;
int night_start_h = 0, night_start_m = 0, night_end_h = 6, night_end_m = 0;
String time_zone = "EST5EDT,M3.2.0,M11.1.0";

void init_brightness() {
  const int PWM_PIN = 16;
  const int PWM_FREQ = 31250;
  const int PWM_RESOLUTION = 8;

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  int brightness = analogRead(LDR_PIN);
  max_brightness = brightness;
  min_brightness = brightness - 1;
}
void init_touch() {
  touch_pad_init();
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  touch_pad_config(TOUCH_PAD_NUM3, TOUCH_THRESHOLD);
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

void init_preference() {
  // TODO: Change display based on these
  preferences.begin("config");

  ssid = preferences.getString("ssid", ssid);
  password = preferences.getString("password", password);

  t_format = preferences.getInt("t_format", t_format);
  t_pad = preferences.getInt("t_pad", t_pad);
  t_divider = preferences.getInt("t_divider", t_divider);

  d_format = preferences.getInt("d_format", d_format);
  d_pad = preferences.getInt("d_pad", d_pad);
  d_divider = preferences.getInt("d_divider", d_divider);

  lo_brightness = preferences.getInt("lo_brightness", lo_brightness);
  hi_brightness = preferences.getInt("hi_brightness", hi_brightness);

  night_start_h = preferences.getInt("night_start_h", night_start_h);
  night_start_m = preferences.getInt("night_start_m", night_start_m);
  night_end_h = preferences.getInt("night_end_h", night_end_h);
  night_end_m = preferences.getInt("night_end_m", night_end_m);

  time_zone = preferences.getString("time_zone", time_zone);

  if (ssid.compareTo("") == 0) {
  } else {
    credentials_saved = 1;
  }
}

void init_spiffs() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS");
  }
}

void init_wifi() {
  const int WIFI_TIMEOUT = 10000;

  Serial.printf("Connecting to %s ", ssid.c_str());
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid.c_str(), password.c_str());

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
    wifi_connected = 1;
    Serial.println(" CONNECTED!");
    Serial.print("Local Address: ");
    Serial.println(WiFi.localIP());
  }
}

String processor(const String& var) {
  if (var == "ssid") {
    return ssid;
  } else if (var == "password") {
    return password;
  }
  if (ap_active) {
    if (var == "ap_hide_begin") {
      return F("<!-- ");
    } else if (var == "ap_hide_end") {
      return F(" -->");
    }
  } else {
    if (var.indexOf("t_format_opt_") >= 0) {
      if (var == "t_format_opt_" + String(t_format)) {
        return F("selected");
      }
    } else if (var.indexOf("t_pad_opt_") >= 0) {
      if (var == "t_pad_opt_" + String(t_pad)) {
        return F("selected");
      }
    } else if (var.indexOf("t_divider_opt_") >= 0) {
      if (var == "t_divider_opt_" + String(t_divider)) {
        return F("selected");
      }
    } else if (var.indexOf("d_format_opt_") >= 0) {
      if (var == "d_format_opt_" + String(d_format)) {
        return F("selected");
      }
    } else if (var.indexOf("d_pad_opt_") >= 0) {
      if (var == "d_pad_opt_" + String(d_pad)) {
        return F("selected");
      }
    } else if (var.indexOf("d_divider_opt_") >= 0) {
      if (var == "d_divider_opt_" + String(d_divider)) {
        return F("selected");
      }
    } else if (var == "lo_brightness") {
      return String(lo_brightness);
    } else if (var == "hi_brightness") {
      return String(hi_brightness);
    } else if (var.indexOf("night_") >= 0) {
      char out[2];
      if (var == "night_start_h") {
        sprintf(out, "%02d", night_start_h);
      } else if (var == "night_start_m") {
        sprintf(out, "%02d", night_start_m);
      } else if (var == "night_end_h") {
        sprintf(out, "%02d", night_end_h);
      } else if (var == "night_end_m") {
        sprintf(out, "%02d", night_end_m);
      }
      return String(out);
    } else if (var == "time_zone") {
      return time_zone;
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
      preferences.putString(p->name().c_str(), p->value());
    } else if (p->name() == "night_start") {
      preferences.putInt("night_start_h", p->value().substring(0, 2).toInt());
      preferences.putInt("night_end_m", p->value().substring(3, 5).toInt());
    } else if (p->name() == "night_end") {
      preferences.putInt("night_end_h", p->value().substring(0, 2).toInt());
      preferences.putInt("night_end_m", p->value().substring(3, 5).toInt());
    } else {  // Save as ints
      preferences.putInt(p->name().c_str(), p->value().toInt());
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

  preferences.clear();
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
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  dns_server.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);

  server.begin();
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
}

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

  setenv("TZ", time_zone.c_str(), 1);
  tzset();
}

// Set brightness PWM based on ambient brightness
void adjust_brightness() {
  // TODO: Set reasonable range for MIN_V and MAX_V
  const int V_IN = 9, MIN_V = 15, MAX_V = 35, V_LIM = 50;
  const int V_RANGE = MAX_V - MIN_V;
  const float ETA = 0.8;

  // max and min output voltage, based on user set preferences
  float lo_brightness_v = MIN_V + float(V_RANGE * (lo_brightness - 1) / 9);
  float hi_brightness_v = MIN_V + float(V_RANGE * (hi_brightness - 1) / 9);
  float brightness_v_range = hi_brightness_v - lo_brightness_v;

  int brightness = analogRead(LDR_PIN);
  float ambient_brightness_p =
      float(brightness - min_brightness) / (max_brightness - min_brightness);
  float v_out = lo_brightness_v + ambient_brightness_p * brightness_v_range;
  int duty = int(255 * (V_IN * ETA / (V_LIM - v_out)));

  ledcWrite(PWM_CHANNEL, duty);
}

void set_touch_state(int& touch_state, int& previous_touch_state,
                     unsigned long& touch_start, unsigned long& touch_end) {
  uint16_t touch_value;
  unsigned long now_ms = millis();

  touch_pad_read_filtered(TOUCH_PAD_NUM3, &touch_value);

  if (touch_value < TOUCH_THRESHOLD) {
    touch_state = 1;
    if (previous_touch_state == 0) {
      touch_start = now_ms;
      Serial.println("Touch start");
    }
  } else {
    touch_state = 0;
    if (previous_touch_state == 1) {
      touch_end = now_ms;
      Serial.println("Touch end");
    }
  }
}

int set_night() {
  time_t now = time(0);
  struct tm timeinfo = *localtime(&now);

  if (night_start_h == timeinfo.tm_hour) {
    if (night_start_m <= timeinfo.tm_min) {
      return 1;
    }
  } else if (night_start_h <= timeinfo.tm_hour) {
    return 1;
  }

  if (night_end_h == timeinfo.tm_hour) {
    if (night_end_m <= timeinfo.tm_min) {
      return 0;
    }
  } else if (night_end_h <= timeinfo.tm_hour) {
    return 0;
  }
}

// Get mode based on success flags, current time, and touch reading
int get_mode() {
  const int SHOW_FOR = 5 * 1000;
  const int SERVER_START = 10 * 1000;
  const int SERVER_TIMEOUT = 10 * 60 * 1000;

  unsigned long now_ms = millis();
  static int touch_state = 0;
  int previous_touch_state = touch_state;
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
        time_synced = 1;
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
        server_active = 0;
      }

      now = time(0);
      timeinfo = *localtime(&now);

      if (t_format == 0 || t_format == 1) {
        if (t_pad == 0) {
          if (t_divider == 0) {
            strftime(disp_text, 10, " %l %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %l-%M-%S", &timeinfo);
          }
        } else {  // t_pad == 1
          if (t_divider == 0) {
            strftime(disp_text, 10, " %I %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %I-%M-%S", &timeinfo);
          }
        }
      } else {  // t_format == 2
        if (t_pad == 0) {
          if (t_divider == 0) {
            strftime(disp_text, 10, " %k %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %k-%M-%S", &timeinfo);
          }
        } else {  // t_pad == 1
          if (t_divider == 0) {
            strftime(disp_text, 10, " %H %M %S", &timeinfo);
          } else {  // t_divider == 1
            strftime(disp_text, 10, " %H-%M-%S", &timeinfo);
          }
        }
      }
      break;

    case 1:  // off
      digitalWrite(VFBLANK, HIGH);
      break;

    case 2:  // date
      digitalWrite(VFBLANK, LOW);

      now = time(0);
      timeinfo = *localtime(&now);

      if (d_format == 0) {
        if (d_pad == 0) {
          if (d_divider == 0) {
            strftime(disp_text, 10, " %m%e%Y", &timeinfo);
          } else if (d_divider == 1) {
            strftime(disp_text, 10, " %m-%e-%y", &timeinfo);
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %m %e %y", &timeinfo);
          }
          if (disp_text[1] == '0') {
            disp_text[1] = ' ';
          }
        } else {  // d_pad == 1
          if (d_divider == 0) {
            strftime(disp_text, 10, " %m%d%Y", &timeinfo);
          } else if (d_divider == 1) {
            strftime(disp_text, 10, " %m-%d-%y", &timeinfo);
          } else {  // d_divider == 2
            strftime(disp_text, 10, " %m %d %y", &timeinfo);
          }
        }
      } else {  // d_format == 1
        if (d_pad == 0) {
          if (d_divider == 0) {
            strftime(disp_text, 10, " %e%m%Y", &timeinfo);
            if (disp_text[3] == '0') {
              disp_text[3] = ' ';
            }
          } else if (d_divider == 1) {
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
          if (d_divider == 0) {
            strftime(disp_text, 10, " %d%m%Y", &timeinfo);
          } else if (d_divider == 1) {
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
        server_active = 1;
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
      if (t_format == 0) {
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
      if (d_divider == 0) {
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
  const int FONT_TABLE[] = {
      0b000000000100000000000000,  // -
      0b000000000000000000000000,  // .
      0b000000000000000000000000,  // /
      0b000000111011100000000000,  // 0
      0b000000010010000000000000,  // 1
      0b000000101110100000000000,  // 2
      0b000000110110100000000000,  // 3
      0b000000010111000000000000,  // 4
      0b000000110101100000000000,  // 5
      0b000000111101100000000000,  // 6
      0b000000010010100000000000,  // 7
      0b000000111111100000000000,  // 8
      0b000000110111100000000000,  // 9
      0b000000000000000000000000,  // :
      0b000000000000000000000000,  // ;
      0b000000000000000000000000,  // <
      0b000000000000000000000000,  // =
      0b000000000000000000000000,  // >
      0b000000000000000000000000,  // ?
      0b000000000000000000000000,  // @
      0b000000011111100000000000,  // A
      0b000000111101000000000000,  // B
      0b000000101001100000000000,  // C
      0b000000111110000000000000,  // D
      0b000000101101100000000000,  // E
      0b000000001101100000000000,  // F
      0b000000111001100000000000,  // G
      0b000000011111000000000000,  // H
      0b000000001001000000000000,  // I
      0b000000111010000000000000,  // J
      0b000000000000000000000000,  // K
      0b000000101001000000000000,  // L
      0b000000000000000000000000,  // M
      0b000000011100000000000000,  // N
      0b000000111011100000000000,  // O
      0b000000001111100000000000,  // P
      0b000000010111100000000000,  // Q
      0b000000001100000000000000,  // R
      0b000000110101100000000000,  // S
      0b000000101101000000000000,  // T
      0b000000111011000000000000,  // U
      0b000000000000000000000000,  // V
      0b000000000000000000000000,  // W
      0b000000000000000000000000,  // X
      0b000000110111000000000000,  // Y
      0b000000000000000000000000   // Z
  };
  const int DOT = 0b000001000000000000000000;  // Dot
  const int DIGIT_TABLE[] = {
      0b000000000000001000000000,  // Digit 1
      0b000000000000000000100000,  // Digit 2
      0b000000000000000000001000,  // Digit 3
      0b000000000000000000000100,  // Digit 4
      0b000000000000000001000000,  // Digit 5
      0b000000000000000000000010,  // Digit 6
      0b000000000000000010000000,  // Digit 7
      0b000000000000000000000001,  // Digit 8
      0b000000000000000100000000,  // Digit 9
  };
  unsigned char chr = disp_text[current_digit];
  int spi_data;

  // If character is space, show dot or nothing at all
  if (chr <= (unsigned char)' ') {
    spi_data = dots[current_digit] * (DOT + DIGIT_TABLE[current_digit]);
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
  spi_data = FONT_TABLE[chr - '-'];
  spi_data += dots[current_digit] * DOT;
  spi_data += DIGIT_TABLE[current_digit];

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
  // time_t now = time(0);
  // struct tm timeinfo = *localtime(&now);
  set_brightness_range();

  // TODO: Brightness dampener
}

void setup() {
  Serial.begin(115200);

  init_brightness();
  init_touch();
  init_spi();
  init_spiffs();
  init_preference();

  if (credentials_saved) {
    // Connect to WiFi
    init_wifi();
    if (!wifi_connected) {
      Serial.print("\n");
      Serial.println("Wifi timed out");
      init_ap();
      ap_active = 1;
    }
  } else {
    // Start AP
    init_ap();
    ap_active = 1;
  }

  // Sync time if wifi connected
  if (wifi_connected) {
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
