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
#include <esp_sleep.h>
#include <esp_sntp.h>

_VOID _EXFUN(tzset, (_VOID));
int _EXFUN(setenv,
           (const char *__string, const char *__value, int __overwrite));

// Flags
bool credentials_saved = false;
bool wifi_connected = false;
bool time_synced = false;
bool ap_active = false;
bool server_active = false;
bool at_night = false;

// PWM and photoresistor
const int LDR_PIN = 36;
const int LED_PIN = 0; // FIXME: Delete later
const int PWM_CHANNEL = 0;
int min_brightness = 2500;
int max_brightness = 3500;

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
const char *HOSTNAME = "Callisto";
const char *AP_SSID = "callisto_config";
const char *AP_PASSWORD = "12345678";

void init_brightness()
{
  /* Define pulse wave modulation (PWM) output pin for boost converter and input
   * for light dependent resistor (LDR)
   * Max and min brightness is set from the LDR at this point so that the
   * brightness range is not zero during later calculations
   */

  const int PWM_PIN = 32;
  const int PWM_FREQ = 31250;
  const int PWM_RESOLUTION = 8;

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
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

  const int SPI_FREQ = 500000;

  pinMode(VFLOAD, OUTPUT);
  pinMode(VFBLANK, OUTPUT);
  digitalWrite(VFLOAD, LOW);
  digitalWrite(VFBLANK, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
}

class CallistoSettings
{
public:
  Preferences preferences;
  String ssid, password;
  int t_format, t_pad, t_divider;
  int d_format, d_pad, d_divider;
  int lo_brightness, hi_brightness;
  int night_start_h, night_start_m, night_end_h, night_end_m;
  String time_zone;

  void init()
  {
    preferences.begin("config");

    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");

    t_format = preferences.getInt(
        "t_format", 0);                             // [0]: 12 with dot, [1]: 12 without dot, [2]: 24
    t_pad = preferences.getInt("t_pad", 0);         // [0]: no pad, [1]: zero pad
    t_divider = preferences.getInt("t_divider", 0); // [0]: space, [1]: hyphen

    d_format =
        preferences.getInt("d_format", 0);  // [0]: MMDDYYYY, [1]: DDMMYYYY
    d_pad = preferences.getInt("d_pad", 0); // [0]: No pad, [1]: Zero pad
    d_divider =
        preferences.getInt("d_divider", 0); // [0] dot, [1] hyphen, [2] space

    lo_brightness = preferences.getInt("lo_brightness", 2);
    hi_brightness = preferences.getInt("hi_brightness", 9);

    night_start_h = preferences.getInt("night_start_h", 0);
    night_start_m = preferences.getInt("night_start_m", 0);
    night_end_h = preferences.getInt("night_end_h", 6);
    night_end_m = preferences.getInt("night_end_m", 0);

    time_zone = preferences.getString("time_zone", "EST5EDT,M3.2.0,M11.1.0");
  };
};

CallistoSettings settings;

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

  const int WIFI_TIMEOUT = 10000;

  Serial.printf("Connecting to %s ", settings.ssid.c_str());
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(settings.ssid.c_str(), settings.password.c_str());

  unsigned long wifi_start = millis();
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
}

// TODO: Make this neater
String ssid_options = "";

String get_ssids()
{
  String ssid_options = "";
  int n = WiFi.scanNetworks();

  for (int i = 0; i < n; i++)
  {
    ssid_options += "<option value=\"";
    ssid_options += WiFi.SSID(i);
    ssid_options += "\">\n";
  }

  return ssid_options;
}

String processor(const String &var)
{
  /* Processor replaces placeholder string within the webpage, based on user
   * preferences, or default values if not found.
   * In addition, this is used to only show wifi settings if in AP mode.
   */

  if (var == "ssid")
  {
    return settings.ssid;
  }
  else if (var == "password")
  {
    return settings.password;
  }

  if (ap_active)
  {
    if (var == "ssid_list")
    {
      return ssid_options;
    }
    else if (var == "ap_hide_begin")
    {
      return F("<!-- ");
    }
    else if (var == "ap_hide_end")
    {
      return F(" -->");
    }
  }
  else
  {
    if (var.indexOf("t_format_opt_") >= 0)
    {
      if (var == "t_format_opt_" + String(settings.t_format))
      {
        return F("selected");
      }
    }
    else if (var.indexOf("t_pad_opt_") >= 0)
    {
      if (var == "t_pad_opt_" + String(settings.t_pad))
      {
        return F("selected");
      }
    }
    else if (var.indexOf("t_divider_opt_") >= 0)
    {
      if (var == "t_divider_opt_" + String(settings.t_divider))
      {
        return F("selected");
      }
    }
    else if (var.indexOf("d_format_opt_") >= 0)
    {
      if (var == "d_format_opt_" + String(settings.d_format))
      {
        return F("selected");
      }
    }
    else if (var.indexOf("d_pad_opt_") >= 0)
    {
      if (var == "d_pad_opt_" + String(settings.d_pad))
      {
        return F("selected");
      }
    }
    else if (var.indexOf("d_divider_opt_") >= 0)
    {
      if (var == "d_divider_opt_" + String(settings.d_divider))
      {
        return F("selected");
      }
    }
    else if (var == "lo_brightness")
    {
      return String(settings.lo_brightness);
    }
    else if (var == "hi_brightness")
    {
      return String(settings.hi_brightness);
    }
    else if (var.indexOf("night_") >= 0)
    {
      char out[2];
      if (var == "night_start_h")
      {
        sprintf(out, "%02d", settings.night_start_h);
      }
      else if (var == "night_start_m")
      {
        sprintf(out, "%02d", settings.night_start_m);
      }
      else if (var == "night_end_h")
      {
        sprintf(out, "%02d", settings.night_end_h);
      }
      else if (var == "night_end_m")
      {
        sprintf(out, "%02d", settings.night_end_m);
      }
      return String(out);
    }
    else if (var == "time_zone")
    {
      return settings.time_zone;
    }
  }

  return String();
}

void on_cancel(AsyncWebServerRequest *request)
{
  /* If cancel button on settings page is clicked
   */

  request->send(
      200, "text/plain",
      "Preferences closed without saving.\n\nYou may now close this page.");
}

void on_get(AsyncWebServerRequest *request)
{
  /* If submit button on settings page is clicked, save them using the
   * preferences library.
   */

  request->send(200, "text/plain",
                "Preferences saved.\nCallisto will now reboot.\n\nYou may "
                "close this page.");
  delay(100);

  int params = request->params();

  for (int i = 0; i < params; i++)
  {
    AsyncWebParameter *p = request->getParam(i);

    if (p->name() == "ssid" || p->name() == "password" ||
        p->name() == "time_zone")
    { // Save as Strings
      settings.preferences.putString(p->name().c_str(), p->value());
    }
    else if (p->name() == "night_start")
    {
      settings.preferences.putInt("night_start_h",
                                  p->value().substring(0, 2).toInt());
      settings.preferences.putInt("night_end_m",
                                  p->value().substring(3, 5).toInt());
    }
    else if (p->name() == "night_end")
    {
      settings.preferences.putInt("night_end_h",
                                  p->value().substring(0, 2).toInt());
      settings.preferences.putInt("night_end_m",
                                  p->value().substring(3, 5).toInt());
    }
    else
    { // Save as ints
      settings.preferences.putInt(p->name().c_str(), p->value().toInt());
    }
  }

  server.end();

  ESP.restart();
}

void on_factory_reset(AsyncWebServerRequest *request)
{
  /* If reset button on settings page is clicked, delete all preferences and
   * reboot.
   */
  request->send(200, "text/plain",
                "All preferences reset to factory defaults.\nCallisto will now "
                "reboot.\n\nYou may "
                "close this page.");
  delay(100);

  settings.preferences.clear();

  server.end();

  ESP.restart();
}

class CaptiveRequestHandler : public AsyncWebHandler
{
public:
  CaptiveRequestHandler() { server.on("/get", HTTP_POST, on_get); }

  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request) { return true; }

  void handleRequest(AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }
};

void init_ap()
{
  /* Initialize the access point (AP), with a captive portal that will
   * automatically be shown once connected.
   */

  // TODO: List visible SSIDs
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  dns_server.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);

  server.begin();
  ap_active = true;
  Serial.print("AP Address: ");
  Serial.println(WiFi.softAPIP());
}

void init_server()
{
  /* Initialize the settings webpage server
   */

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false, processor); });
  server.on("/get", HTTP_POST, on_get);
  server.on("/factory_reset", HTTP_GET, on_factory_reset);
  server.on("/cancel", HTTP_GET, on_cancel);

  server.begin();
  server_active = true;
}

void init_font_table()
{
  /* Create the font table and digit grid output based on which pins are being
   * used on the MAX6921 VFD driver chip.
   */

  const int SEGMENT_OUT[] = {
      11, // Segment A
      13, // Segment B
      16, // Segment C
      17, // Segment D
      15, // Segment E
      12, // Segment F
      14  // Segment G
  };
  const int SEGMENTS[] = {
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
  for (int i = 0; i < sizeof(SEGMENTS) / sizeof(SEGMENTS[0]); i++)
  {
    for (int j = 0; j < 7; j++)
    {
      if (bitRead(SEGMENTS[i], j))
      {
        bitSet(font_table[i], SEGMENT_OUT[j]);
      }
    }
  }

  const int DOT_SEGMENT_OUT = 18; // Decimal point segment
  dot = 1 << DOT_SEGMENT_OUT;

  const int GRID_OUT[] = {
      9, // Digit 9
      5, // Digit 8
      3, // Digit 7
      2, // Digit 6
      6, // Digit 5
      1, // Digit 4
      7, // Digit 3
      0, // Digit 2
      8  // Digit 1
  };
  for (int i = 0; i < sizeof(GRID_OUT) / sizeof(GRID_OUT[0]); i++)
  {
    bitSet(digit_table[i], GRID_OUT[i]);
  }
};

void init_sntp()
{
  /* Connect to the the simple network time protocol (SNTP) server, and retrieve
   * the current time. Also set the time zone based on user settings.
   */

  const char *const ntp_server[] = {
      "pool.ntp.org",
      "0.pool.ntp.org",
      "1.pool.ntp.org",
      "2.pool.ntp.org",
      "3.pool.ntp.org",
  };

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  for (int i = 0; i < sizeof(ntp_server) / sizeof(ntp_server[0]); i++)
  {
    sntp_setservername(i, (char *)ntp_server[i]);
  }
  sntp_init();

  setenv("TZ", settings.time_zone.c_str(), 1);
  tzset();
}

void adjust_brightness()
{
  /* Set the anode voltage based on ambient brightness.
   * Target voltages used in the calculation do not reflect actual boost
   * converter outputs for whatever reason...
   */

  // TODO: Use PWM blanking instead of boost converter voltage?
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

void set_touch_state(bool &touch_state, bool &previous_touch_state,
                     unsigned long &touch_start, unsigned long &touch_end)
{
  /* Figure out when touch starts and ends, and set the time in ms.
   */

  uint16_t touch_value;
  unsigned long now_ms = millis();

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

  time_t now = time(0);
  struct tm timeinfo = *localtime(&now);

  int current_time = timeinfo.tm_hour * 100 + timeinfo.tm_min;
  static int night_start =
      settings.night_start_h * 100 + settings.night_start_m;
  static int night_end = settings.night_end_h * 100 + settings.night_end_m;

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

  const int SHOW_FOR = 5 * 1000;
  const int SERVER_START = 10 * 1000;
  const int SERVER_TIMEOUT = 10 * 60 * 1000;

  unsigned long now_ms;
  static bool touch_state = false;
  bool previous_touch_state = touch_state;
  static unsigned long touch_start = 0, touch_end = 0;
  static unsigned long server_start_time;

  if (ap_active)
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

void set_disp_text(int disp_mode, char *disp_text, int num)
{
  /* Set display data based on display mode
   */

  time_t now;
  struct tm timeinfo;

  switch (disp_mode)
  {
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

    time_t now = time(0);
    struct tm timeinfo = *localtime(&now);

    int current_time = timeinfo.tm_hour * 100 + timeinfo.tm_min;
    int night_end = settings.night_end_h * 100 + settings.night_end_m;
    int deep_sleep_sec;

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
    esp_deep_sleep(deep_sleep_sec * 1000000);

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
    dns_server.processNextRequest();
    break;

  case 6: // AP active
    digitalWrite(VFBLANK, LOW);
    sprintf(disp_text, " %-8s", "connect");
    dns_server.processNextRequest();
    break;
  }
}

void dot_scroll(int *dots)
{
  /* Generate array for scrolling dots
   */

  unsigned long now_ms = millis();

  dots[(now_ms / 500) % 8 + 1] = 1;
}

void set_dots(int disp_mode, int *dots)
{
  /* Set decimal point display based on display mode and settings.
   */

  switch (disp_mode)
  {
  case 0: // Time
    if (settings.t_format == 0)
    {
      char am_pm[3];

      time_t now = time(0);
      struct tm timeinfo = *localtime(&now);
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

  unsigned char chr = disp_text[current_digit];
  int spi_data;

  // If character is space, show dot or nothing at all
  if (chr <= (unsigned char)' ')
  {
    spi_data = dots[current_digit] * (dot + digit_table[current_digit]);
    return spi_data;
  }
  // Else, retrieve data based on font table
  else
  {
    if (chr > (unsigned char)'Z')
    { // If lower case, convert to upper case
      chr -= 32;
    }
    if (chr > (unsigned char)'Z')
    { // If still after 'Z', return -1
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
  for (int i = 0; i < 3; i++)
  {
    int tube_out = (unsigned int)spi_data >> (8 * (2 - i)) & 0xff;
    SPI.transfer(tube_out);
  }
  digitalWrite(VFLOAD, HIGH);
}

void brightness_utilities()
{
  /* Various utilities regarding brightness
   * Set min and max brightness, based on what was seen
   * Shrink brightness range every 2 days, in order to account for brightness
   * outliers.
   */

  int brightness = analogRead(LDR_PIN);
  
  if (max_brightness < brightness)
  {
    max_brightness = brightness;
  }
  if (brightness < min_brightness)
  {
    min_brightness = brightness;
  }

  const int UTILITY_FREQUENCY = 1000 * 60 * 60 * 24 * 2;
  const float DAMP_P = .02;
  static unsigned long now_ms = millis();
  unsigned long last_run = 0;

  if (now_ms - last_run >= UTILITY_FREQUENCY)
  {
    last_run = now_ms;
    int brightness_mean = (max_brightness + min_brightness) / 2;
    max_brightness = max(int(max_brightness * (1 - DAMP_P)), brightness_mean);
    min_brightness = min(int(max_brightness * (1 + DAMP_P)), brightness_mean);
  }
}

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

  brightness_utilities();

  current_digit++;
  current_digit %= 9;
}
