#include "init.h"
#include "config_server.h"

void CallistoSettings::init()
{
    preferences.begin("config");

    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");

    t_format = preferences.getInt("t_format", 0);   // [0]: 12 with dot, [1]: 12 without dot, [2]: 24
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
    setenv("TZ", time_zone.c_str(), 1);
    tzset();
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
            char out[2]{};
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

void on_get(AsyncWebServerRequest *request)
{
    /* If submit button on settings page is clicked, save them using the
   * preferences library.
   */

    request->send(200, "text/plain",
                  "Preferences saved.\n"
                  "Callisto will now reboot.\n\n"
                  "You may close this page.");
    delay(100);

    unsigned int params{request->params()};

    for (int i{0}; i < params; ++i)
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

void on_cancel(AsyncWebServerRequest *request)
{
    /* If cancel button on settings page is clicked
   */

    request->send(
        200, "text/plain",
        "Preferences closed without saving.\n\n"
        "You may now close this page.");

    delay(100);
    server.end();
    ESP.restart();
}

void on_reset(AsyncWebServerRequest *request)
{
    /* If reset button on settings page is clicked, delete all preferences and
   * reboot.
   */

    request->send(
        200, "text/plain",
        "All preferences reset to factory defaults.\n"
        "Callisto will now reboot.\n\n"
        "You may close this page.");
    delay(100);

    settings.preferences.clear();
    server.end();
    ESP.restart();
}

String get_ssids()
{
    String ssid_options = "";
    int n = WiFi.scanNetworks();

    for (int i{0}; i < n; ++i)
    {
        ssid_options += "<option value=\"";
        ssid_options += WiFi.SSID(i);
        ssid_options += "\">\n";
    }

    return ssid_options;
}

void init_ap()
{
    /* Initialize the access point (AP), with a captive portal that will
   * automatically be shown once connected.
   */

    Serial.println("Starting access point");
    const char *AP_SSID = "callisto_config";
    const char *AP_PASSWORD = "12345678";

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

    // TODO: This is a lambda function. Try to understand it and make everything else better.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", String(), false, processor); });
    server.on("/reset", HTTP_GET, on_reset);
    server.on("/get", HTTP_POST, on_get);
    server.on("/cancel", HTTP_GET, on_cancel);

    server.begin();
    server_active = true;
}