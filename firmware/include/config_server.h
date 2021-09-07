#ifndef CONFIG_SERVER_H
#define CONFIG_SERVER_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>

extern DNSServer dns_server;
extern AsyncWebServer server;

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

    void init();
};
extern CallistoSettings settings;

/* Get list of SSIDs as options.
*/
String get_ssids();

/* Processor replaces placeholder string within the webpage, based on user
* preferences, or default values if not found.
* In addition, this is used to only show wifi settings if in AP mode.
*/
extern bool ap_active;
extern String ssid_options;
String processor(const String &var);

/* If cancel button on settings page is clicked
*/
void on_cancel(AsyncWebServerRequest *request);

/* If submit button on settings page is clicked, save them using the
* preferences library.
*/
void on_get(AsyncWebServerRequest *request);

/* If reset button on settings page is clicked, delete all preferences and
* reboot.
*/
void on_factory_reset(AsyncWebServerRequest *request);

/* Initialize the access point (AP), with a captive portal that will
* automatically be shown once connected.
*/

void init_ap();

/* Initialize the settings webpage server
*/
extern bool server_active;
void init_server();

#endif