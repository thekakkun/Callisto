#include <Arduino.h>
#include <Preferences.h>

class CallistoSettings {
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