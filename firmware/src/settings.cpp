#include "settings.h"

void CallistoSettings::init() {
  preferences.begin("config");

  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");

  t_format = preferences.getInt(
      "t_format", 0);  // [0]: 12 with dot, [1]: 12 without dot, [2]: 24
  t_pad = preferences.getInt("t_pad", 0);          // [0]: no pad, [1]: zero pad
  t_divider = preferences.getInt("t_divider", 0);  // [0]: space, [1]: hyphen

  d_format = preferences.getInt("d_format", 0);  // [0]: MMDDYYYY, [1]: DDMMYYYY
  d_pad = preferences.getInt("d_pad", 0);        // [0]: No pad, [1]: Zero pad
  d_divider =
      preferences.getInt("d_divider", 0);  // [0] dot, [1] hyphen, [2] space

  lo_brightness = preferences.getInt("lo_brightness", 2);
  hi_brightness = preferences.getInt("hi_brightness", 9);

  night_start_h = preferences.getInt("night_start_h", 0);
  night_start_m = preferences.getInt("night_start_m", 0);
  night_end_h = preferences.getInt("night_end_h", 6);
  night_end_m = preferences.getInt("night_end_m", 0);

  time_zone = preferences.getString("time_zone", "EST5EDT,M3.2.0,M11.1.0");
};