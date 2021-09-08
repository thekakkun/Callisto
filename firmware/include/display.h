#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <SPI.h>

extern const int LDR_PIN;
extern const int PWM_CHANNEL;
void adjust_brightness(int min_brightness, int max_brightness);
void brightness_utilities(int *min_brightness, int *max_brightness);
extern const int LED_PIN; // TODO: Delete later
extern const int TOUCH_THRESHOLD;
void set_touch_state(bool &touch_state, bool &previous_touch_state,
                     unsigned long &touch_start, unsigned long &touch_end);

extern bool ap_active;
extern bool wifi_connected;
extern bool credentials_saved;
extern bool server_active;
extern bool time_synced;
extern const int VFBLANK;
bool is_night();
int get_mode();
void set_text(int disp_mode, char *disp_text);

void dot_scroll(int *dots);
void set_dots(int disp_mode, int *dots);

extern int dot;
extern int font_table[46];
extern int digit_table[9];
extern const int VFLOAD;
int get_spi_data(int current_digit, char *disp_text, int *dots);
void send_spi_data(int spi_data);

#endif