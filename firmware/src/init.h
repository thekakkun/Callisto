#ifndef INIT_H
#define INIT_H

#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_sntp.h>

extern const int BOOST_CHANNEL;
void init_brightness();

extern const int TOUCH_THRESHOLD;
void touch_callback();
void init_touch();

extern const int VFLOAD;
extern const int VFBLANK;
extern const int BLANK_CHANNEL;
void init_spi();

void init_spiffs();

extern bool wifi_connected;
void init_wifi();

extern int font_table[46];
extern int digit_table[9];
extern int dot;
void init_font_table();

void init_power_save();

_VOID _EXFUN(tzset, (_VOID));
int _EXFUN(setenv,
           (const char *__string, const char *__value, int __overwrite));
void init_sntp();

#endif