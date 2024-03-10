#ifndef APDS9960_H
#define APDS9960_H

#define COLOR_UNKNOWN_I -1
#define COLOR_BLACK_I -2
#define COLOR_WHITE_I -3
#define COLOR_UNUSED_I -127

#define COLOR_UNKNOWN "unknown"
#define COLOR_BLACK "black"
#define COLOR_WHITE "white"


struct RGB_set {
 uint16_t r;
 uint16_t g;
 uint16_t b;
};

struct HSV_set {
 signed int h;
 uint16_t s;
 uint16_t v;
};

struct color_range {
 char* name;
 signed int h;
 uint16_t s;
 uint16_t v;
};

// #define N_NAMED_COLORS 6
// static const color_range color_ranges[N_NAMED_COLORS] = {
//  {"red", 0, 60},
//  {"yellow", 60, 120},
//  {"green", 120, 180},
//  {"cyan", 180, 240},
//  {"blue", 240, 300},
//  {"magenta", 300, 360},
// };


#endif  /*APDS9960_H*/
