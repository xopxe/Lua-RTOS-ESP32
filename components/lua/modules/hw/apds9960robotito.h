#ifndef APDS9960R_H
#define APDS9960R_H


#define COLOR_UNKNOWN "unknown"
#define COLOR_WHITE "white"
#define COLOR_BLACK "black"
struct RGB_set {
 char r;
 char g;
 char b;
};

struct HSV_set {
 signed int h;
 char s;
 char v;
};

struct color_range {
    char* name;
    int min;
    int max;
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


#endif  /*APDS9960R_H*/
