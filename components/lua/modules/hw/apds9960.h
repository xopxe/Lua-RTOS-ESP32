#ifndef APDS9960_H
#define APDS9960_H

struct RGB_set {
 unsigned char r;
 unsigned char g;
 unsigned char b;
};
 
struct HSV_set {
 signed int h;
 unsigned char s;
 unsigned char v;
};

struct color_range {
    const char* name;
    int min;
    int max;
};

static const color_range color_ranges[6] = {
 {"red", 0, 60},
 {"yellow", 60, 120},
 {"green", 120, 180},
 {"cyan", 180, 240},
 {"blue", 240, 300},
 {"magenta", 300, 360},\
};


#endif  /*APDS9960_H*/

