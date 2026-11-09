#ifndef RGB_LED_H
#define RGB_LED_H

enum rgb_color {
    RGB_OFF,
    RGB_RED,
    RGB_GREEN,
    RGB_BLUE,
    RGB_YELLOW,
    RGB_CYAN,
    RGB_MAGENTA,
    RGB_WHITE
};

int rgb_led_init(void);
void rgb_led_set(enum rgb_color color);

#endif