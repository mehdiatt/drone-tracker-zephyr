#include <stdbool.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "rgb_led.h"

#define RGB_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec red =
    GPIO_DT_SPEC_GET(RGB_NODE, red_gpios);

static const struct gpio_dt_spec green =
    GPIO_DT_SPEC_GET(RGB_NODE, green_gpios);

static const struct gpio_dt_spec blue =
    GPIO_DT_SPEC_GET(RGB_NODE, blue_gpios);

int rgb_led_init(void)
{
    int ret;

    if (!gpio_is_ready_dt(&red) ||
        !gpio_is_ready_dt(&green) ||
        !gpio_is_ready_dt(&blue)) {
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&red, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    ret = gpio_pin_configure_dt(&green, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    ret = gpio_pin_configure_dt(&blue, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    return 0;
}

void rgb_led_set(enum rgb_color color)
{
    bool r = false;
    bool g = false;
    bool b = false;

    switch (color) {
    case RGB_RED:     r = true; break;
    case RGB_GREEN:   g = true; break;
    case RGB_BLUE:    b = true; break;
    case RGB_YELLOW:  r = true; g = true; break;
    case RGB_CYAN:    g = true; b = true; break;
    case RGB_MAGENTA: r = true; b = true; break;
    case RGB_WHITE:   r = true; g = true; b = true; break;
    case RGB_OFF:
    default: break;
    }

    gpio_pin_set_dt(&red, r);
    gpio_pin_set_dt(&green, g);
    gpio_pin_set_dt(&blue, b);
}