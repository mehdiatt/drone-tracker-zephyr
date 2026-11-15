
#include "mode.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

/* Bouton B1 */
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static struct gpio_callback button_callback;

/* Variable unique des modes */
static atomic_t current_mode = ATOMIC_INIT(MODE_TEST);

/* Anti-rebond */
#define BUTTON_DEBOUNCE_MS 200

static uint32_t last_press_time;
static bool first_press = true;

/* Lecture du mode */
enum op_mode mode_get(void)
{
    return (enum op_mode)atomic_get(&current_mode);
}

/* Définition du mode */
void mode_set(enum op_mode mode)
{
    if (mode >= MODE_TEST && mode <= MODE_SIMU) {
        atomic_set(&current_mode, (atomic_val_t)mode);
    }
}

/* Passage au mode suivant */
enum op_mode mode_next(void)
{
    atomic_val_t old_mode;
    atomic_val_t next_mode;

    do {
        old_mode = atomic_get(&current_mode);

        next_mode = old_mode + 1;

        if (next_mode > MODE_SIMU) {
            next_mode = MODE_TEST;
        }

    } while (!atomic_cas(&current_mode, old_mode, next_mode));

    return (enum op_mode)next_mode;
}

/* Interruption du bouton B1 */
static void button_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    uint32_t now = k_uptime_get_32();

    if (!first_press &&
        (uint32_t)(now - last_press_time) < BUTTON_DEBOUNCE_MS) {
        return;
    }

    first_press = false;
    last_press_time = now;

    /* Changement atomique du mode */
    mode_next();
}

/* Initialisation du bouton */
int mode_button_init(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }

    gpio_init_callback(&button_callback,
                       button_isr,
                       BIT(button.pin));

    ret = gpio_add_callback(button.port, &button_callback);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(
        &button,
        GPIO_INT_EDGE_TO_ACTIVE
    );

    if (ret < 0) {
        gpio_remove_callback(button.port, &button_callback);
        return ret;
    }

    mode_set(MODE_TEST);
    first_press = true;
    last_press_time = 0;

    return 0;
}

/* Nom du mode */
const char *mode_name(enum op_mode mode)
{
    switch (mode) {
    case MODE_TEST:
        return "TEST";

    case MODE_NORMAL:
        return "NORMAL";

    case MODE_SIMU:
        return "SIMU";

    default:
        return "UNKNOWN";
    }
}
