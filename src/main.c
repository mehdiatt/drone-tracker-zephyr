#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

// LED from the board devicetree (standard alias: led0 for LED1)
static const struct gpio_dt_spec led_blinky = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    printk("Embedded Platforms and Communications for IoT\n");
    printk("        ETSIST - UPM - MUIoT 2026-2027       \n\n");
	
    printk("    Board LED toggle (single thread: main)   \n");

    // Ensure the GPIO device is ready
    if (!device_is_ready(led_blinky.port)) {
        printk("Error: LED device not ready\n");
        return 0; 
    }

    // Configure as output and start LOW (off)
    if (gpio_pin_configure_dt(&led_blinky, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Error: configuring led_blinky\n");
        return 0;
    }

    // Toggle loop: change state every 1 second
    while (1) {
        printk("Toggle!!\n");
        gpio_pin_toggle_dt(&led_blinky);
        k_sleep(K_SECONDS(3));
    }
}
