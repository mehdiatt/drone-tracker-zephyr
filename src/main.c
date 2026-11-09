
/*
 * MPU6050 test: sensor configuration and converted measurements
 * displayed every 500 ms.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "mpu6050.h"
#include "rgb_led.h"
#include "ldr.h"

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

/*
 * Print floating-point values using fixed-point notation
 * with a sign and two decimal places (e.g. +009.81).
 */
static void print_2dec(float v)
{
    int x = (int)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    char sign = (x < 0) ? '-' : '+';

    if (x < 0) {
        x = -x;
    }

    printk("%c%03d.%02d", sign, x / 100, x % 100);
}

int main(void)
{
    struct mpu6050_data d;
    int ret;
int ldr_ready = 0;

    printk("\r\n=== APPLICATION STARTED ===\r\n");
/* Initialize LDR sensor */
ret = ldr_init();

if (ret == 0) {
    ldr_ready = 1;
    printk("LDR initialized successfully\n");
} else {
    printk("LDR initialization failed (err %d)\n", ret);
}
/* Read LDR sensor */
if (ldr_ready) {
    int ldr_raw = ldr_read_raw();

    if (ldr_raw < 0) {
        printk("LDR reading error: %d\n", ldr_raw);
    } else {
        int light_percent = ldr_raw_to_percent(ldr_raw);

        printk("LDR: raw=%d | Light=%d%%\n",
               ldr_raw, light_percent);
    }
}
    /* Initialize RGB LED */
    ret = rgb_led_init();
    printk("RGB initialization result: %d\r\n", ret);
    if (ret != 0) {
        printk("Error: RGB LED initialization failed (err %d)\n", ret);
    } else {
        rgb_led_set(RGB_GREEN);
        printk("RGB LED initialized successfully\n");
    }
    /* Check I2C2 readiness */
    if (!device_is_ready(i2c)) {
        printk("Error: I2C2 is not ready\n");
        rgb_led_set(RGB_RED);
        return 0;
    }

    /* Initialize MPU6050 */
    ret = mpu6050_init(i2c);

    if (ret != 0) {
        printk("Error: MPU6050 initialization failed (err %d)\n", ret);
        rgb_led_set(RGB_RED);
        return 0;
    }

    printk("\n=== MPU6050 configured: +-8 g, +-500 deg/s, 50 Hz, DLPF 20 Hz ===\n");

    /* Gyroscope calibration */
    printk("Gyroscope calibration: keep the board still (2 seconds)...\n");

    ret = mpu6050_calibrate_gyro(i2c, 100);

    if (ret != 0) {
        printk("Error: Gyroscope calibration failed (err %d)\n", ret);
        rgb_led_set(RGB_RED);
        return 0;
    }

    float bx, by, bz;

    mpu6050_get_gyro_bias(&bx, &by, &bz);

    printk("Gyroscope bias (deg/s): bx=");
    print_2dec(bx);
    printk(" by=");
    print_2dec(by);
    printk(" bz=");
    print_2dec(bz);
    printk("\n\n");
	printk("---\n");

    /* Main sensor reading loop */
    while (1) {
        ret = mpu6050_read(i2c, &d);

        if (ret != 0) {
            printk("Sensor reading error (err %d)\n", ret);
            rgb_led_set(RGB_RED);
        } else {
            float a_norm = sqrtf(
                d.ax * d.ax +
                d.ay * d.ay +
                d.az * d.az
            );

            rgb_led_set(RGB_GREEN);

            printk("ACC (m/s2): ax=");
            print_2dec(d.ax);
            printk(" ay=");
            print_2dec(d.ay);
            printk(" az=");
            print_2dec(d.az);
            printk(" |a|=");
            print_2dec(a_norm);

            printk("\nGYR (deg/s): gx=");
            print_2dec(d.gx);
            printk(" gy=");
            print_2dec(d.gy);
            printk(" gz=");
            print_2dec(d.gz);

            printk("   T=");
print_2dec(d.temp_c);
printk(" C\n");

/* Read LDR continuously */
if (ldr_ready) {
    int ldr_raw = ldr_read_raw();

    if (ldr_raw < 0) {
        printk("LDR reading error: %d\n", ldr_raw);
    } else {
        int light_percent = ldr_raw_to_percent(ldr_raw);

        printk("LDR: raw=%d | Light=%d%%\n",
               ldr_raw, light_percent);
    }
}

printk("---\n");
        }

        k_sleep(K_MSEC(500));
    }

    return 0;
}
