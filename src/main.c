/*
 * Test du thread IMU : le thread mesure à 50 Hz et publie dans telemetry ;
 * main se contente d'afficher une copie de la télémétrie toutes les 500 ms.
 *
 * LED : jaune = calibration en cours, vert = calibré, rouge = IMU en panne.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "imu.h"
#include "mpu6050.h"
#include "rgb_led.h"
#include "telemetry.h"

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

/* Affiche une valeur avec signe et deux décimales (ex. +009.81) */
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
	struct telemetry t;
	int ret;

	printk("\n=== Test thread IMU (50 Hz) ===\n");

	if (rgb_led_init() != 0) {
		printk("Erreur : LED RGB\n");
	}

	if (!device_is_ready(i2c)) {
		printk("Erreur : I2C2 n'est pas pret\n");
		rgb_led_set(RGB_RED);
		return 0;
	}

	ret = mpu6050_init(i2c);
	if (ret != 0) {
		printk("Erreur : initialisation MPU6050 (err %d)\n", ret);
		rgb_led_set(RGB_RED);
		return 0;
	}

	ret = imu_start(i2c);
	if (ret != 0) {
		printk("Erreur : demarrage du thread IMU (err %d)\n", ret);
		return 0;
	}
	printk("Calibration : laissez la carte immobile...\n");

	while (1) {
		k_sleep(K_MSEC(500));
		telemetry_snapshot(&t);

		if (t.imu.flags & FLAG_IMU_INVALID) {
			rgb_led_set(RGB_RED);
		} else if (!t.imu.calibrated) {
			rgb_led_set(RGB_YELLOW);
		} else {
			rgb_led_set(RGB_GREEN);
		}

		printk("ATT deg: roll=");  print_2dec(t.imu.roll);
		printk(" pitch=");         print_2dec(t.imu.pitch);
		printk(" yaw=");           print_2dec(t.imu.yaw);
		printk("\nGYR deg/s: gx=");  print_2dec(t.imu.gx);
		printk(" gy=");            print_2dec(t.imu.gy);
		printk(" gz=");            print_2dec(t.imu.gz);
		printk("\nIMU: %s  flags=%04X  mesures=%u  erreurs=%u  reinit=%u\n---\n",
		       t.imu.calibrated ? "calibre" : "calibration...",
		       t.flags, t.imu.samples, t.imu.read_errors, t.imu.reinits);
	}

	return 0;
}
