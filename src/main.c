/*
 * Test : thread IMU (attitude, vitesse, position, état) + thread environnement.
 * main affiche une copie de la télémétrie toutes les 500 ms.
 *
 * LED (NM6) : vert = IDLE, jaune = FLYING, rouge = PROBLEM.
 * Pendant la calibration : bleu.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "env.h"
#include "imu.h"
#include "mpu6050.h"
#include "rgb_led.h"
#include "telemetry.h"

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

static const char *const state_names[] = { "IDLE", "FLYING", "PROBLEM" };

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

static void print_xyz(const char *label, float x, float y, float z)
{
	printk("%s x=", label); print_2dec(x);
	printk(" y=");          print_2dec(y);
	printk(" z=");          print_2dec(z);
	printk("\n");
}

int main(void)
{
	struct telemetry t;
	int ret;

	printk("\n=== Test IMU + etat + environnement ===\n");

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

	imu_start(i2c);
	env_start(i2c);
	printk("Calibration : laissez la carte immobile et a plat (~7 s)...\n");

	while (1) {
		k_sleep(K_MSEC(500));
		telemetry_snapshot(&t);

		if (!t.imu.calibrated) {
			rgb_led_set(RGB_BLUE);
		} else if (t.state == STATE_PROBLEM) {
			rgb_led_set(RGB_RED);
		} else if (t.state == STATE_FLYING) {
			rgb_led_set(RGB_YELLOW);
		} else {
			rgb_led_set(RGB_GREEN);
		}

		printk("STATE: %s  %s  flags=%04X\n", state_names[t.state],
		       t.imu.calibrated ? "calibre" : "calibration...", t.flags);
		printk("ATT deg: roll=");  print_2dec(t.imu.roll);
		printk(" pitch=");         print_2dec(t.imu.pitch);
		printk(" yaw=");           print_2dec(t.imu.yaw);
		printk("\n");
		print_xyz("POS m:  ", t.imu.x, t.imu.y, t.imu.z);
		print_xyz("VEL m/s:", t.imu.vx, t.imu.vy, t.imu.vz);
		printk("ENV: T=");         print_2dec(t.env.temp_c);
		printk(" C  RH=");         print_2dec(t.env.rh_percent);
		printk(" %%  (%s)\n", (t.flags & FLAG_SI7021_INVALID) ? "invalide" : "ok");
		printk("---\n");
	}

	return 0;
}
