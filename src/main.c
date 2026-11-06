/*
 * Test MPU6050 : configuration, puis affichage des mesures converties
 * toutes les 500 ms.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "mpu6050.h"

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

/*
 * printk ne gère pas les flottants : on affiche la valeur en virgule fixe,
 * avec signe et deux décimales (ex. +009.81).
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

	if (!device_is_ready(i2c)) {
		printk("Erreur : I2C2 n'est pas pret\n");
		return 0;
	}

	ret = mpu6050_init(i2c);
	if (ret != 0) {
		printk("Erreur : initialisation MPU6050 (err %d)\n", ret);
		return 0;
	}
	printk("\n=== MPU6050 configure : +-8 g, +-500 deg/s, 50 Hz, DLPF 20 Hz ===\n");

	while (1) {
		ret = mpu6050_read(i2c, &d);
		if (ret != 0) {
			printk("Erreur de lecture (err %d)\n", ret);
		} else {
			float a_norm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);

			printk("ACC m/s2: ax=");  print_2dec(d.ax);
			printk(" ay=");           print_2dec(d.ay);
			printk(" az=");           print_2dec(d.az);
			printk(" |a|=");          print_2dec(a_norm);
			printk("\nGYR deg/s: gx="); print_2dec(d.gx);
			printk(" gy=");           print_2dec(d.gy);
			printk(" gz=");           print_2dec(d.gz);
			printk("   T=");          print_2dec(d.temp_c);
			printk(" C\n---\n");
		}
		k_sleep(K_MSEC(500));
	}

	return 0;
}
