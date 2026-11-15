/*
 * Drone Tracking & Telemetry node - NUCLEO-WL55JC / Zephyr
 *
 * main initialise le matériel et démarre les threads, puis se termine :
 *   imu    (prio 2) : MPU6050 à 50 Hz, attitude, vitesse/position, état
 *   output (prio 4) : sortie TEST / NORMAL / SIMU, LED
 *   gps    (prio 5) : réception NMEA par interruption UART
 *   env    (prio 6) : Si7021 toutes les 2 s
 * Le mode courant (mode.c) est changé par le bouton B1 (rôle B).
 *
 * Un capteur absent au démarrage ne bloque pas le système : son flag est
 * levé et les trames continuent d'être envoyées (SM5).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include "env.h"
#include "gps.h"
#include "imu.h"
#include "mode.h"
#include "mpu6050.h"
#include "output.h"
#include "rgb_led.h"

/*
 * Mode au démarrage. La spec impose TEST.
 * Pour tester le simulateur Python sans le bouton B1, mettre MODE_SIMU.
 */
#define START_MODE MODE_TEST

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

int main(void)
{
	/* En SIMU, aucun texte en dehors des trames (GR5) */
	const bool verbose = (START_MODE != MODE_SIMU);
/* Bouton B1 : TEST -> NORMAL -> SIMU -> TEST */
	if (mode_button_init() != 0 && verbose) {
		printk("Avertissement : bouton B1 non disponible\n");
	}
	mode_set(START_MODE);

	if (verbose) {
		printk("\n=== Drone tracker : demarrage (mode %s) ===\n",
		       mode_name(START_MODE));
	}

	if (rgb_led_init() != 0 && verbose) {
		printk("Avertissement : LED RGB non disponible\n");
	}

	if (!device_is_ready(i2c)) {
		if (verbose) {
			printk("Erreur : bus I2C2 non pret, IMU et Si7021 indisponibles\n");
		}
	} else {
		/* Si le MPU6050 ne répond pas, le thread IMU le réinitialise toutes les secondes */
		int ret = mpu6050_init(i2c);

		if (ret != 0 && verbose) {
			printk("Avertissement : MPU6050 absent (err %d), nouvelle tentative auto\n", ret);
		}
		imu_start(i2c);
		env_start(i2c);
	}

	gps_init();
	output_start();

	if (verbose) {
		printk("Calibration IMU : laissez la carte immobile et a plat (~7 s)\n");
	}

	return 0;   /* les threads continuent de tourner */
}
