/*
 * Driver minimal Si7021 (humidité relative + température) sur I2C.
 */

#ifndef SI7021_H
#define SI7021_H

#include <stdbool.h>
#include <zephyr/device.h>

struct si7021_data {
	float rh_percent;   /* humidité relative, %, bornée à 0..100 */
	float temp_c;       /* température, °C */
};

/* Reset logiciel, puis vérifie que le capteur répond. Retourne 0 si OK. */
int si7021_init(const struct device *i2c);

/*
 * Mesure l'humidité puis récupère la température de la même mesure.
 * Bloque environ 25 ms (temps de conversion du capteur) sans occuper
 * le bus I2C pendant ce temps. Retourne 0 si OK.
 */
int si7021_read(const struct device *i2c, struct si7021_data *out);

#endif /* SI7021_H */
