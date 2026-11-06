/*
 * Driver minimal MPU6050 (accéléromètre + gyroscope) sur I2C.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <zephyr/device.h>

/* Mesures converties en unités physiques */
struct mpu6050_data {
	float ax, ay, az;   /* accélération, m/s² */
	float gx, gy, gz;   /* vitesse angulaire, °/s */
	float temp_c;       /* température interne de la puce, °C */
};

/* Vérifie l'identité, réveille et configure le capteur. Retourne 0 si OK. */
int mpu6050_init(const struct device *i2c);

/* Lit les 6 axes en une seule transaction et les convertit. Retourne 0 si OK. */
int mpu6050_read(const struct device *i2c, struct mpu6050_data *out);

#endif /* MPU6050_H */
