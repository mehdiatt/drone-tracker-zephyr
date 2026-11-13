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

/*
 * Mesure le biais du gyroscope : moyenne de `samples` lectures, une toutes
 * les 20 ms. La carte doit rester IMMOBILE pendant la mesure.
 * Retourne 0 si OK.
 */
int mpu6050_calibrate_gyro(const struct device *i2c, int samples);

/* Fixe le biais du gyroscope (°/s), soustrait ensuite par mpu6050_read() */
void mpu6050_set_gyro_bias(float bx, float by, float bz);

/* Renvoie le biais du gyroscope actuellement appliqué, en °/s */
void mpu6050_get_gyro_bias(float *bx, float *by, float *bz);

/*
 * Lit les 6 axes en une seule transaction, les convertit en unités physiques
 * et applique les corrections de calibration. Retourne 0 si OK,
 * -ENODATA si le capteur a redémarré (données toutes nulles), ou l'erreur I2C.
 */
int mpu6050_read(const struct device *i2c, struct mpu6050_data *out);

#endif /* MPU6050_H */
