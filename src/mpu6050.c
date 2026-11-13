/*
 * Driver minimal MPU6050 : configuration par registres et lecture des mesures.
 * Référence : MPU-6000/MPU-6050 Register Map and Descriptions (TDK InvenSense).
 */

#include "mpu6050.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/* Adresse I2C 7 bits (broche AD0 à la masse) */
#define MPU6050_ADDR            0x68

/* Registres utilisés */
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B   /* début du bloc de 14 octets de données */
#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75

#define WHO_AM_I_VALUE          0x68

/* PWR_MGMT_1 */
#define PWR_DEVICE_RESET        0x80   /* bit 7 : remise à zéro de tous les registres */
#define PWR_WAKE_INTERNAL_OSC   0x00   /* SLEEP = 0, horloge = oscillateur interne 8 MHz */
#define PWR_WAKE_PLL_GYROX      0x01   /* SLEEP = 0, horloge = PLL du gyroscope X */

/* Fréquence d'échantillonnage : 1 kHz / (1 + 19) = 50 Hz (filtre DLPF actif) */
#define SMPLRT_DIV_50HZ         19

/* CONFIG : DLPF_CFG = 4 -> bande passante ~21 Hz (accél.) / ~20 Hz (gyro) */
#define CONFIG_DLPF_20HZ        0x04

/* GYRO_CONFIG : FS_SEL = 1 (bits 4:3) -> ±500 °/s */
#define GYRO_FS_500DPS          (1 << 3)
#define GYRO_SENS_LSB_PER_DPS   65.5f

/* ACCEL_CONFIG : AFS_SEL = 2 (bits 4:3) -> ±8 g */
#define ACCEL_FS_8G             (2 << 3)
#define ACCEL_SENS_LSB_PER_G    4096.0f

#define STANDARD_GRAVITY        9.80665f

/*
 * Calibration de l'accéléromètre (mesurée à la main, méthode à deux positions).
 * Pour chaque axe : mesure à +1 g (m_plus) et à -1 g (m_moins), carte immobile.
 *   offset = (m_plus + m_moins) / 2
 *   gain   = 9.80665 / ((m_plus - m_moins) / 2)
 *   valeur corrigée = (mesure - offset) * gain
 *
 * Axe Z mesuré le 06/11/2026 : m_plus = +8.22, m_moins = -11.80 m/s²
 *   -> offset = -1.79 m/s², gain = 9.80665 / 10.01 = 0.980
 * Axes X et Y : pas encore mesurés, corrections neutres.
 */
#define ACC_X_OFFSET            0.0f
#define ACC_X_GAIN              1.0f
#define ACC_Y_OFFSET            0.0f
#define ACC_Y_GAIN              1.0f
#define ACC_Z_OFFSET            (-1.79f)
#define ACC_Z_GAIN              0.980f

/* Biais du gyroscope, fixé par mpu6050_calibrate_gyro() ou mpu6050_set_gyro_bias() */
static float gyro_bias_x, gyro_bias_y, gyro_bias_z;

/* Assemble deux octets (poids fort en premier) en entier signé 16 bits */
static inline int16_t be16_to_s16(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int write_reg(const struct device *i2c, uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c, MPU6050_ADDR, reg, val);
}

/*
 * Écrit un registre puis le relit pour vérifier que la valeur a bien été prise.
 * Réessaie jusqu'à 3 fois en cas d'erreur passagère sur le bus.
 * Aucun affichage ici : le driver ne doit jamais écrire sur la console
 * (en mode SIMU, seul le thread de sortie a le droit d'écrire).
 */
#define WRITE_RETRIES 3

static int write_reg_checked(const struct device *i2c, uint8_t reg, uint8_t val)
{
	uint8_t readback;
	int ret = -EIO;

	for (int attempt = 0; attempt < WRITE_RETRIES; attempt++) {
		if (attempt > 0) {
			k_msleep(5);
		}

		ret = write_reg(i2c, reg, val);
		if (ret != 0) {
			continue;
		}

		ret = i2c_reg_read_byte(i2c, MPU6050_ADDR, reg, &readback);
		if (ret != 0) {
			continue;
		}

		if (readback == val) {
			return 0;
		}
		ret = -EIO;
	}
	return ret;
}

int mpu6050_init(const struct device *i2c)
{
	uint8_t who;
	uint8_t pwr;
	int ret;

	/* 1. Identité */
	ret = i2c_reg_read_byte(i2c, MPU6050_ADDR, REG_WHO_AM_I, &who);
	if (ret != 0) {
		return ret;
	}
	if (who != WHO_AM_I_VALUE) {
		return -ENODEV;
	}

	/* 2. Reset complet, puis attente que le capteur redémarre */
	ret = write_reg(i2c, REG_PWR_MGMT_1, PWR_DEVICE_RESET);
	if (ret != 0) {
		return ret;
	}
	k_msleep(100);

	/*
	 * 3. Réveil sur l'oscillateur interne (le capteur démarre en veille).
	 * L'oscillateur interne est stable immédiatement : on peut configurer
	 * les registres sans risque que l'interface I2C décroche.
	 */
	ret = write_reg_checked(i2c, REG_PWR_MGMT_1, PWR_WAKE_INTERNAL_OSC);
	if (ret != 0) {
		return ret;
	}
	k_msleep(10);

	/* 4. Filtre, fréquence d'échantillonnage et plages de mesure */
	ret = write_reg_checked(i2c, REG_CONFIG, CONFIG_DLPF_20HZ);
	if (ret == 0) {
		ret = write_reg_checked(i2c, REG_SMPLRT_DIV, SMPLRT_DIV_50HZ);
	}
	if (ret == 0) {
		ret = write_reg_checked(i2c, REG_GYRO_CONFIG, GYRO_FS_500DPS);
	}
	if (ret == 0) {
		ret = write_reg_checked(i2c, REG_ACCEL_CONFIG, ACCEL_FS_8G);
	}
	if (ret != 0) {
		return ret;
	}

	/*
	 * 5. Bascule en dernier sur la PLL du gyroscope X, plus stable.
	 * Le gyroscope met ~30 ms à démarrer (datasheet) et l'I2C peut ne pas
	 * répondre pendant le changement d'horloge : on attend 100 ms
	 * avant de vérifier.
	 */
	ret = write_reg(i2c, REG_PWR_MGMT_1, PWR_WAKE_PLL_GYROX);
	if (ret != 0) {
		return ret;
	}
	k_msleep(100);

	ret = i2c_reg_read_byte(i2c, MPU6050_ADDR, REG_PWR_MGMT_1, &pwr);
	if (ret != 0) {
		return ret;
	}
	return (pwr == PWR_WAKE_PLL_GYROX) ? 0 : -EIO;
}

/* Lecture convertie en unités physiques, SANS correction de calibration */
static int read_uncalibrated(const struct device *i2c, struct mpu6050_data *out)
{
	uint8_t buf[14];
	int ret;

	/* Lecture des 14 octets en une seule transaction : les 6 axes datent du même instant */
	ret = i2c_burst_read(i2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
	}

	/*
	 * Capteur redémarré (coupure d'alimentation brève) : il revient en veille
	 * avec tous ses registres de données à zéro, tout en continuant de répondre
	 * sur le bus. Une vraie mesure n'est jamais entièrement nulle (la gravité
	 * est toujours présente), donc 14 octets à zéro = donnée invalide.
	 */
	bool all_zero = true;

	for (size_t i = 0; i < sizeof(buf); i++) {
		if (buf[i] != 0) {
			all_zero = false;
			break;
		}
	}
	if (all_zero) {
		return -ENODATA;
	}

	/* Ordre : AX, AY, AZ, TEMP, GX, GY, GZ (2 octets chacun) */
	const float a_scale = STANDARD_GRAVITY / ACCEL_SENS_LSB_PER_G;

	out->ax = be16_to_s16(&buf[0]) * a_scale;
	out->ay = be16_to_s16(&buf[2]) * a_scale;
	out->az = be16_to_s16(&buf[4]) * a_scale;

	out->temp_c = be16_to_s16(&buf[6]) / 340.0f + 36.53f;

	out->gx = be16_to_s16(&buf[8])  / GYRO_SENS_LSB_PER_DPS;
	out->gy = be16_to_s16(&buf[10]) / GYRO_SENS_LSB_PER_DPS;
	out->gz = be16_to_s16(&buf[12]) / GYRO_SENS_LSB_PER_DPS;

	return 0;
}

int mpu6050_calibrate_gyro(const struct device *i2c, int samples)
{
	struct mpu6050_data d;
	float sx = 0.0f, sy = 0.0f, sz = 0.0f;
	int ret;

	if (samples <= 0) {
		return -EINVAL;
	}

	for (int i = 0; i < samples; i++) {
		ret = read_uncalibrated(i2c, &d);
		if (ret != 0) {
			return ret;
		}
		sx += d.gx;
		sy += d.gy;
		sz += d.gz;
		k_msleep(20);   /* une nouvelle mesure toutes les 20 ms (50 Hz) */
	}

	gyro_bias_x = sx / samples;
	gyro_bias_y = sy / samples;
	gyro_bias_z = sz / samples;

	return 0;
}

void mpu6050_set_gyro_bias(float bx, float by, float bz)
{
	gyro_bias_x = bx;
	gyro_bias_y = by;
	gyro_bias_z = bz;
}

void mpu6050_get_gyro_bias(float *bx, float *by, float *bz)
{
	*bx = gyro_bias_x;
	*by = gyro_bias_y;
	*bz = gyro_bias_z;
}

int mpu6050_read(const struct device *i2c, struct mpu6050_data *out)
{
	int ret = read_uncalibrated(i2c, out);

	if (ret != 0) {
		return ret;
	}

	/* Accéléromètre : correction d'offset et de gain (calibration deux positions) */
	out->ax = (out->ax - ACC_X_OFFSET) * ACC_X_GAIN;
	out->ay = (out->ay - ACC_Y_OFFSET) * ACC_Y_GAIN;
	out->az = (out->az - ACC_Z_OFFSET) * ACC_Z_GAIN;

	/* Gyroscope : soustraction du biais mesuré au démarrage */
	out->gx -= gyro_bias_x;
	out->gy -= gyro_bias_y;
	out->gz -= gyro_bias_z;

	return 0;
}
