/*
 * Driver minimal MPU6050 : configuration par registres et lecture des mesures.
 * Référence : MPU-6000/MPU-6050 Register Map and Descriptions (TDK InvenSense).
 */

#include "mpu6050.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

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
 * Réessaie jusqu'à 3 fois en cas d'erreur sur le bus.
 */
#define WRITE_RETRIES 3

static int write_reg_checked(const struct device *i2c, uint8_t reg, uint8_t val)
{
	uint8_t readback;
	int ret = 0;

	for (int attempt = 1; attempt <= WRITE_RETRIES; attempt++) {
		ret = write_reg(i2c, reg, val);
		if (ret != 0) {
			/* DEBUG : à retirer une fois l'initialisation validée */
			printk("  reg 0x%02X : ecriture refusee (essai %d, err %d)\n",
			       reg, attempt, ret);
			k_msleep(5);
			continue;
		}

		ret = i2c_reg_read_byte(i2c, MPU6050_ADDR, reg, &readback);
		if (ret != 0) {
			printk("  reg 0x%02X : relecture refusee (essai %d, err %d)\n",
			       reg, attempt, ret);
			k_msleep(5);
			continue;
		}

		if (readback != val) {
			printk("  reg 0x%02X : ecrit 0x%02X, relu 0x%02X (essai %d)\n",
			       reg, val, readback, attempt);
			ret = -EIO;
			k_msleep(5);
			continue;
		}

		return 0;
	}
	return ret;
}

/* DEBUG : affiche l'étape qui échoue (à retirer une fois validé) */
#define STEP(n, call)                                             \
	do {                                                      \
		ret = (call);                                     \
		if (ret != 0) {                                   \
			printk("MPU6050 init : echec etape %d (err %d)\n", \
			       (n), ret);                         \
			return ret;                               \
		}                                                 \
	} while (0)

int mpu6050_init(const struct device *i2c)
{
	uint8_t who;
	int ret;

	/* DEBUG : confirme que c'est bien cette version qui a été flashée */
	printk("MPU6050 init (driver v3 : retries + diagnostic)\n");

	/* 1. Identité */
	STEP(1, i2c_reg_read_byte(i2c, MPU6050_ADDR, REG_WHO_AM_I, &who));
	if (who != WHO_AM_I_VALUE) {
		printk("MPU6050 init : WHO_AM_I = 0x%02X\n", who);
		return -ENODEV;
	}

	/* 2. Reset complet, puis attente que le capteur redémarre */
	STEP(2, write_reg(i2c, REG_PWR_MGMT_1, PWR_DEVICE_RESET));
	k_msleep(100);

	/*
	 * 3. Réveil sur l'oscillateur interne (le capteur démarre en veille).
	 * L'oscillateur interne est stable immédiatement : on peut configurer
	 * les registres sans risque que l'interface I2C décroche.
	 */
	STEP(3, write_reg_checked(i2c, REG_PWR_MGMT_1, PWR_WAKE_INTERNAL_OSC));
	k_msleep(10);

	/* 4. Filtre, fréquence d'échantillonnage et plages de mesure */
	STEP(4, write_reg_checked(i2c, REG_CONFIG, CONFIG_DLPF_20HZ));
	STEP(5, write_reg_checked(i2c, REG_SMPLRT_DIV, SMPLRT_DIV_50HZ));
	STEP(6, write_reg_checked(i2c, REG_GYRO_CONFIG, GYRO_FS_500DPS));
	STEP(7, write_reg_checked(i2c, REG_ACCEL_CONFIG, ACCEL_FS_8G));

	/*
	 * 8. Bascule en dernier sur la PLL du gyroscope X, plus stable.
	 * Le gyroscope met ~30 ms à démarrer (datasheet) et l'I2C peut ne pas
	 * répondre pendant le changement d'horloge : on attend 100 ms
	 * avant de vérifier.
	 */
	STEP(8, write_reg(i2c, REG_PWR_MGMT_1, PWR_WAKE_PLL_GYROX));
	k_msleep(100);

	uint8_t pwr;

	STEP(9, i2c_reg_read_byte(i2c, MPU6050_ADDR, REG_PWR_MGMT_1, &pwr));
	if (pwr != PWR_WAKE_PLL_GYROX) {
		printk("  reg 0x%02X : ecrit 0x%02X, relu 0x%02X\n",
		       REG_PWR_MGMT_1, PWR_WAKE_PLL_GYROX, pwr);
		return -EIO;
	}

	return 0;
}

int mpu6050_read(const struct device *i2c, struct mpu6050_data *out)
{
	uint8_t buf[14];
	int ret;

	/* Lecture des 14 octets en une seule transaction : les 6 axes datent du même instant */
	ret = i2c_burst_read(i2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
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
