/*
 * Driver minimal Si7021 : humidité relative et température.
 * Référence : Silicon Labs, Si7021-A20 datasheet (rev. 1.2).
 *
 * Choix principal : mode "No Hold Master". En mode "Hold Master", le capteur
 * maintient SCL à l'état bas pendant toute la conversion (jusqu'à ~23 ms) et
 * bloque le bus I2C, donc aussi le MPU6050 qui doit être lu toutes les 20 ms.
 * En "No Hold", on lance la mesure, on libère le bus, et on revient lire.
 */

#include "si7021.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/* Adresse I2C 7 bits (fixe) */
#define SI7021_ADDR                 0x40

/* Commandes */
#define CMD_MEASURE_RH_NO_HOLD      0xF5   /* lance une mesure d'humidité */
#define CMD_READ_TEMP_FROM_RH       0xE0   /* température issue de la mesure RH */
#define CMD_RESET                   0xFE   /* reset logiciel */
#define CMD_READ_USER_REG1          0xE7

/* Registre utilisateur après reset : RH 12 bits / T 14 bits, chauffage off */
#define USER_REG1_RESET_VALUE       0x3A

/*
 * Temps de conversion max (datasheet, tableau 2) : RH 12 bits = 12 ms,
 * plus la température 14 bits faite automatiquement = 10,8 ms.
 * Total ~23 ms : on attend 25 ms, puis on réessaie si le capteur
 * n'a pas fini (il répond alors NACK à la lecture).
 */
#define CONVERSION_WAIT_MS          25
#define READ_RETRIES                5
#define READ_RETRY_DELAY_MS         5

/* Après un reset, le capteur est prêt en 15 ms max */
#define RESET_DELAY_MS              15

/*
 * CRC-8 de la datasheet : polynôme x^8 + x^5 + x^4 + 1 (0x31),
 * valeur initiale 0x00, calculé sur les 2 octets de la mesure.
 */
static uint8_t crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

int si7021_init(const struct device *i2c)
{
	uint8_t cmd = CMD_RESET;
	uint8_t user;
	int ret;

	ret = i2c_write(i2c, &cmd, 1, SI7021_ADDR);
	if (ret != 0) {
		return ret;
	}
	k_msleep(RESET_DELAY_MS);

	/* Vérifie que le capteur répond et qu'il est bien revenu à sa config par défaut */
	cmd = CMD_READ_USER_REG1;
	ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, &user, 1);
	if (ret != 0) {
		return ret;
	}
	return (user == USER_REG1_RESET_VALUE) ? 0 : -ENODEV;
}

int si7021_read(const struct device *i2c, struct si7021_data *out)
{
	uint8_t cmd = CMD_MEASURE_RH_NO_HOLD;
	uint8_t buf[3];   /* MSB, LSB, CRC */
	int ret;

	/* 1. Lance la mesure, puis libère le bus pendant la conversion */
	ret = i2c_write(i2c, &cmd, 1, SI7021_ADDR);
	if (ret != 0) {
		return ret;
	}
	k_msleep(CONVERSION_WAIT_MS);

	/* 2. Lecture du résultat : tant que la conversion n'est pas finie, le capteur répond NACK */
	for (int attempt = 0; attempt < READ_RETRIES; attempt++) {
		ret = i2c_read(i2c, buf, sizeof(buf), SI7021_ADDR);
		if (ret == 0) {
			break;
		}
		k_msleep(READ_RETRY_DELAY_MS);
	}
	if (ret != 0) {
		return ret;
	}

	/* 3. Contrôle d'intégrité de la donnée reçue */
	if (crc8(buf, 2) != buf[2]) {
		return -EBADMSG;
	}

	uint16_t rh_code = ((uint16_t)buf[0] << 8) | buf[1];

	/* 4. Température mesurée pendant la mesure d'humidité : pas de nouvelle conversion */
	cmd = CMD_READ_TEMP_FROM_RH;
	ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, buf, 2);
	if (ret != 0) {
		return ret;
	}
	uint16_t t_code = ((uint16_t)buf[0] << 8) | buf[1];

	/* 5. Conversions (datasheet, sections 5.1.1 et 5.1.2) */
	float rh = 125.0f * rh_code / 65536.0f - 6.0f;

	/* La formule peut donner un peu moins de 0 ou plus de 100 : on borne */
	if (rh < 0.0f) {
		rh = 0.0f;
	} else if (rh > 100.0f) {
		rh = 100.0f;
	}

	out->rh_percent = rh;
	out->temp_c = 175.72f * t_code / 65536.0f - 46.85f;

	return 0;
}
