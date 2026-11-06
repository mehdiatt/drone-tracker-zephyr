/*
 * Test I2C : vérifie que le MPU6050 et le Si7021 répondent sur I2C2.
 * Relit l'identité des deux capteurs toutes les 2 s.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* Adresses I2C (7 bits) */
#define MPU6050_ADDR          0x68
#define SI7021_ADDR           0x40

/* MPU6050 : registre d'identité */
#define MPU6050_REG_WHO_AM_I  0x75
#define MPU6050_WHO_AM_I_VAL  0x68

/* Si7021 : commande de lecture du registre utilisateur 1 */
#define SI7021_CMD_READ_USER  0xE7
#define SI7021_USER_RESET_VAL 0x3A

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

static void check_mpu6050(void)
{
	uint8_t who = 0;
	int ret = i2c_reg_read_byte(i2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who);

	if (ret != 0) {
		printk("MPU6050 : pas de reponse a 0x%02X (err %d)\n", MPU6050_ADDR, ret);
	} else if (who == MPU6050_WHO_AM_I_VAL) {
		printk("MPU6050 : OK, WHO_AM_I = 0x%02X\n", who);
	} else {
		printk("MPU6050 : repond, mais WHO_AM_I = 0x%02X (attendu 0x%02X)\n",
		       who, MPU6050_WHO_AM_I_VAL);
	}
}

static void check_si7021(void)
{
	uint8_t cmd = SI7021_CMD_READ_USER;
	uint8_t user = 0;
	int ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, &user, 1);

	if (ret != 0) {
		printk("Si7021  : pas de reponse a 0x%02X (err %d)\n", SI7021_ADDR, ret);
	} else if (user == SI7021_USER_RESET_VAL) {
		printk("Si7021  : OK, registre utilisateur = 0x%02X\n", user);
	} else {
		printk("Si7021  : repond, registre utilisateur = 0x%02X (0x%02X apres reset)\n",
		       user, SI7021_USER_RESET_VAL);
	}
}

int main(void)
{
	if (!device_is_ready(i2c)) {
		printk("Erreur : I2C2 n'est pas pret\n");
		return 0;
	}

	printk("\n=== Test I2C2 (D15 = SCL, D14 = SDA) ===\n");

	while (1) {
		check_mpu6050();
		check_si7021();
		printk("---\n");
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
