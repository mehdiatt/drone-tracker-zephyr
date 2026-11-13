/*
 * Thread environnement : lit le Si7021 toutes les 2 s et publie dans telemetry.
 *
 * Le Si7021 partage le bus I2C2 avec le MPU6050. Le driver I2C de Zephyr
 * sérialise les transactions, et le mode "No Hold" du Si7021 libère le bus
 * pendant ses ~25 ms de conversion : le thread IMU n'est donc jamais bloqué.
 *
 * Robustesse (SM5) : en cas d'erreur, flag 0x0008 et dernières valeurs
 * conservées ; le capteur est réinitialisé à la période suivante.
 */

#include "env.h"
#include "si7021.h"
#include "telemetry.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#define ENV_STACK_SIZE  1024
#define ENV_PRIORITY    6      /* plus basse que l'IMU : mesures lentes */

K_THREAD_STACK_DEFINE(env_stack, ENV_STACK_SIZE);
static struct k_thread env_thread_data;

static const struct device *env_i2c;

static void env_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct env_telemetry st = { .flags = FLAG_SI7021_INVALID };
	struct si7021_data d;
	bool sensor_ready = false;

	while (1) {
		if (!sensor_ready) {
			sensor_ready = (si7021_init(env_i2c) == 0);
		}

		if (sensor_ready && si7021_read(env_i2c, &d) == 0) {
			st.temp_c = d.temp_c;
			st.rh_percent = d.rh_percent;
			st.flags &= ~FLAG_SI7021_INVALID;
		} else {
			/* Dernières valeurs valides conservées ; réinit au prochain tour */
			st.flags |= FLAG_SI7021_INVALID;
			sensor_ready = false;
		}

		telemetry_set_env(&st);
		k_sleep(K_MSEC(ENV_PERIOD_MS));
	}
}

int env_start(const struct device *i2c)
{
	if (i2c == NULL) {
		return -EINVAL;
	}
	env_i2c = i2c;

	k_thread_create(&env_thread_data, env_stack,
			K_THREAD_STACK_SIZEOF(env_stack),
			env_thread, NULL, NULL, NULL,
			ENV_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&env_thread_data, "env");

	return 0;
}
