/*
 * Données de télémétrie partagées, protégées par un mutex.
 * Les sections critiques se limitent à une copie mémoire : quelques
 * microsecondes, donc aucun risque de retarder le thread IMU.
 */

#include "telemetry.h"

#include <zephyr/kernel.h>

K_MUTEX_DEFINE(telemetry_lock);

/*
 * Au démarrage, tout est marqué invalide : chaque section ne devient valide
 * que lorsque son propriétaire a publié une vraie mesure.
 */
static struct telemetry data = {
	.imu   = { .flags = FLAG_IMU_INVALID | FLAG_CALIB_PENDING |
			FLAG_INERTIAL_ONLY | FLAG_YAW_UNRELIABLE },
	.env   = { .flags = FLAG_SI7021_INVALID },
	.gps   = { .flags = FLAG_GPS_INVALID },
	.light = { .flags = FLAG_LDR_INVALID },
	.state = STATE_IDLE,
};

void telemetry_set_imu(const struct imu_telemetry *imu)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	data.imu = *imu;
	k_mutex_unlock(&telemetry_lock);
}

void telemetry_set_env(const struct env_telemetry *env)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	data.env = *env;
	k_mutex_unlock(&telemetry_lock);
}

void telemetry_set_gps(const struct gps_telemetry *gps)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	data.gps = *gps;
	k_mutex_unlock(&telemetry_lock);
}

void telemetry_set_light(const struct light_telemetry *light)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	data.light = *light;
	k_mutex_unlock(&telemetry_lock);
}

void telemetry_set_state(enum tracker_state state)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	data.state = state;
	k_mutex_unlock(&telemetry_lock);
}

void telemetry_snapshot(struct telemetry *out)
{
	k_mutex_lock(&telemetry_lock, K_FOREVER);
	*out = data;
	k_mutex_unlock(&telemetry_lock);

	out->flags = (out->imu.flags | out->env.flags | out->gps.flags |
		      out->light.flags) & FLAGS_DEFINED_MASK;
}
