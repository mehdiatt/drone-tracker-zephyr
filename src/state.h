/*
 * Détection de l'état du tracker : IDLE, FLYING ou PROBLEM (SR7, NM5).
 * Logique pure, sans accès au matériel : appelée à chaque mesure IMU (50 Hz).
 */

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"

/* Ce que le thread IMU fournit à chaque mesure */
struct state_inputs {
	bool imu_valid;            /* le MPU6050 répond et ses données sont valides */
	bool env_valid;            /* le Si7021 répond (flag 0x0008 absent) */
	float a_norm;              /* |a| en m/s² */
	float g_ref;               /* g mesuré au repos pendant la calibration, m/s² */
	float gyro_norm;           /* |ω| en °/s, biais retiré */
	float roll, pitch;         /* degrés */
};

/* Mémoire du détecteur entre deux mesures */
struct state_detector {
	int moving_count;          /* mesures consécutives en mouvement */
	int64_t last_motion_ms;    /* dernière mesure en mouvement */
	bool flying;
	int free_fall_count;       /* mesures consécutives en chute libre */
	int64_t problem_until_ms;  /* fin du maintien en PROBLEM après choc/chute */
	int tilt_count;            /* mesures consécutives trop inclinées */
	int64_t env_invalid_since_ms;
	bool env_was_invalid;
};

/* Résultat d'une mise à jour */
struct state_output {
	enum tracker_state state;
	bool impact;               /* -> flag 0x0010 */
};

void state_detector_init(struct state_detector *det);

struct state_output state_detector_update(struct state_detector *det,
					  const struct state_inputs *in,
					  int64_t now_ms);

#endif /* STATE_H */
