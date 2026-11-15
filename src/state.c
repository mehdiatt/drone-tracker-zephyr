/*
 * Détection de l'état du tracker (SR7, NM5).
 *
 * PROBLEM est prioritaire (SR7). Il est déclenché par :
 *   - IMU invalide (MPU6050 absent ou redémarré) : capteur principal perdu ;
 *   - choc : |a| > 2,5 g, maintenu 5 s (flag 0x0010) ;
 *   - chute libre : |a| < 0,3 g pendant 200 ms, maintenu 5 s (flag 0x0010) ;
 *   - inclinaison anormale : |roll| ou |pitch| > 60° pendant 2 s (drone retourné) ;
 *   - Si7021 en erreur depuis plus de 10 s (erreur capteur persistante).
 * L'absence de fix GPS ne déclenche PAS PROBLEM (NM5). Le LDR non plus :
 * une saturation au soleil n'est pas une panne.
 *
 * FLYING / IDLE, avec hystérésis pour éviter les clignotements :
 *   - mesure "en mouvement" si |ω| > 10 °/s ou si |a| s'écarte de g de plus
 *     de 1 m/s². Au repos, on a mesuré |ω| < 0,3 °/s et |a| stable à
 *     ±0,1 m/s² : ces seuils laissent une marge d'environ 10 ;
 *   - FLYING après 5 mesures consécutives en mouvement (100 ms) ;
 *   - retour en IDLE après 3 s sans aucun mouvement.
 */

#include "state.h"

#include <math.h>

#define G                       9.80665f

#define IMPACT_ACC              (2.5f * G)   /* m/s² */
#define FREE_FALL_ACC           (0.3f * G)   /* m/s² */
#define FREE_FALL_SAMPLES       10           /* 200 ms à 50 Hz */
#define PROBLEM_HOLD_MS         5000

#define TILT_LIMIT_DEG          60.0f
#define TILT_SAMPLES            100          /* 2 s à 50 Hz */

#define ENV_INVALID_LIMIT_MS    10000

#define MOVING_GYRO_DPS         10.0f
#define MOVING_ACC_DEV          1.0f         /* m/s² */
#define MOVING_SAMPLES          5            /* 100 ms à 50 Hz */
#define IDLE_AFTER_MS           3000

void state_detector_init(struct state_detector *det)
{
	det->moving_count = 0;
	det->last_motion_ms = 0;
	det->flying = false;
	det->free_fall_count = 0;
	det->problem_until_ms = 0;
	det->tilt_count = 0;
	det->env_invalid_since_ms = 0;
	det->env_was_invalid = false;
}

struct state_output state_detector_update(struct state_detector *det,
					  const struct state_inputs *in,
					  int64_t now_ms)
{
	struct state_output out = { .state = STATE_IDLE, .impact = false };
	bool problem = false;

	/* --- Erreur capteur persistante : Si7021 --- */
	if (!in->env_valid) {
		if (!det->env_was_invalid) {
			det->env_was_invalid = true;
			det->env_invalid_since_ms = now_ms;
		}
		if (now_ms - det->env_invalid_since_ms >= ENV_INVALID_LIMIT_MS) {
			problem = true;
		}
	} else {
		det->env_was_invalid = false;
	}

	/* --- IMU perdue : on ne peut plus rien estimer du mouvement --- */
	if (!in->imu_valid) {
		det->moving_count = 0;
		det->free_fall_count = 0;
		det->tilt_count = 0;
		out.impact = now_ms < det->problem_until_ms;
		out.state = STATE_PROBLEM;
		return out;
	}

	/* --- Choc et chute libre (maintenus 5 s) --- */
	if (in->a_norm > IMPACT_ACC) {
		det->problem_until_ms = now_ms + PROBLEM_HOLD_MS;
	}

	det->free_fall_count = (in->a_norm < FREE_FALL_ACC) ? det->free_fall_count + 1 : 0;
	if (det->free_fall_count >= FREE_FALL_SAMPLES) {
		det->problem_until_ms = now_ms + PROBLEM_HOLD_MS;
	}

	if (now_ms < det->problem_until_ms) {
		out.impact = true;
		problem = true;
	}

	/* --- Inclinaison anormale persistante --- */
	bool tilted = fabsf(in->roll) > TILT_LIMIT_DEG || fabsf(in->pitch) > TILT_LIMIT_DEG;

	det->tilt_count = tilted ? det->tilt_count + 1 : 0;
	if (det->tilt_count >= TILT_SAMPLES) {
		problem = true;
	}

	/* --- Mouvement : FLYING / IDLE avec hystérésis --- */
	bool moving = in->gyro_norm > MOVING_GYRO_DPS ||
		      fabsf(in->a_norm - in->g_ref) > MOVING_ACC_DEV;

	if (moving) {
		det->moving_count++;
		det->last_motion_ms = now_ms;
		if (det->moving_count >= MOVING_SAMPLES) {
			det->flying = true;
		}
	} else {
		det->moving_count = 0;
		if (det->flying && now_ms - det->last_motion_ms >= IDLE_AFTER_MS) {
			det->flying = false;
		}
	}

	if (problem) {
		out.state = STATE_PROBLEM;   /* prioritaire (SR7) */
	} else {
		out.state = det->flying ? STATE_FLYING : STATE_IDLE;
	}
	return out;
}
