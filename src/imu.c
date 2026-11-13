/*
 * Thread IMU.
 *
 * Cadence : un k_timer périodique de 20 ms réveille le thread, qui dort
 * entre deux mesures (k_timer_status_sync), donc aucun polling.
 *
 * Calibration du gyroscope : non bloquante, faite dans la boucle à 50 Hz.
 * On accumule 100 mesures (2 s). Si la carte a bougé pendant ce temps,
 * la calibration est rejetée et recommence ; le flag 0x0020 reste actif.
 *
 * Attitude (SR4) : intégration numérique d'Euler des vitesses angulaires,
 * orientation initiale nulle. On utilise la relation entre vitesses angulaires
 * du capteur (p, q, r) et dérivées des angles d'Euler (roll, pitch, yaw),
 * plutôt qu'une simple intégration axe par axe, qui n'est juste que pour
 * de petits angles. Limites (à discuter dans le rapport) : dérive accumulée
 * et absence de référence absolue de cap pour le yaw.
 *
 * Vitesse et position locales : l'accélération est ramenée dans le repère
 * local avec les angles estimés, g est retirée, puis on intègre deux fois
 * (Euler). La dérive est inévitable (flag 0x0040 toujours actif) ; deux
 * protections la limitent : une zone morte sur les petites accélérations,
 * et la remise à zéro de la vitesse quand la carte est immobile (ZUPT).
 *
 * État (IDLE / FLYING / PROBLEM) : décidé à chaque mesure par state.c.
 *
 * Robustesse (GR3) : si le MPU6050 ne répond plus ou a redémarré, le flag
 * 0x0001 est activé, les dernières valeurs valides sont conservées, et le
 * capteur est réinitialisé automatiquement toutes les secondes.
 */

#include "imu.h"
#include "mpu6050.h"
#include "state.h"
#include "telemetry.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/* ---------- Paramètres (justifiés dans les commentaires) ---------- */

#define IMU_STACK_SIZE          2048
#define IMU_PRIORITY            2      /* la plus haute de l'application */

/* Calibration : 100 mesures à 50 Hz = 2 s */
#define CALIB_SAMPLES           100

/*
 * Immobilité pendant la calibration. Au repos, le bruit mesuré du gyroscope
 * (filtre 20 Hz) varie de moins de 0,5 °/s : un écart max - min supérieur à
 * 2 °/s sur un axe signifie que la carte a bougé.
 */
#define CALIB_GYRO_SPREAD_DPS   2.0f

/* La norme de l'accélération doit rester proche de g (ici ±1 m/s²) */
#define STANDARD_GRAVITY        9.80665f
#define CALIB_ACC_TOLERANCE     1.0f

/*
 * Préchauffage : juste après sa (ré)initialisation, le biais du gyroscope
 * varie pendant que la puce chauffe (mesuré : ~0,15 °/s d'écart dans les
 * secondes qui suivent). On attend 5 s avant de calibrer.
 */
#define WARMUP_MS               5000

/*
 * Suivi du biais au repos. Si la carte est immobile (chaque axe du gyroscope
 * sous 1 °/s une fois le biais retiré, |a| à ±0,5 m/s² de g) pendant 1 s,
 * le biais est corrigé lentement (constante de temps 5 s) vers la valeur
 * mesurée. On intègre toujours le gyroscope seul (SR4) : seule l'estimation
 * de son biais s'améliore. Limite : une vraie rotation plus lente que 1 °/s
 * et très longue serait en partie prise pour du biais.
 */
#define STILL_GYRO_DPS          1.0f
#define STILL_ACC_TOLERANCE     0.5f
#define STILL_SAMPLES           50     /* 1 s à 50 Hz */
#define BIAS_TRACK_TAU_S        5.0f

/* Après 3 lectures ratées consécutives (60 ms), l'IMU est déclarée invalide */
#define ERRORS_BEFORE_INVALID   3

/* Tentative de réinitialisation du capteur toutes les secondes */
#define REINIT_PERIOD_MS        1000

/*
 * Yaw non fiable (flag 0x0080). Sans magnétomètre, rien ne corrige la dérive
 * du yaw. Avec un biais résiduel mesuré d'environ 0,1 °/s, l'erreur dépasse 5°
 * au bout de 50 s : au-delà, le yaw est signalé non fiable.
 */
#define GYRO_RESIDUAL_BIAS_DPS  0.1f
#define YAW_DRIFT_BUDGET_DEG    5.0f

/*
 * Zone morte sur l'accélération linéaire. Au repos, |a| fluctue d'environ
 * ±0,1 m/s² : en dessous de 0,2 m/s², on considère qu'il n'y a pas
 * d'accélération réelle, sinon ce bruit, intégré deux fois, fait dériver
 * la position même carte posée.
 */
#define ACC_DEADBAND            0.2f

/* Bornes des champs de la trame SIMU (sIII.dd et sIIII.dd) */
#define MAX_SPEED               999.99f
#define MAX_POSITION            9999.99f

/* Garde-fou contre le blocage de cardan (pitch proche de ±90°) */
#define MIN_COS_PITCH           0.1f

#define DEG2RAD                 0.017453293f
#define RAD2DEG                 57.29578f

/* ---------- Thread et timer ---------- */

K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread_data;
K_TIMER_DEFINE(imu_timer, NULL, NULL);

static const struct device *imu_i2c;

/* ---------- Calibration du gyroscope ---------- */

struct calib_state {
	int count;
	float sum[3];              /* gyroscope */
	float asum[3];             /* accéléromètre */
	float min[3], max[3];
	bool moved;
};

static void calib_reset(struct calib_state *c)
{
	c->count = 0;
	c->moved = false;
	for (int i = 0; i < 3; i++) {
		c->sum[i] = 0.0f;
		c->asum[i] = 0.0f;
		c->min[i] = INFINITY;
		c->max[i] = -INFINITY;
	}
}

/* Ajoute une mesure ; retourne true quand la fenêtre de calibration est pleine */
static bool calib_add(struct calib_state *c, const struct mpu6050_data *d)
{
	const float g[3] = { d->gx, d->gy, d->gz };
	const float a[3] = { d->ax, d->ay, d->az };
	float a_norm = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);

	if (fabsf(a_norm - STANDARD_GRAVITY) > CALIB_ACC_TOLERANCE) {
		c->moved = true;
	}

	for (int i = 0; i < 3; i++) {
		c->sum[i] += g[i];
		c->asum[i] += a[i];
		c->min[i] = fminf(c->min[i], g[i]);
		c->max[i] = fmaxf(c->max[i], g[i]);
		if (c->max[i] - c->min[i] > CALIB_GYRO_SPREAD_DPS) {
			c->moved = true;
		}
	}

	c->count++;
	return c->count >= CALIB_SAMPLES;
}

/* ---------- Attitude ---------- */

static float wrap_180(float angle)
{
	while (angle > 180.0f) {
		angle -= 360.0f;
	}
	while (angle <= -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

/*
 * Un pas d'intégration d'Euler des angles d'Euler (convention aéronautique
 * Z-Y-X : yaw, puis pitch, puis roll). p, q, r = vitesses angulaires autour
 * des axes X, Y, Z du capteur.
 *   roll'  = p + (q sin(roll) + r cos(roll)) tan(pitch)
 *   pitch' = q cos(roll) - r sin(roll)
 *   yaw'   = (q sin(roll) + r cos(roll)) / cos(pitch)
 */
static void attitude_step(struct imu_telemetry *s, const struct mpu6050_data *d, float dt)
{
	const float p = d->gx * DEG2RAD;
	const float q = d->gy * DEG2RAD;
	const float r = d->gz * DEG2RAD;

	const float phi = s->roll * DEG2RAD;
	const float theta = s->pitch * DEG2RAD;

	const float s_phi = sinf(phi);
	const float c_phi = cosf(phi);
	float c_theta = cosf(theta);

	if (fabsf(c_theta) < MIN_COS_PITCH) {
		c_theta = copysignf(MIN_COS_PITCH, c_theta);
	}
	const float tan_theta = sinf(theta) / c_theta;

	const float roll_dot = p + (q * s_phi + r * c_phi) * tan_theta;
	const float pitch_dot = q * c_phi - r * s_phi;
	const float yaw_dot = (q * s_phi + r * c_phi) / c_theta;

	s->roll = wrap_180(s->roll + roll_dot * dt * RAD2DEG);
	s->pitch = fmaxf(-90.0f, fminf(90.0f, s->pitch + pitch_dot * dt * RAD2DEG));
	s->yaw = wrap_180(s->yaw + yaw_dot * dt * RAD2DEG);
}

/* ---------- Vitesse et position ---------- */

/*
 * Alignement initial, mesuré pendant la calibration. SR4 suppose l'orientation
 * initiale nulle (carte à plat) : la moyenne m de l'accélération au repos
 * devrait donc valoir (0, 0, g). L'écart restant (offsets des axes X/Y non
 * calibrés, carte pas tout à fait horizontale) est retiré dans le repère du
 * capteur, et g_ref = |m| sert de gravité de référence (absorbe l'erreur
 * d'échelle). Sans cela, un offset de 0,5 m/s² sur Y ferait dériver la
 * position de 0,25 x t² mètres.
 */
static float acc_align[3];
static float g_ref = STANDARD_GRAVITY;

static float clampf(float v, float lim)
{
	return fmaxf(-lim, fminf(lim, v));
}

static float deadband(float v)
{
	return (fabsf(v) < ACC_DEADBAND) ? 0.0f : v;
}

/*
 * Matrice de rotation capteur -> repère local, convention Z-Y-X
 * (R = Rz(yaw) Ry(pitch) Rx(roll)), cohérente avec attitude_step().
 * Au repos à plat, l'accéléromètre mesure +g sur Z : on retire g sur Z local.
 */
static void motion_step(struct imu_telemetry *s, const struct mpu6050_data *d,
			float dt, bool still)
{
	const float bx = d->ax - acc_align[0];
	const float by = d->ay - acc_align[1];
	const float bz = d->az - acc_align[2];

	const float sf = sinf(s->roll * DEG2RAD), cf = cosf(s->roll * DEG2RAD);
	const float st = sinf(s->pitch * DEG2RAD), ct = cosf(s->pitch * DEG2RAD);
	const float sp = sinf(s->yaw * DEG2RAD), cp = cosf(s->yaw * DEG2RAD);

	float ax = cp * ct * bx + (cp * st * sf - sp * cf) * by + (cp * st * cf + sp * sf) * bz;
	float ay = sp * ct * bx + (sp * st * sf + cp * cf) * by + (sp * st * cf - cp * sf) * bz;
	float az = -st * bx + ct * sf * by + ct * cf * bz - g_ref;

	ax = deadband(ax);
	ay = deadband(ay);
	az = deadband(az);

	if (still) {
		/* ZUPT : carte immobile, donc vitesse nulle par définition */
		s->vx = 0.0f;
		s->vy = 0.0f;
		s->vz = 0.0f;
	} else {
		s->vx = clampf(s->vx + ax * dt, MAX_SPEED);
		s->vy = clampf(s->vy + ay * dt, MAX_SPEED);
		s->vz = clampf(s->vz + az * dt, MAX_SPEED);
	}

	s->x = clampf(s->x + s->vx * dt, MAX_POSITION);
	s->y = clampf(s->y + s->vy * dt, MAX_POSITION);
	s->z = clampf(s->z + s->vz * dt, MAX_POSITION);
}

/* ---------- Publication : télémétrie IMU + état du tracker ---------- */

static void publish(struct imu_telemetry *st, struct state_detector *det,
		    bool imu_valid, const struct mpu6050_data *d)
{
	struct state_inputs in = {
		.imu_valid = imu_valid,
		.env_valid = !(telemetry_get_flags() & FLAG_SI7021_INVALID),
		.a_norm = g_ref,       /* pas de nouvelle mesure : pas de mouvement */
		.g_ref = g_ref,
		.gyro_norm = 0.0f,
		.roll = st->roll,
		.pitch = st->pitch,
	};

	if (d != NULL) {
		in.a_norm = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);
		in.gyro_norm = sqrtf(d->gx * d->gx + d->gy * d->gy + d->gz * d->gz);
	}

	struct state_output out = state_detector_update(det, &in, k_uptime_get());

	if (out.impact) {
		st->flags |= FLAG_IMPACT;
	} else {
		st->flags &= ~FLAG_IMPACT;
	}

	telemetry_set_imu(st);
	telemetry_set_state(out.state);
}

/* ---------- Boucle principale du thread ---------- */

static void imu_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct imu_telemetry st = { 0 };
	struct calib_state calib;
	struct mpu6050_data d;
	struct state_detector det;

	bool sensor_ok = true;
	bool attitude_lost = false;   /* rotations manquées pendant une panne */
	int consecutive_errors = 0;
	int64_t last_reinit_ms = 0;
	float seconds_since_calib = 0.0f;
	int still_count = 0;
	const int64_t start_ms = k_uptime_get();

	/* Au départ : calibration en attente, intégration inertielle seule */
	st.flags = FLAG_CALIB_PENDING | FLAG_INERTIAL_ONLY | FLAG_YAW_UNRELIABLE;
	mpu6050_set_gyro_bias(0.0f, 0.0f, 0.0f);
	calib_reset(&calib);
	state_detector_init(&det);

	int64_t last_ticks = k_uptime_ticks();

	k_timer_start(&imu_timer, K_MSEC(IMU_PERIOD_MS), K_MSEC(IMU_PERIOD_MS));

	while (1) {
		/* Dort jusqu'à la prochaine échéance du timer (pas de polling) */
		k_timer_status_sync(&imu_timer);

		/* dt mesuré, plutôt que supposé égal à 20 ms */
		int64_t now_ticks = k_uptime_ticks();
		float dt = (float)k_ticks_to_us_near64(now_ticks - last_ticks) * 1e-6f;

		last_ticks = now_ticks;

		/* --- Capteur en panne : tentative de réinitialisation périodique --- */
		if (!sensor_ok) {
			if (k_uptime_get() - last_reinit_ms >= REINIT_PERIOD_MS) {
				last_reinit_ms = k_uptime_get();
				if (mpu6050_init(imu_i2c) == 0) {
					/* Le biais (propriété du capteur) reste valable */
					sensor_ok = true;
					consecutive_errors = 0;
					st.reinits++;
				}
			}
			publish(&st, &det, false, NULL);
			continue;
		}

		/* --- Lecture --- */
		if (mpu6050_read(imu_i2c, &d) != 0) {
			st.read_errors++;
			consecutive_errors++;
			if (consecutive_errors >= ERRORS_BEFORE_INVALID) {
				sensor_ok = false;
				attitude_lost = true;
				st.flags |= FLAG_IMU_INVALID | FLAG_YAW_UNRELIABLE;
				last_reinit_ms = k_uptime_get();
			}
			/* Les dernières valeurs valides restent publiées */
			publish(&st, &det, sensor_ok, NULL);
			continue;
		}

		consecutive_errors = 0;
		st.flags &= ~FLAG_IMU_INVALID;
		st.samples++;

		st.ax = d.ax;
		st.ay = d.ay;
		st.az = d.az;
		st.gx = d.gx;
		st.gy = d.gy;
		st.gz = d.gz;

		/* --- Calibration du gyroscope (non bloquante), après préchauffage --- */
		if (!st.calibrated) {
			if (k_uptime_get() - start_ms < WARMUP_MS) {
				publish(&st, &det, true, &d);
				continue;
			}
			if (calib_add(&calib, &d)) {
				if (!calib.moved) {
					mpu6050_set_gyro_bias(calib.sum[0] / calib.count,
							      calib.sum[1] / calib.count,
							      calib.sum[2] / calib.count);
					/* Alignement initial de l'accéléromètre */
					const float mx = calib.asum[0] / calib.count;
					const float my = calib.asum[1] / calib.count;
					const float mz = calib.asum[2] / calib.count;

					g_ref = sqrtf(mx * mx + my * my + mz * mz);
					acc_align[0] = mx;
					acc_align[1] = my;
					acc_align[2] = mz - g_ref;

					st.calibrated = true;
					st.flags &= ~(FLAG_CALIB_PENDING | FLAG_YAW_UNRELIABLE);

					/* SR4 : orientation initiale nulle */
					st.roll = 0.0f;
					st.pitch = 0.0f;
					st.yaw = 0.0f;
					st.x = st.y = st.z = 0.0f;
					st.vx = st.vy = st.vz = 0.0f;
					attitude_lost = false;
					seconds_since_calib = 0.0f;
				} else {
					/* La carte a bougé : on recommence */
					calib_reset(&calib);
				}
			}
			publish(&st, &det, true, &d);
			continue;
		}

		/* --- Suivi du biais quand la carte est immobile --- */
		float a_norm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
		bool still = fabsf(d.gx) < STILL_GYRO_DPS &&
			     fabsf(d.gy) < STILL_GYRO_DPS &&
			     fabsf(d.gz) < STILL_GYRO_DPS &&
			     fabsf(a_norm - g_ref) < STILL_ACC_TOLERANCE;

		still_count = still ? still_count + 1 : 0;
		if (still_count >= STILL_SAMPLES) {
			float bx, by, bz;
			const float alpha = dt / BIAS_TRACK_TAU_S;

			/* d.gx = mesure - biais : le résidu est l'erreur du biais */
			mpu6050_get_gyro_bias(&bx, &by, &bz);
			mpu6050_set_gyro_bias(bx + alpha * d.gx,
					      by + alpha * d.gy,
					      bz + alpha * d.gz);
		}

		/* --- Attitude, puis vitesse et position --- */
		attitude_step(&st, &d, dt);
		motion_step(&st, &d, dt, still_count >= STILL_SAMPLES);
		seconds_since_calib += dt;

		bool yaw_drifted = seconds_since_calib * GYRO_RESIDUAL_BIAS_DPS >
				   YAW_DRIFT_BUDGET_DEG;

		if (attitude_lost || yaw_drifted) {
			st.flags |= FLAG_YAW_UNRELIABLE;
		} else {
			st.flags &= ~FLAG_YAW_UNRELIABLE;
		}

		publish(&st, &det, true, &d);
	}
}

int imu_start(const struct device *i2c)
{
	if (i2c == NULL) {
		return -EINVAL;
	}
	imu_i2c = i2c;

	k_thread_create(&imu_thread_data, imu_stack,
			K_THREAD_STACK_SIZEOF(imu_stack),
			imu_thread, NULL, NULL, NULL,
			IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "imu");

	return 0;
}
