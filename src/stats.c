/*
 * Statistiques glissantes sur 30 s (NM4).
 *
 * Principe : la fenêtre est découpée en « cases » d'une seconde (tampon
 * circulaire de 31 cases : 30 secondes complètes + la case en cours).
 * Chaque case garde, pour chaque grandeur, la somme, le nombre, le min et le
 * max de ses mesures (10 mesures, une toutes les 100 ms). Toutes les secondes, on passe à la case suivante et on la vide :
 * la mesure la plus ancienne sort ainsi de la fenêtre.
 *
 * Pourquoi des cases plutôt que garder les 300 dernières mesures ?
 *   - mémoire : 31 cases x 5 grandeurs x 16 octets ≈ 2,5 Ko, contre 6 Ko ;
 *   - calcul : le rapport combine 31 cases, au lieu de parcourir 300 mesures.
 * Le prix : la fenêtre glisse par pas d'une seconde (elle couvre entre 30 et
 * 31 s), ce qui est largement suffisant pour un rapport toutes les 5 s.
 *
 * |a| et |ω| sont échantillonnés à 10 Hz (et non 50 Hz comme l'IMU) : un choc
 * très bref peut donc être sous-estimé dans le max ; la détection d'impact,
 * elle, se fait à 50 Hz dans le thread IMU (flag 0x0010).
 */

#include "stats.h"
#include "output.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>

#define SAMPLES_PER_BUCKET  (1000 / OUTPUT_TICK_MS)   /* 10 mesures par case */
#define NB_BUCKETS          (STATS_WINDOW_S + 1)      /* 30 s complètes + case en cours */

struct accum {
	float sum;
	float min;
	float max;
	uint32_t n;
};

struct bucket {
	struct accum q[STAT_COUNT];
};

static struct bucket buckets[NB_BUCKETS];
static int current;          /* case en cours de remplissage */
static int samples_in_bucket;
static int filled = 1;       /* nombre de cases utilisées (1 à 31) */
static bool initialized;

static void accum_reset(struct accum *a)
{
	a->sum = 0.0f;
	a->min = FLT_MAX;
	a->max = -FLT_MAX;
	a->n = 0;
}

static void bucket_reset(struct bucket *b)
{
	for (int i = 0; i < STAT_COUNT; i++) {
		accum_reset(&b->q[i]);
	}
}

static void accum_add(struct accum *a, float v)
{
	a->sum += v;
	a->min = fminf(a->min, v);
	a->max = fmaxf(a->max, v);
	a->n++;
}

void stats_add(const struct telemetry *t)
{
	if (!initialized) {
		bucket_reset(&buckets[0]);
		initialized = true;
	}

	struct bucket *b = &buckets[current];

	/* Seules les mesures valides entrent dans les statistiques */
	if (!(t->flags & FLAG_LDR_INVALID)) {
		accum_add(&b->q[STAT_LIGHT], t->light.percent);
	}
	if (!(t->flags & FLAG_SI7021_INVALID)) {
		accum_add(&b->q[STAT_TEMP], t->env.temp_c);
		accum_add(&b->q[STAT_RH], t->env.rh_percent);
	}
	if (!(t->flags & FLAG_IMU_INVALID)) {
		const struct imu_telemetry *m = &t->imu;

		accum_add(&b->q[STAT_ACC],
			  sqrtf(m->ax * m->ax + m->ay * m->ay + m->az * m->az));
		accum_add(&b->q[STAT_GYR],
			  sqrtf(m->gx * m->gx + m->gy * m->gy + m->gz * m->gz));
	}

	/* Case pleine (1 s) : on passe à la suivante et on la vide */
	if (++samples_in_bucket >= SAMPLES_PER_BUCKET) {
		samples_in_bucket = 0;
		current = (current + 1) % NB_BUCKETS;
		bucket_reset(&buckets[current]);
		if (filled < NB_BUCKETS) {
			filled++;
		}
	}
}

int stats_get(struct stats_result out[STAT_COUNT])
{
	for (int q = 0; q < STAT_COUNT; q++) {
		struct accum total;

		accum_reset(&total);

		/* Les cases non encore utilisées sont ignorées (filled) */
		for (int k = 0; k < filled; k++) {
			const struct accum *a =
				&buckets[(current - k + NB_BUCKETS) % NB_BUCKETS].q[q];

			if (a->n == 0) {
				continue;
			}
			total.sum += a->sum;
			total.min = fminf(total.min, a->min);
			total.max = fmaxf(total.max, a->max);
			total.n += a->n;
		}

		out[q].n = total.n;
		if (total.n > 0) {
			out[q].mean = total.sum / (float)total.n;
			out[q].min = total.min;
			out[q].max = total.max;
		} else {
			out[q].mean = out[q].min = out[q].max = 0.0f;
		}
	}

	/* Secondes complètes couvertes (la case en cours n'est pas comptée) */
	return filled - 1;
}
