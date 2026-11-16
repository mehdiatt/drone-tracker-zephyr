/*
 * Statistiques glissantes sur les 30 dernières secondes (NM4).
 *
 * Grandeurs suivies : lumière, température, humidité, |a| et |ω|.
 * Pour chacune : moyenne, minimum et maximum, calculés uniquement sur les
 * mesures valides (un capteur en erreur n'entre pas dans les statistiques).
 *
 * Utilisation : un seul thread (le thread de sortie) appelle stats_add()
 * à chaque réveil (100 ms) et stats_get() au moment du rapport NORMAL.
 * Pas d'autre thread : pas besoin de mutex.
 */

#ifndef STATS_H
#define STATS_H

#include <stdint.h>

#include "telemetry.h"

/* Durée de la fenêtre glissante */
#define STATS_WINDOW_S  30

enum stats_quantity {
	STAT_LIGHT,     /* % */
	STAT_TEMP,      /* °C */
	STAT_RH,        /* % */
	STAT_ACC,       /* |a| en m/s², gravité comprise (≈ 9,8 au repos) */
	STAT_GYR,       /* |ω| en °/s */
	STAT_COUNT
};

struct stats_result {
	float mean;
	float min;
	float max;
	uint32_t n;     /* nombre de mesures valides dans la fenêtre (0 = aucune) */
};

/* Ajoute une mesure ; à appeler toutes les OUTPUT_TICK_MS (100 ms) */
void stats_add(const struct telemetry *t);

/*
 * Calcule les statistiques de chaque grandeur sur la fenêtre.
 * Retourne le nombre de secondes complètes couvertes : 30 en régime normal,
 * moins juste après le démarrage, le temps que la fenêtre se remplisse.
 */
int stats_get(struct stats_result out[STAT_COUNT]);

#endif /* STATS_H */
