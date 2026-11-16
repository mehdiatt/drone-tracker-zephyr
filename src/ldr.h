/*
 * Capteur de lumière (LDR) sur ADC1 canal 5 (A0), résolution 12 bits.
 */

#ifndef LDR_H
#define LDR_H

#include <stdbool.h>
#include <stdint.h>

/* Une mesure complète du LDR */
struct ldr_sample {
	uint16_t raw;          /* moyenne des lectures ADC, 0..4095 */
	float percent;         /* lumière calibrée, 0..100 % */
	bool valid;            /* false si erreur ADC, saturation ou hors calibration */
};

/* Initialisation du capteur LDR */
int ldr_init(void);

/* Lecture brute ADC (une seule conversion) */
int ldr_read_raw(void);

/* Conversion en pourcentage de luminosité (avec la calibration de ldr.c) */
int ldr_raw_to_percent(int raw);

/*
 * Mesure complète : moyenne de plusieurs conversions, pourcentage calibré
 * et validité. Retourne 0 si l'ADC a répondu, une erreur négative sinon.
 */
int ldr_read(struct ldr_sample *out);

#endif /* LDR_H */
