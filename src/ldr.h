
#ifndef LDR_H
#define LDR_H

#include <stdint.h>

/* Initialisation du capteur LDR */
int ldr_init(void);

/* Lecture brute ADC */
int ldr_read_raw(void);

/* Conversion en pourcentage de luminosité */
int ldr_raw_to_percent(int raw);

#endif /* LDR_H */
