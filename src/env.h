/*
 * Thread environnement : température et humidité (Si7021) toutes les 2 s.
 */

#ifndef ENV_H
#define ENV_H

#include <zephyr/device.h>

/* NM2 suggère 2 à 5 s ; 2 s suffit aussi pour le TEST MODE (TM2) */
#define ENV_PERIOD_MS 2000

/* Démarre le thread. Le Si7021 est initialisé par le thread lui-même. */
int env_start(const struct device *i2c);

#endif /* ENV_H */
