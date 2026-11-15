/*
 * Thread de sortie : seul thread qui écrit sur la console (ST-LINK VCP).
 *   TEST   : rapport lisible toutes les 2 s, LED bleue (TM1-TM6)
 *   NORMAL : rapport lisible toutes les 5 s, LED = état du tracker (NM3, NM6)
 *   SIMU   : une trame SIMU toutes les 100 ms, rien d'autre (SM1-SM7)
 */

#ifndef OUTPUT_H
#define OUTPUT_H

/* Base de temps du thread : 100 ms = période des trames SIMU (10 Hz) */
#define OUTPUT_TICK_MS        100
#define OUTPUT_TEST_PERIOD_MS 2000
#define OUTPUT_NORMAL_PERIOD_MS 5000

int output_start(void);

#endif /* OUTPUT_H */
