/*
 * Mode de fonctionnement courant, stocké dans une variable atomique.
 */

#include "mode.h"

#include <zephyr/sys/atomic.h>

/* Le système démarre en TEST MODE (spec) */
static atomic_t current_mode = ATOMIC_INIT(MODE_TEST);

enum op_mode mode_get(void)
{
	return (enum op_mode)atomic_get(&current_mode);
}

void mode_set(enum op_mode mode)
{
	atomic_set(&current_mode, (atomic_val_t)mode);
}

enum op_mode mode_next(void)
{
	atomic_val_t old;
	atomic_val_t next;

	/* Boucle compare-and-swap : correcte même si deux appels se chevauchent */
	do {
		old = atomic_get(&current_mode);
		next = (old == MODE_SIMU) ? MODE_TEST : old + 1;
	} while (!atomic_cas(&current_mode, old, next));

	return (enum op_mode)next;
}

const char *mode_name(enum op_mode mode)
{
	switch (mode) {
	case MODE_TEST:
		return "TEST";
	case MODE_NORMAL:
		return "NORMAL";
	case MODE_SIMU:
		return "SIMU";
	default:
		return "?";
	}
}
