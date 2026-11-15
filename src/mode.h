/*
 * Mode de fonctionnement courant : TEST, NORMAL ou SIMU.
 *
 * Interface entre le bouton B1 (rôle B) et le thread de sortie (rôle A) :
 *   - le gestionnaire du bouton appelle mode_next() à chaque appui validé ;
 *   - le thread de sortie lit mode_get() toutes les 100 ms.
 * Variable atomique : aucun verrou nécessaire, utilisable depuis une ISR.
 */

#ifndef MODE_H
#define MODE_H

enum op_mode {
	MODE_TEST,
	MODE_NORMAL,
	MODE_SIMU,
};

enum op_mode mode_get(void);
void mode_set(enum op_mode mode);

/* Passe au mode suivant : TEST -> NORMAL -> SIMU -> TEST. Retourne le nouveau mode. */
enum op_mode mode_next(void);

const char *mode_name(enum op_mode mode);

#endif /* MODE_H */
