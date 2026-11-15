
#ifndef MODE_H
#define MODE_H

/*
 * Gestion centralisée des modes :
 * TEST -> NORMAL -> SIMU -> TEST
 */

enum op_mode {
    MODE_TEST = 0,
    MODE_NORMAL,
    MODE_SIMU
};

/* Initialisation du bouton B1 */
int mode_button_init(void);

/* Lecture et modification du mode courant */
enum op_mode mode_get(void);
void mode_set(enum op_mode mode);
enum op_mode mode_next(void);

/* Nom du mode pour les rapports */
const char *mode_name(enum op_mode mode);

#endif
