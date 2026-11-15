/*
 * Données de télémétrie partagées entre les threads.
 *
 * Règle : chaque section a UN SEUL thread écrivain (son propriétaire).
 *   imu   -> thread IMU (rôle A)
 *   env   -> lecture Si7021 (rôle A)
 *   gps   -> thread GPS (rôle B)
 *   light -> lecture LDR (rôle B)
 *   state -> thread de statut
 * Les lecteurs (sortie TEST / NORMAL / SIMU) prennent une copie complète et
 * cohérente avec telemetry_snapshot(), puis formatent en dehors du verrou.
 *
 * Chaque section porte ses propres bits de FLAGS (table 5 de la spec) ;
 * telemetry_snapshot() les combine en un seul champ.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Bits du champ FLAGS de la trame SIMU (spec, table 5) */
#define FLAG_IMU_INVALID        0x0001  /* données IMU invalides / erreur MPU6050 */
#define FLAG_GPS_INVALID        0x0002  /* GPS invalide ou pas de fix */
#define FLAG_LDR_INVALID        0x0004  /* LDR invalide, saturé ou hors calibration */
#define FLAG_SI7021_INVALID     0x0008  /* Si7021 invalide / erreur I2C */
#define FLAG_IMPACT             0x0010  /* choc détecté */
#define FLAG_CALIB_PENDING      0x0020  /* calibration en attente ou invalide */
#define FLAG_INERTIAL_ONLY      0x0040  /* position/vitesse par intégration seule */
#define FLAG_YAW_UNRELIABLE     0x0080  /* yaw non fiable */
#define FLAGS_DEFINED_MASK      0x00FF  /* bits 8 à 15 réservés, toujours à 0 */

enum tracker_state {
	STATE_IDLE,
	STATE_FLYING,
	STATE_PROBLEM,
};

struct imu_telemetry {
	float ax, ay, az;          /* m/s², calibrés */
	float gx, gy, gz;          /* °/s, biais retiré */
	float roll, pitch, yaw;    /* degrés, intégration du gyroscope */
	float x, y, z;             /* m, position locale (intégration, dérive) */
	float vx, vy, vz;          /* m/s, vitesse locale */
	bool calibrated;           /* biais du gyroscope mesuré et valide */
	uint32_t samples;          /* nombre de mesures réussies */
	uint32_t read_errors;      /* nombre de lectures en échec */
	uint32_t reinits;          /* nombre de réinitialisations du capteur */
	uint16_t flags;
};

struct env_telemetry {
	float temp_c;
	float rh_percent;
	uint16_t flags;
};

struct gps_telemetry {
	double lat, lon;           /* degrés, négatif = S / W */
	float alt;                 /* m */
	uint8_t fix;               /* 0 = pas de fix */
	uint8_t sats;
	uint8_t hh, mm, ss;        /* heure UTC */
	bool time_valid;
	uint16_t flags;
};

struct light_telemetry {
	float percent;             /* 0..100 */
	uint16_t flags;
};

struct telemetry {
	struct imu_telemetry imu;
	struct env_telemetry env;
	struct gps_telemetry gps;
	struct light_telemetry light;
	enum tracker_state state;
	uint16_t flags;            /* combinaison des flags de toutes les sections */
};

/* Écritures : réservées au thread propriétaire de chaque section */
void telemetry_set_imu(const struct imu_telemetry *imu);
void telemetry_set_env(const struct env_telemetry *env);
void telemetry_set_gps(const struct gps_telemetry *gps);
void telemetry_set_light(const struct light_telemetry *light);
void telemetry_set_state(enum tracker_state state);

/* Lecture rapide des flags combinés seulement (sans copier toute la structure) */
uint16_t telemetry_get_flags(void);

/* Lecture : copie complète et cohérente, flags combinés */
void telemetry_snapshot(struct telemetry *out);

#endif /* TELEMETRY_H */
