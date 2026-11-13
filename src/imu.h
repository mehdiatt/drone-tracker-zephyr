/*
 * Thread IMU : acquisition du MPU6050 à 50 Hz, calibration du gyroscope,
 * estimation de l'attitude (roll, pitch, yaw) et publication dans telemetry.
 */

#ifndef IMU_H
#define IMU_H

#include <zephyr/device.h>

/* Période d'échantillonnage : 20 ms = 50 Hz (NM1 suggère 20 à 50 Hz) */
#define IMU_PERIOD_MS 20

/*
 * Démarre le thread IMU. Le MPU6050 doit déjà avoir été initialisé
 * avec mpu6050_init(). Retourne 0 si OK.
 */
int imu_start(const struct device *i2c);

#endif /* IMU_H */
