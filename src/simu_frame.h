
#ifndef SIMU_FRAME_H
#define SIMU_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry.h"

#define SIMU_FRAME_LENGTH 221
#define SIMU_FRAME_BUFFER_SIZE (SIMU_FRAME_LENGTH + 1)

/*
 * Construit une trame SIMU complète.
 *
 * Retourne :
 *   221 : succès
 *   -EINVAL : argument invalide
 *   -ENOSPC : buffer trop petit
 */
int simu_frame_build(const struct telemetry *t,
                     uint16_t seq,
                     uint32_t tms_ms,
                     char *out,
                     size_t out_len);

#endif /* SIMU_FRAME_H */
