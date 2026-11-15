
/*
 * SIMU frame formatter
 * Drone Tracker - Role B
 *
 * Fixed length: 221 characters including CRLF.
 */

#include "simu_frame.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Writes a fixed-width decimal number.
 *
 * Examples:
 *   fmt_fixed(p, 9.81, 1, 3, 2)  -> +009.81
 *   fmt_fixed(p, 73.5, 0, 3, 1)  -> 073.5
 *
 * NaN -> 0
 * Values are clamped to the field range.
 * Negative zero is never generated.
 */
static int fmt_fixed(char *p, double v,
                     int with_sign,
                     int int_digits,
                     int dec_digits)
{
    long scale = 1;
    long max_int = 1;

    for (int i = 0; i < dec_digits; i++) {
        scale *= 10;
    }

    for (int i = 0; i < int_digits; i++) {
        max_int *= 10;
    }

    long max_val = max_int * scale - 1;

    if (isnan(v)) {
        v = 0.0;
    }

    int negative = signbit(v) && (v != 0.0);

    if (!with_sign && negative) {
        v = 0.0;
        negative = 0;
    }

    double abs_v = fabs(v);

    /* Clamp before conversion to avoid integer overflow. */
    if (!isfinite(abs_v) || abs_v * (double)scale > (double)max_val) {
        abs_v = (double)max_val / (double)scale;
    }

    long x = lround(abs_v * (double)scale);

    if (x > max_val) {
        x = max_val;
    }

    if (x == 0) {
        negative = 0;
    }

    int n = 0;

    if (with_sign) {
        p[n++] = negative ? '-' : '+';
    }

    n += sprintf(p + n, "%0*ld.%0*ld",
                 int_digits, x / scale,
                 dec_digits, x % scale);

    return n;
}

int simu_frame_build(const struct telemetry *t,
                     uint16_t seq,
                     uint32_t tms_ms,
                     char *out,
                     size_t out_len)
{
    if (t == NULL || out == NULL) {
        return -EINVAL;
    }

    if (out_len < SIMU_FRAME_BUFFER_SIZE) {
        return -ENOSPC;
    }

    char *p = out;

    /* Fixed header */
    p += sprintf(p, "$SIMU1");

    /* Sequence and elapsed time */
    p += sprintf(p, ",%05u,%010u",
                 (unsigned int)seq,
                 (unsigned int)tms_ms);

    /* GPS fix and satellites */
    unsigned int fix = t->gps.fix ? 1 : 0;
    unsigned int sats = t->gps.sats > 99 ? 99 : t->gps.sats;

    p += sprintf(p, ",%1u,%02u", fix, sats);

    /* GPS coordinates */
    *p++ = ',';
    p += fmt_fixed(p, t->gps.lat, 1, 2, 6);

    *p++ = ',';
    p += fmt_fixed(p, t->gps.lon, 1, 3, 6);

    *p++ = ',';
    p += fmt_fixed(p, t->gps.alt, 1, 5, 1);

    /* Position X, Y, Z */
    *p++ = ',';
    p += fmt_fixed(p, t->imu.x, 1, 4, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.y, 1, 4, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.z, 1, 4, 2);

    /* Velocity VX, VY, VZ */
    *p++ = ',';
    p += fmt_fixed(p, t->imu.vx, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.vy, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.vz, 1, 3, 2);

    /* Orientation ROLL, PITCH, YAW */
    *p++ = ',';
    p += fmt_fixed(p, t->imu.roll, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.pitch, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.yaw, 1, 3, 2);

    /* Acceleration AX, AY, AZ */
    *p++ = ',';
    p += fmt_fixed(p, t->imu.ax, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.ay, 1, 3, 2);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.az, 1, 3, 2);

    /* Gyroscope GX, GY, GZ */
    *p++ = ',';
    p += fmt_fixed(p, t->imu.gx, 1, 4, 1);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.gy, 1, 4, 1);

    *p++ = ',';
    p += fmt_fixed(p, t->imu.gz, 1, 4, 1);

    
/* Light */
*p++ = ',';
p += fmt_fixed(p, t->light.percent, 0, 3, 1);

/* Temperature */
*p++ = ',';
p += fmt_fixed(p, t->env.temp_c, 1, 3, 1);

/* Humidity */
*p++ = ',';
p += fmt_fixed(p, t->env.rh_percent, 0, 3, 1);

    /* State: exactly 8 characters */
    switch (t->state) {
    case STATE_IDLE:
        p += sprintf(p, ",IDLE____");
        break;

    case STATE_FLYING:
        p += sprintf(p, ",FLYING__");
        break;

    case STATE_PROBLEM:
        p += sprintf(p, ",PROBLEM_");
        break;

    default:
        p += sprintf(p, ",IDLE____");
        break;
    }

    /* Combined flags: uppercase hexadecimal */
    p += sprintf(p, ",%04X",
                 (unsigned int)(t->flags & FLAGS_DEFINED_MASK));

    /*
     * XOR checksum:
     * Characters after '$' and before '*'.
     */
    unsigned char checksum = 0;

    for (char *q = out + 1; q < p; q++) {
        checksum ^= (unsigned char)*q;
    }

    p += sprintf(p, "*%02X\r\n", checksum);

    /* Null terminator is not part of the transmitted frame. */
    *p = '\0';

    if ((size_t)(p - out) != SIMU_FRAME_LENGTH) {
        return -EINVAL;
    }

    return SIMU_FRAME_LENGTH;
}
