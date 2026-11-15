
#include <stdio.h>
#include <string.h>

#include "simu_frame.h"

int main(void)
{
    struct telemetry t = {0};
    char out[SIMU_FRAME_BUFFER_SIZE];

    /* GPS */
    t.gps.fix = 1;
    t.gps.sats = 8;
    t.gps.lat = 40.389462;
    t.gps.lon = -3.627416;
    t.gps.alt = 628.4f;

    /* IMU: position */
    t.imu.x = 12.34f;
    t.imu.y = -3.20f;
    t.imu.z = 15.00f;

    /* Velocity */
    t.imu.vx = 0.50f;
    t.imu.vy = -0.10f;
    t.imu.vz = 1.20f;

    /* Orientation */
    t.imu.roll = 2.40f;
    t.imu.pitch = -1.10f;
    t.imu.yaw = 45.00f;

    /* Acceleration */
    t.imu.ax = -0.12f;
    t.imu.ay = 0.04f;
    t.imu.az = 9.81f;

    /* Gyroscope */
    t.imu.gx = 0.5f;
    t.imu.gy = -1.2f;
    t.imu.gz = 0.1f;

    /* Environment */
    t.light.percent = 73.5f;
    t.env.temp_c = 24.6f;
    t.env.rh_percent = 45.2f;

    t.state = STATE_FLYING;
    t.flags = 0x0000;

    const char expected[] =
        "$SIMU1,00152,0000123456,1,08,"
        "+40.389462,-003.627416,+00628.4,"
        "+0012.34,-0003.20,+0015.00,"
        "+000.50,-000.10,+001.20,"
        "+002.40,-001.10,+045.00,"
        "-000.12,+000.04,+009.81,"
        "+0000.5,-0001.2,+0000.1,"
        "073.5,+024.6,045.2,"
        "FLYING__,0000*3F\r\n";

    printf("\n===== SIMU FRAME SELF TEST =====\n");

    int ret = simu_frame_build(&t, 152, 123456,
                               out, sizeof(out));

    if (ret < 0) {
        printf("ERROR: simu_frame_build returned %d\n", ret);
        return 1;
    }

    printf("Expected length : %d\n", SIMU_FRAME_LENGTH);
    printf("Actual length   : %d\n", ret);
    printf("Expected CS     : 3F\n");
    printf("Actual CS       : %.2s\n", out + ret - 4);

    if (ret != SIMU_FRAME_LENGTH) {
        printf("FAIL: incorrect frame length\n");
        return 1;
    }

    if (sizeof(expected) - 1 != SIMU_FRAME_LENGTH) {
        printf("FAIL: reference string length is %zu\n",
               sizeof(expected) - 1);
        return 1;
    }

    for (int i = 0; i < SIMU_FRAME_LENGTH; i++) {
        if (out[i] != expected[i]) {
            printf("FAIL: first difference at position %d\n", i);
            printf("Expected: 0x%02X\n",
                   (unsigned char)expected[i]);
            printf("Actual  : 0x%02X\n",
                   (unsigned char)out[i]);
            return 1;
        }
    }

    printf("Frame comparison: OK\n");
    printf("SIMU FRAME TEST: PASS\n");

    return 0;
}
