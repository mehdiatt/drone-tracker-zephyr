
#include "gps.h"
#include "telemetry.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Configuration */
#define GPS_UART_NODE DT_NODELABEL(usart1)
/* UART */
static const struct device *gps_uart = DEVICE_DT_GET(GPS_UART_NODE);

#define GPS_LINE_MAX       128
#define GPS_QUEUE_DEPTH   8

#define GPS_STACK_SIZE    2048
#define GPS_PRIORITY      5

/* Debug : afficher les lignes NMEA reçues */
#define GPS_DEBUG_RAW     1




/* File de lignes NMEA */
K_MSGQ_DEFINE(gps_msgq,
              GPS_LINE_MAX,
              GPS_QUEUE_DEPTH,
              4);

/* Buffer ISR */
static char rx_line[GPS_LINE_MAX];
static size_t rx_pos;
static bool rx_overflow;

/* Thread GPS */
K_THREAD_STACK_DEFINE(gps_stack, GPS_STACK_SIZE);
static struct k_thread gps_thread_data;

static bool gps_initialized;

/* Dernière position valide */
static struct gps_telemetry gps_data;

/* -------------------------------------------------- */
/* Conversion hexadécimale */
/* -------------------------------------------------- */

static int hex_value(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';

    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;

    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;

    return -1;
}

/* -------------------------------------------------- */
/* Vérification checksum NMEA */
/* -------------------------------------------------- */

static bool nmea_checksum_valid(const char *line)
{
    if (line == NULL || line[0] != '$')
        return false;

    const char *star = strchr(line, '*');

    if (star == NULL || star[1] == '\0' || star[2] == '\0')
        return false;

    int h = hex_value(star[1]);
    int l = hex_value(star[2]);

    if (h < 0 || l < 0)
        return false;

    uint8_t checksum = 0;

    for (const char *p = line + 1; p < star; p++)
        checksum ^= (uint8_t)*p;

    return checksum == (uint8_t)((h << 4) | l);
}

/* -------------------------------------------------- */
/* Découpage des champs sans strtok */
/* -------------------------------------------------- */

static int split_fields(char *line, char *fields[], int max_fields)
{
    int count = 0;

    if (line == NULL || fields == NULL)
        return 0;

    fields[count++] = line;

    for (char *p = line; *p != '\0'; p++)
    {
        if (*p == '*')
        {
            *p = '\0';
            break;
        }

        if (*p == ',')
        {
            *p = '\0';

            if (count >= max_fields)
                return 0;

            fields[count++] = p + 1;
        }
    }

    return count;
}

/* -------------------------------------------------- */
/* Conversion d'une valeur numérique */
/* -------------------------------------------------- */

static bool parse_double_field(const char *field, double *value)
{
    if (field == NULL || value == NULL || *field == '\0')
        return false;

    char *end;
    double v = strtod(field, &end);

    if (end == field || *end != '\0')
        return false;

    *value = v;
    return true;
}

/* -------------------------------------------------- */
/* Latitude / Longitude NMEA -> degrés décimaux */
/* -------------------------------------------------- */

static bool parse_coordinate(const char *field,
                             const char *hemisphere,
                             bool is_latitude,
                             double *result)
{
    double raw;

    if (!parse_double_field(field, &raw))
        return false;

    if (hemisphere == NULL || hemisphere[0] == '\0'
        || hemisphere[1] != '\0')
        return false;

    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);

    if (minutes < 0.0 || minutes >= 60.0)
        return false;

    double coordinate = degrees + minutes / 60.0;

    if (is_latitude)
    {
        if (hemisphere[0] != 'N' && hemisphere[0] != 'S')
            return false;

        if (degrees > 90 || coordinate > 90.0)
            return false;

        if (hemisphere[0] == 'S')
            coordinate = -coordinate;
    }
    else
    {
        if (hemisphere[0] != 'E' && hemisphere[0] != 'W')
            return false;

        if (degrees > 180 || coordinate > 180.0)
            return false;

        if (hemisphere[0] == 'W')
            coordinate = -coordinate;
    }

    *result = coordinate;
    return true;
}

/* -------------------------------------------------- */
/* Parsing de l'heure UTC */
/* -------------------------------------------------- */

static void parse_utc(const char *field,
                      struct gps_telemetry *gps)
{
    if (field == NULL || strlen(field) < 6)
        return;

    for (int i = 0; i < 6; i++)
    {
        if (field[i] < '0' || field[i] > '9')
            return;
    }

    uint8_t hh = (field[0] - '0') * 10
               + (field[1] - '0');

    uint8_t mm = (field[2] - '0') * 10
               + (field[3] - '0');

    uint8_t ss = (field[4] - '0') * 10
               + (field[5] - '0');

    if (hh > 23 || mm > 59 || ss > 60)
        return;

    gps->hh = hh;
    gps->mm = mm;
    gps->ss = ss;
    gps->time_valid = true;
}

/* -------------------------------------------------- */
/* Traitement GGA */
/* -------------------------------------------------- */

static void process_gga(char *line)
{
    char *fields[16];

    int count = split_fields(line, fields, 16);

    if (count < 10)
        return;

    if (strcmp(fields[0], "$GPGGA") != 0 &&
        strcmp(fields[0], "$GNGGA") != 0)
        return;

    struct gps_telemetry next = gps_data;

    parse_utc(fields[1], &next);

    double fix_value = 0.0;
    double sats_value = 0.0;

    if (!parse_double_field(fields[6], &fix_value) ||
        !parse_double_field(fields[7], &sats_value))
        return;

    if (fix_value < 0.0 || fix_value > 8.0 ||
        sats_value < 0.0 || sats_value > 99.0)
        return;

    next.fix = (uint8_t)fix_value;

    /* Sans fix : conserver les dernières coordonnées */
    if (next.fix == 0)
    {
        next.sats = 0;
        next.flags = FLAG_GPS_INVALID;

        gps_data = next;
        telemetry_set_gps(&next);

#if GPS_DEBUG_RAW
        printk("GPS: NO FIX, satellites=00\n");
#endif
        return;
    }

    double latitude;
    double longitude;
    double altitude;

    if (!parse_coordinate(fields[2],
                          fields[3],
                          true,
                          &latitude))
        return;

    if (!parse_coordinate(fields[4],
                          fields[5],
                          false,
                          &longitude))
        return;

    if (!parse_double_field(fields[9], &altitude))
        return;

    next.lat = latitude;
    next.lon = longitude;
    next.alt = (float)altitude;
    next.sats = (uint8_t)sats_value;
    next.flags = 0;

    gps_data = next;

    telemetry_set_gps(&next);

#if GPS_DEBUG_RAW
    printk("GPS FIX: %u satellites\n", next.sats);
    printk("Latitude : %.6f\n", next.lat);
    printk("Longitude: %.6f\n", next.lon);
    printk("Altitude : %.1f m\n", (double)next.alt);
#endif
}

/* -------------------------------------------------- */
/* Thread GPS */
/* -------------------------------------------------- */

static void gps_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[GPS_LINE_MAX];

    while (1)
    {
        /* Bloqué jusqu'à réception d'une ligne */
        if (k_msgq_get(&gps_msgq, line, K_FOREVER) != 0)
            continue;

#if GPS_DEBUG_RAW
        printk("NMEA: %s\n", line);
#endif

        if (!nmea_checksum_valid(line))
            continue;

        process_gga(line);
    }
}

/* -------------------------------------------------- */
/* Interruption UART */
/* -------------------------------------------------- */

static void gps_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) &&
           uart_irq_rx_ready(dev))
    {
        uint8_t c;

        while (uart_fifo_read(dev, &c, 1) == 1)
        {
            if (c == '\n')
            {
                if (!rx_overflow && rx_pos > 0)
                {
                    if (rx_line[rx_pos - 1] == '\r')
                        rx_pos--;

                    rx_line[rx_pos] = '\0';

                    if (rx_pos > 0)
                    {
                        char message[GPS_LINE_MAX];

                        memcpy(message, rx_line, rx_pos + 1);

                        /* Aucun blocage dans l'ISR */
                        k_msgq_put(&gps_msgq,
                                   message,
                                   K_NO_WAIT);
                    }
                }

                rx_pos = 0;
                rx_overflow = false;
            }
            else if (!rx_overflow)
            {
                if (rx_pos < GPS_LINE_MAX - 1)
                {
                    rx_line[rx_pos++] = (char)c;
                }
                else
                {
                    rx_overflow = true;
                }
            }
        }
    }
}

/* -------------------------------------------------- */
/* Initialisation */
/* -------------------------------------------------- */

int gps_init(void)
{
    if (gps_initialized)
        return 0;

    if (!device_is_ready(gps_uart))
    {
        printk("GPS ERROR: USART1 not ready\n");
        return -ENODEV;
    }

    memset(&gps_data, 0, sizeof(gps_data));

    gps_data.flags = FLAG_GPS_INVALID;

    telemetry_set_gps(&gps_data);

    k_msgq_purge(&gps_msgq);

    int ret = uart_irq_callback_user_data_set(
        gps_uart,
        gps_isr,
        NULL
    );

    if (ret != 0)
    {
        printk("GPS ERROR: UART callback (%d)\n", ret);
        return ret;
    }

    k_thread_create(
        &gps_thread_data,
        gps_stack,
        K_THREAD_STACK_SIZEOF(gps_stack),
        gps_thread,
        NULL,
        NULL,
        NULL,
        GPS_PRIORITY,
        0,
        K_NO_WAIT
    );

    k_thread_name_set(&gps_thread_data, "gps_thread");

    uart_irq_rx_enable(gps_uart);

    gps_initialized = true;

    printk("GPS initialized: USART1 / 9600 baud / IRQ\n");

    return 0;
}
