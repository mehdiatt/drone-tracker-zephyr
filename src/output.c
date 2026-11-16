/*
 * Thread de sortie.
 *
 * Un k_timer de 100 ms réveille le thread (k_timer_status_sync : pas de
 * polling). À chaque réveil :
 *   - il lit le mode courant (mode_get) et met à jour la LED ;
 *   - en SIMU, il envoie une trame à chaque réveil (10 Hz) ;
 *   - en TEST, un rapport tous les 20 réveils (2 s) ; en NORMAL, tous les 50 (5 s).
 * Un changement de mode est donc pris en compte en moins de 100 ms, et le
 * premier rapport du nouveau mode part immédiatement.
 *
 * Seul ce thread écrit sur la console : en SIMU, aucun autre texte ne peut
 * s'intercaler entre deux trames (GR5, SM6), à condition qu'aucun autre code
 * n'appelle printk pendant ce mode.
 *
 * Les trames SIMU sont écrites octet par octet avec uart_poll_out() et non
 * avec printk : la console Zephyr ajoute un '\r' devant chaque '\n', ce qui
 * donnerait "\r\r\n" et une trame de 222 caractères au lieu de 221.
 */

#include "output.h"
#include "mode.h"
#include "rgb_led.h"
#include "simu_frame.h"
#include "telemetry.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define OUTPUT_STACK_SIZE 2048
#define OUTPUT_PRIORITY   4      /* après l'IMU (2), avant GPS (5) et env (6) */

#define TEST_TICKS   (OUTPUT_TEST_PERIOD_MS / OUTPUT_TICK_MS)     /* 20 */
#define NORMAL_TICKS (OUTPUT_NORMAL_PERIOD_MS / OUTPUT_TICK_MS)   /* 50 */

K_THREAD_STACK_DEFINE(output_stack, OUTPUT_STACK_SIZE);
static struct k_thread output_thread_data;
K_TIMER_DEFINE(output_timer, NULL, NULL);

/* Console = LPUART1 via le ST-LINK (choisie par la carte : zephyr,console) */
static const struct device *const console_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static const char *const state_names[] = { "IDLE", "FLYING", "PROBLEM" };

/* ---------- Affichage lisible (TEST / NORMAL) ---------- */

/*
 * Affiche v en virgule fixe, sans dépendre du support des flottants ni des
 * largeurs variables de printk : ex. put_fixed(9.81, true, 3, 2) -> "+009.81".
 */
static void put_fixed(double v, bool with_sign, int int_digits, int dec_digits)
{
	char buf[24];
	int n = 0;
	long scale = 1;
	long max_int = 1;

	for (int i = 0; i < dec_digits; i++) {
		scale *= 10;
	}
	for (int i = 0; i < int_digits; i++) {
		max_int *= 10;
	}

	if (!(v == v)) {         /* NaN */
		v = 0.0;
	}
	long x = lround(fabs(v) * (double)scale);

	if (x > max_int * scale - 1) {
		x = max_int * scale - 1;
	}
	bool neg = (v < 0.0) && (x != 0);

	if (!with_sign && neg) {   /* champ non signé : une valeur négative s'affiche 0 */
		x = 0;
		neg = false;
	}

	if (with_sign) {
		buf[n++] = neg ? '-' : '+';
	}

	long ip = x / scale;
	long fp = x % scale;

	for (int i = int_digits - 1; i >= 0; i--) {
		buf[n + i] = (char)('0' + ip % 10);
		ip /= 10;
	}
	n += int_digits;
	buf[n++] = '.';
	for (int i = dec_digits - 1; i >= 0; i--) {
		buf[n + i] = (char)('0' + fp % 10);
		fp /= 10;
	}
	n += dec_digits;
	buf[n] = '\0';

	printk("%s", buf);
}

static void put_xyz(const char *label, float x, float y, float z,
		    int int_digits, int dec_digits, const char *unit)
{
	printk("%s x=", label); put_fixed(x, true, int_digits, dec_digits);
	printk(" y=");          put_fixed(y, true, int_digits, dec_digits);
	printk(" z=");          put_fixed(z, true, int_digits, dec_digits);
	printk(" %s\n", unit);
}

static void print_report(const struct telemetry *t, enum op_mode mode)
{
	const bool gps_ok = !(t->flags & FLAG_GPS_INVALID);
	const bool env_ok = !(t->flags & FLAG_SI7021_INVALID);
	const bool ldr_ok = !(t->flags & FLAG_LDR_INVALID);

	printk("\n===== %s MODE  t=%u ms  STATE=%s  FLAGS=%04X =====\n",
	       mode_name(mode), k_uptime_get_32(), state_names[t->state], t->flags);

	if (!t->imu.calibrated) {
		printk("IMU : calibration en cours, carte immobile svp\n");
	}

	put_xyz("ACC ", t->imu.ax, t->imu.ay, t->imu.az, 3, 2, "m/s2");
	put_xyz("GYR ", t->imu.gx, t->imu.gy, t->imu.gz, 4, 1, "deg/s");

	printk("ATT  roll=");  put_fixed(t->imu.roll, true, 3, 2);
	printk(" pitch=");     put_fixed(t->imu.pitch, true, 3, 2);
	printk(" yaw=");       put_fixed(t->imu.yaw, true, 3, 2);
	printk(" deg%s\n", (t->flags & FLAG_YAW_UNRELIABLE) ? "  (yaw non fiable)" : "");

	put_xyz("POS ", t->imu.x, t->imu.y, t->imu.z, 4, 2, "m");
	put_xyz("VEL ", t->imu.vx, t->imu.vy, t->imu.vz, 3, 2, "m/s");

	printk("GPS  fix=%u sats=%02u lat=", gps_ok ? 1U : 0U, (unsigned int)t->gps.sats);
	put_fixed(t->gps.lat, true, 2, 6);
	printk(" lon=");  put_fixed(t->gps.lon, true, 3, 6);
	printk(" alt=");  put_fixed(t->gps.alt, true, 5, 1);
	printk(" m  %s", gps_ok ? "(fix valide)" : "(pas de fix)");
	if (t->gps.time_valid) {
		printk("  UTC %02u:%02u:%02u", t->gps.hh, t->gps.mm, t->gps.ss);
	}
	printk("\n");

	printk("ENV  T=");  put_fixed(t->env.temp_c, true, 3, 1);
	printk(" C  RH=");  put_fixed(t->env.rh_percent, false, 3, 1);
	printk(" %%  %s\n", env_ok ? "(ok)" : "(Si7021 invalide)");

	printk("LDR  light=");  put_fixed(t->light.percent, false, 3, 1);
	printk(" %%  raw=%u  %s\n", (unsigned int)t->light.raw,
	       ldr_ok ? "(ok)" : "(invalide : sature ou hors calibration)");
}

/* ---------- Trame SIMU ---------- */

static void send_raw(const char *s, int len)
{
	for (int i = 0; i < len; i++) {
		uart_poll_out(console_uart, (unsigned char)s[i]);
	}
}

/* ---------- LED ---------- */

static void update_led(enum op_mode mode, enum tracker_state state)
{
	switch (mode) {
	case MODE_TEST:
		rgb_led_set(RGB_BLUE);            /* TM4 : bleu quel que soit l'état */
		break;
	case MODE_SIMU:
		rgb_led_set(RGB_MAGENTA);         /* SM7 : couleur dédiée, choisie */
		break;
	case MODE_NORMAL:
	default:
		/* NM6 : vert = IDLE, jaune = FLYING, rouge = PROBLEM */
		if (state == STATE_PROBLEM) {
			rgb_led_set(RGB_RED);
		} else if (state == STATE_FLYING) {
			rgb_led_set(RGB_YELLOW);
		} else {
			rgb_led_set(RGB_GREEN);
		}
		break;
	}
}

/* ---------- Boucle du thread ---------- */

static void output_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct telemetry t;
	char frame[SIMU_FRAME_BUFFER_SIZE];
	uint16_t seq = 0;                 /* revient à 0 après 65535 (SEQ) */
	enum op_mode last_mode = (enum op_mode)-1;
	int ticks = 0;

	k_timer_start(&output_timer, K_MSEC(OUTPUT_TICK_MS), K_MSEC(OUTPUT_TICK_MS));

	while (1) {
		k_timer_status_sync(&output_timer);

		enum op_mode mode = mode_get();

		if (mode != last_mode) {
			last_mode = mode;
			ticks = 0;                /* premier rapport immédiat */
		}

		telemetry_snapshot(&t);
		update_led(mode, t.state);

		switch (mode) {
		case MODE_SIMU: {
			int len = simu_frame_build(&t, seq, k_uptime_get_32(),
						   frame, sizeof(frame));
			if (len == SIMU_FRAME_LENGTH) {
				send_raw(frame, len);
				seq++;
			}
			/* Trame invalide : on n'envoie rien plutôt qu'une trame fausse */
			break;
		}
		case MODE_TEST:
			if (ticks % TEST_TICKS == 0) {
				print_report(&t, mode);
			}
			break;
		case MODE_NORMAL:
		default:
			if (ticks % NORMAL_TICKS == 0) {
				/* TODO (rôle A) : ajouter les statistiques sur 30 s (NM4) */
				print_report(&t, mode);
			}
			break;
		}

		ticks++;
	}
}

int output_start(void)
{
	if (!device_is_ready(console_uart)) {
		return -ENODEV;
	}

	k_thread_create(&output_thread_data, output_stack,
			K_THREAD_STACK_SIZEOF(output_stack),
			output_thread, NULL, NULL, NULL,
			OUTPUT_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&output_thread_data, "output");

	return 0;
}
