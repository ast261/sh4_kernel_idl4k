/*
 * dvb_tune - tune a DVB-S/S2 frontend through the standard DVB API and
 * hold it, printing status, signal strength, SNR and BER once a second.
 *
 * Universal LNB (9750 / 10600 MHz, high band from 11700 MHz).
 *
 * Build:
 *   sh4-idl4k-linux-gnu-gcc -O2 -Wall -static -o dvb_tune dvb_tune.c
 *
 * Usage:
 *   dvb_tune ADAPTER FREQ_MHZ h|v SR_KSPS s|s2 [SECONDS [DISEQC_PORT [ISI [PLS]]]]
 *
 * Test tool, GPLv2.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/dvb/frontend.h>

#ifndef DTV_SCRAMBLING_SEQUENCE_INDEX
#define DTV_SCRAMBLING_SEQUENCE_INDEX	70
#endif

static double now(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return tv.tv_sec + tv.tv_usec / 1e6;
}

static void msleep(int ms)
{
	struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };

	nanosleep(&ts, NULL);
}

int main(int argc, char **argv)
{
	struct dtv_property p[16];
	struct dtv_properties props = { .props = p };
	int fd, freq, hiband, vert, sr, s2, secs = 30, port = -1, isi = -1, pls = 0;
	unsigned int ifreq;
	char path[64];
	double t0, tlock = -1;
	int n = 0, i;

	if (argc < 6) {
		fprintf(stderr, "usage: %s ADAPTER FREQ_MHZ h|v SR_KSPS s|s2 "
			"[SECONDS [DISEQC_PORT [ISI [PLS]]]]\n", argv[0]);
		return 1;
	}
	freq = atoi(argv[2]);
	vert = argv[3][0] == 'v' || argv[3][0] == 'V';
	sr = atoi(argv[4]);
	s2 = !strcmp(argv[5], "s2");
	if (argc > 6)
		secs = atoi(argv[6]);
	if (argc > 7)
		port = atoi(argv[7]);
	if (argc > 8)
		isi = atoi(argv[8]);
	if (argc > 9)
		pls = atoi(argv[9]);
	hiband = freq >= 11700;
	ifreq = (freq - (hiband ? 10600 : 9750)) * 1000;

	snprintf(path, sizeof(path), "/dev/dvb/adapter%s/frontend0", argv[1]);
	fd = open(path, O_RDWR);
	if (fd < 0) {
		perror(path);
		return 1;
	}

	t0 = now();
	if (ioctl(fd, FE_SET_TONE, SEC_TONE_OFF) < 0)
		perror("FE_SET_TONE");
	if (ioctl(fd, FE_SET_VOLTAGE, vert ? SEC_VOLTAGE_13 : SEC_VOLTAGE_18) < 0)
		perror("FE_SET_VOLTAGE");
	msleep(20);
	if (port >= 0) {
		struct dvb_diseqc_master_cmd cmd = {
			{ 0xe0, 0x10, 0x38, 0xf0 | (port & 3) << 2 |
			  (vert ? 0 : 2) | hiband }, 4
		};

		if (ioctl(fd, FE_DISEQC_SEND_MASTER_CMD, &cmd) < 0)
			perror("FE_DISEQC_SEND_MASTER_CMD");
		msleep(20);
		if (ioctl(fd, FE_DISEQC_SEND_BURST, (port & 1) ? SEC_MINI_B : SEC_MINI_A) < 0)
			perror("FE_DISEQC_SEND_BURST");
		msleep(20);
	}
	if (ioctl(fd, FE_SET_TONE, hiband ? SEC_TONE_ON : SEC_TONE_OFF) < 0)
		perror("FE_SET_TONE");

	p[n].cmd = DTV_CLEAR; n++;
	p[n].cmd = DTV_DELIVERY_SYSTEM; p[n++].u.data = s2 ? SYS_DVBS2 : SYS_DVBS;
	p[n].cmd = DTV_FREQUENCY; p[n++].u.data = ifreq;
	p[n].cmd = DTV_SYMBOL_RATE; p[n++].u.data = sr * 1000;
	p[n].cmd = DTV_INNER_FEC; p[n++].u.data = FEC_AUTO;
	p[n].cmd = DTV_MODULATION; p[n++].u.data = s2 ? PSK_8 : QPSK;
	p[n].cmd = DTV_INVERSION; p[n++].u.data = INVERSION_AUTO;
	p[n].cmd = DTV_PILOT; p[n++].u.data = PILOT_AUTO;
	p[n].cmd = DTV_ROLLOFF; p[n++].u.data = ROLLOFF_AUTO;
	if (isi >= 0) {
		p[n].cmd = DTV_STREAM_ID; p[n++].u.data = isi;
		p[n].cmd = DTV_SCRAMBLING_SEQUENCE_INDEX; p[n++].u.data = pls;
	}
	p[n].cmd = DTV_TUNE; n++;
	props.num = n;
	if (ioctl(fd, FE_SET_PROPERTY, &props) < 0) {
		perror("FE_SET_PROPERTY");
		return 1;
	}
	printf("tuning %d MHz %c %d ksps %s, IF %u kHz, %s band\n", freq,
	       vert ? 'V' : 'H', sr, s2 ? "DVB-S2" : "DVB-S", ifreq,
	       hiband ? "high" : "low");

	for (i = 0; now() - t0 < secs; i++) {
		fe_status_t st = 0;
		unsigned short sig = 0, snr = 0;
		unsigned int ber = 0;

		ioctl(fd, FE_READ_STATUS, &st);
		ioctl(fd, FE_READ_SIGNAL_STRENGTH, &sig);
		ioctl(fd, FE_READ_SNR, &snr);
		ioctl(fd, FE_READ_BER, &ber);
		if ((st & FE_HAS_LOCK) && tlock < 0) {
			tlock = now() - t0;
			printf("LOCK after %.2f s\n", tlock);
		}
		if (i % 10 == 0)
			printf("%3.0f s: status 0x%02x %s signal %5u snr %5u ber %u\n",
			       now() - t0, st, st & FE_HAS_LOCK ? "LOCK" : "----",
			       sig, snr, ber);
		fflush(stdout);
		msleep(100);
	}
	close(fd);
	return tlock < 0 ? 2 : 0;
}
