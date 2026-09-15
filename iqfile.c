/*
 *  Copyright (c) 2026 xor-droid
 *
 *   Wideband raw I/Q file/stdin input, modeled on rtl.c/hackrf.c's
 *   device lifecycle and channelizer wiring, but driven by a blocking
 *   read() loop instead of a live device callback. Exists so a single
 *   physical receiver's wideband capture (e.g. a HackRF opened once
 *   via vsdr-transfer -r -) can be `tee`d to feed this decoder and a
 *   second decoder (e.g. dumpvdl2 --iq-file -) from the same capture,
 *   working around vsdrd's one-exclusive-lease-per-device model that
 *   would otherwise stop two decoders each independently opening the
 *   device natively (see hackrf.c's own header comment).
 *
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License version 2
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include "acarsdec.h"
#include "lib.h"

#define ERRPFX	"ERROR: IQFILE: "

static FILE *iqfp = NULL;
static volatile int iqfile_stopping = 0;

/* Only the two byte conventions this project's own SDR drivers
 * actually produce: rtl.c's RTL-SDR (unsigned 8-bit) and hackrf.c's
 * HackRF (signed 8-bit). Not a general-purpose recorded-file format
 * parser (no header, no other bit depths) -- CU8/CS8 raw samples
 * only, matching what a live `vsdr-transfer -r -`-style tee actually
 * emits. */
enum iqfile_format { IQFMT_U8, IQFMT_S8 };
static enum iqfile_format g_format = IQFMT_U8;

int initIqfile(char *optarg)
{
	if (!optarg)
		return 1;	// cannot happen with getopt()

	if (!R.rateMult) {
		fprintf(stderr, ERRPFX "-m <rateMult> is mandatory for --iq-file "
				"(no live device to auto-negotiate a valid rate against)\n");
		return 1;
	}
	if (!R.Fc) {
		fprintf(stderr, ERRPFX "-c <freq> is mandatory for --iq-file "
				"(must match the actual tuned frequency of the capture)\n");
		return 1;
	}

	if (strcmp(optarg, "-") == 0)
		iqfp = stdin;
	else
		iqfp = fopen(optarg, "rb");

	if (iqfp == NULL) {
		fprintf(stderr, ERRPFX "cannot open %s: %s\n", optarg, strerror(errno));
		return 1;
	}

	return channels_init_sdr(R.Fc, R.rateMult, 127.5F);
}

int initIqfileFormat(const char *fmt)
{
	if (strcasecmp(fmt, "U8") == 0 || strcasecmp(fmt, "CU8") == 0) {
		g_format = IQFMT_U8;
		return 0;
	}
	if (strcasecmp(fmt, "S8") == 0 || strcasecmp(fmt, "CS8") == 0) {
		g_format = IQFMT_S8;
		return 0;
	}
	fprintf(stderr, ERRPFX "unsupported --sample-format '%s' (supported: U8, S8)\n", fmt);
	return 1;
}

/* Same per-sample conversion each live driver already does inline in
 * its own callback (rtl.c's in_callback()'s ~127.37 DC-offset
 * subtraction for U8, hackrf.c's direct signed cast for S8) --
 * duplicated here rather than shared, matching this project's own
 * existing pattern of one self-contained conversion per input driver
 * rather than a factored-out common helper. */
static void process_chunk(const uint8_t *buf, unsigned int nread)
{
	const unsigned int mult = R.rateMult;
	float complex phasors[mult];

	if (nread % 2) {
		fprintf(stderr, ERRPFX "incomplete read\n");
		return;
	}

	while (nread) {
		unsigned int lim = unlikely(nread / 2 < mult) ? nread / 2 : mult;

		for (unsigned int ind = 0; ind < lim; ind++) {
			float i, q;

			if (g_format == IQFMT_U8) {
				i = (float)(*buf++) - 127.37f;
				q = (float)(*buf++) - 127.37f;
			} else {
				i = (float)((int8_t)*buf++);
				q = (float)((int8_t)*buf++);
			}

			phasors[ind] = i + q * I;
		}
		channels_mix_phasors(phasors, lim, mult);
		nread -= lim * 2;
	}
}

int runIqfileSample(void)
{
	unsigned int bufsize = DMBUFSZ * R.rateMult * 2;
	uint8_t *buf = malloc(bufsize);

	if (buf == NULL) {
		fprintf(stderr, ERRPFX "allocation failed\n");
		return 1;
	}

	while (R.running && !iqfile_stopping) {
		size_t n = fread(buf, 1, bufsize, iqfp);

		if (n == 0) {
			if (feof(iqfp))
				fprintf(stderr, "IQFILE: end of input, exiting\n");
			else
				fprintf(stderr, ERRPFX "read error: %s\n", strerror(errno));
			break;
		}
		process_chunk(buf, (unsigned int)n);
	}

	free(buf);
	return 0;
}

int runIqfileCancel(void)
{
	iqfile_stopping = 1;
	return 0;
}

int runIqfileClose(void)
{
	if (iqfp != NULL && iqfp != stdin)
		fclose(iqfp);
	iqfp = NULL;
	return 0;
}
