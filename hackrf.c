/*
 *  Copyright (c) 2026 xor-droid
 *
 *   HackRF One input support, modeled directly on this project's own
 *   rtl.c (device lifecycle, channelizer wiring) but using libhackrf's
 *   real API instead of librtlsdr's.
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
#include <math.h>
#include <libhackrf/hackrf.h>
#include "acarsdec.h"
#include "lib.h"

/* HackRF One's real, documented sample-rate range is 2-20 Msps
 * (libhackrf's own hackrf_set_sample_rate() doc comment). Unlike
 * rtl.c's RTLMULTMAX, there is no "hole" in the middle of the range to
 * work around here. */
#define HACKRFMULTMIN (2000000U/INTRATE)
#define HACKRFMULTMAX (20000000U/INTRATE)

/* HackRF has no on-device AGC (unlike RTL-SDR/Airspy) - LNA and VGA
 * gain are always set explicitly. Defaults chosen to match this
 * project's existing dumphfdl/dumpvdl2 HackRF deployment conventions
 * for VHF aviation reception: enough gain to hear weak signals
 * without the front end (LNA, 0-40dB in 8dB steps) clipping on strong
 * nearby transmitters. */
#define HACKRF_DEFAULT_LNA_GAIN 32U
#define HACKRF_DEFAULT_VGA_GAIN 40U

#define ERRPFX	"ERROR: HACKRF: "
#define WARNPFX	"WARNING: HACKRF: "

static hackrf_device *dev = NULL;
static volatile int hackrf_stopping = 0;

/* Mirrors rtl.c's verbose_device_search() shape: accepts a serial
 * number (exact or suffix match, matching libhackrf's own
 * hackrf_open_by_serial() convention) or is left NULL to open the
 * first available device. Unlike RTL-SDR, HackRF has no small integer
 * device-index concept in its public API. */
static int find_and_open(const char *serial)
{
	int r;

	if (serial && *serial) {
		vprerr("Opening HackRF with serial matching '%s'\n", serial);
		r = hackrf_open_by_serial(serial, &dev);
	} else {
		vprerr("Opening first available HackRF\n");
		r = hackrf_open(&dev);
	}

	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, ERRPFX "Failed to open device: %s\n",
			hackrf_error_name((enum hackrf_error)r));
		return 1;
	}

	return 0;
}

int initHackrf(char *optarg)
{
	int r;
	unsigned int Fc, m;

	if (!R.rateMult) {
		m = min_multiplier(R.minFc, R.maxFc);
		R.rateMult = (m > HACKRFMULTMIN) ? m : HACKRFMULTMIN;
	}

	if (R.rateMult > HACKRFMULTMAX) {
		fprintf(stderr, ERRPFX "rateMult can't be larger than %u\n", HACKRFMULTMAX);
		return 1;
	}
	if (R.rateMult < HACKRFMULTMIN) {
		fprintf(stderr, ERRPFX "rateMult can't be smaller than %u (HackRF's minimum sample rate is 2 Msps)\n",
			HACKRFMULTMIN);
		return 1;
	}

	Fc = find_centerfreq(R.minFc, R.maxFc, R.rateMult);
	if (!Fc)
		return 1;

	r = hackrf_init();
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, ERRPFX "hackrf_init failed: %s\n", hackrf_error_name((enum hackrf_error)r));
		return 1;
	}

	if (find_and_open(optarg))
		return 1;

	r = channels_init_sdr(Fc, R.rateMult, 127.5F);
	if (r)
		return r;

	vprerr("Setting center freq: %.4f MHz\n", Fc / 1e6);
	r = hackrf_set_freq(dev, (uint64_t)Fc);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, ERRPFX "Failed to set center frequency: %s\n",
			hackrf_error_name((enum hackrf_error)r));
		return 1;
	}

	uint32_t sample_rate = INTRATE * R.rateMult;
	fprintf(stderr, "Setting sample rate: %.4f MS/s\n", sample_rate / 1e6);
	r = hackrf_set_sample_rate(dev, (double)sample_rate);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, ERRPFX "Failed to set sample rate: %s\n",
			hackrf_error_name((enum hackrf_error)r));
		return 1;
	}

	/* Let libhackrf pick the nearest valid baseband filter for our
	 * actual reception bandwidth, same "computed, not fixed" approach
	 * rtl.c takes with rtlsdr_set_tuner_bandwidth(). */
	uint32_t bw = (R.maxFc - R.minFc) + 2 * INTRATE;
	uint32_t filter_bw = hackrf_compute_baseband_filter_bw(bw);
	fprintf(stderr, "Setting baseband filter bandwidth to: %.2f kHz\n", filter_bw / 1e3);
	r = hackrf_set_baseband_filter_bandwidth(dev, filter_bw);
	if (r != HACKRF_SUCCESS)
		fprintf(stderr, WARNPFX "Failed to set baseband filter bandwidth: %s\n",
			hackrf_error_name((enum hackrf_error)r));

	/* R.gain is acarsdec's single generic "-g" gain knob, shared
	 * across every input driver's own gain model (see rtl.c's
	 * nearest_gain()/soapy.c's own interpretation). HackRF has two
	 * independently-settable stages (LNA, VGA) and no AGC at all;
	 * -g <0 (acarsdec's usual "request AGC" sentinel, see rtl.c) has
	 * no HackRF equivalent, so it falls back to this driver's fixed
	 * defaults rather than silently claiming an AGC mode HackRF
	 * cannot provide (CLAUDE.md-equivalent "never silently claim
	 * success for an unsupported operation" -- this project has no
	 * such file, but the principle is sound regardless). */
	unsigned int lna_gain = HACKRF_DEFAULT_LNA_GAIN;
	unsigned int vga_gain = HACKRF_DEFAULT_VGA_GAIN;
	if (R.gain > 0.0F) {
		/* Split a single requested gain value across both stages,
		 * proportionally to each stage's own real max (40dB LNA,
		 * 62dB VGA), rounded down to the nearest step libhackrf
		 * actually accepts (8dB LNA steps, 2dB VGA steps). */
		float total = R.gain;
		if (total > 102.0F)
			total = 102.0F;
		lna_gain = ((unsigned int)(total * 40.0F / 102.0F) / 8U) * 8U;
		vga_gain = ((unsigned int)(total * 62.0F / 102.0F) / 2U) * 2U;
	} else {
		vprerr("No gain requested (HackRF has no AGC) -- using defaults LNA=%u VGA=%u\n",
			lna_gain, vga_gain);
	}

	vprerr("Setting LNA gain: %u dB\n", lna_gain);
	r = hackrf_set_lna_gain(dev, lna_gain);
	if (r != HACKRF_SUCCESS)
		fprintf(stderr, WARNPFX "Failed to set LNA gain: %s\n", hackrf_error_name((enum hackrf_error)r));

	vprerr("Setting VGA gain: %u dB\n", vga_gain);
	r = hackrf_set_vga_gain(dev, vga_gain);
	if (r != HACKRF_SUCCESS)
		fprintf(stderr, WARNPFX "Failed to set VGA gain: %s\n", hackrf_error_name((enum hackrf_error)r));

	vprerr("Setting antenna port power (bias tee) to %d\n", R.bias);
	r = hackrf_set_antenna_enable(dev, (uint8_t)(R.bias ? 1 : 0));
	if (r != HACKRF_SUCCESS)
		fprintf(stderr, WARNPFX "Failed to set antenna port power: %s\n",
			hackrf_error_name((enum hackrf_error)r));

	return 0;
}

/* HackRF's native RX format is signed 8-bit interleaved IQ, already
 * zero-centered (unlike RTL-SDR's unsigned 8-bit, which needs the
 * ~127.37 DC-offset subtraction rtl.c's in_callback() documents at
 * length) -- a plain (int8_t) cast is the whole conversion. */
static int hackrf_rx_callback(hackrf_transfer *transfer)
{
	const unsigned int mult = R.rateMult;
	uint8_t *buf = transfer->buffer;
	uint32_t nread = transfer->valid_length;
	float complex phasors[mult];

	if (hackrf_stopping)
		return 0;

	if (nread % 2) {
		fprintf(stderr, ERRPFX "incomplete read\n");
		return 0;
	}

	while (nread) {
		unsigned int lim = unlikely(nread / 2 < mult) ? nread / 2 : mult;

		for (unsigned int ind = 0; ind < lim; ind++) {
			float i, q;

			i = (float)((int8_t)*buf++);
			q = (float)((int8_t)*buf++);

			phasors[ind] = i + q * I;
		}
		channels_mix_phasors(phasors, lim, mult);
		nread -= lim * 2;
	}

	return 0;
}

int runHackrfSample(void)
{
	int r;

	r = hackrf_start_rx(dev, hackrf_rx_callback, NULL);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, ERRPFX "Failed to start RX: %s\n", hackrf_error_name((enum hackrf_error)r));
		return 1;
	}

	/* hackrf_start_rx() is asynchronous (unlike rtlsdr_read_async(),
	 * which blocks the calling thread until cancelled) -- block here
	 * ourselves so this function's blocking-until-stopped contract
	 * matches every other input driver's runXSample(), which
	 * acarsdec.c's main() calls synchronously before immediately
	 * calling runXClose(). */
	while (R.running && hackrf_is_streaming(dev) == HACKRF_TRUE) {
		struct timespec tick = { .tv_sec = 0, .tv_nsec = 100000000L };	// 100ms
		nanosleep(&tick, NULL);
	}

	return 0;
}

int runHackrfCancel(void)
{
	hackrf_stopping = 1;

	if (dev)
		hackrf_stop_rx(dev);	// unblocks runHackrfSample()'s poll loop

	return 0;
}

int runHackrfClose(void)
{
	int res = 0;

	if (dev) {
		res = hackrf_close(dev);
		dev = NULL;
	}
	if (res != HACKRF_SUCCESS)
		fprintf(stderr, WARNPFX "hackrf_close: %s\n", hackrf_error_name((enum hackrf_error)res));

	hackrf_exit();

	return res;
}
