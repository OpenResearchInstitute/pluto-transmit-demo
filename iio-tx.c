// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * libiio - AD9361 IIO streaming example
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <unistd.h>

/* OPV-RTP configuration */
#define SYMBOLS_PER_40MS	1092
#define SAMPLES_PER_SYMBOL	10
#define SAMPLES_PER_SECOND	273000

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
//static struct iio_channel *rx0_i = NULL;
//static struct iio_channel *rx0_q = NULL;

static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
//static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown()
{	
	printf("* Destroying buffers\n");
//	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	printf("* Disabling streaming channels\n");
//	if (rx0_i) { iio_channel_disable(rx0_i); }
//	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	printf("Waiting for process to finish... Got signal %d\n", sig);
	stop = true;
}

/* check return value of iio_attr_write function (for whole device) */
static void errchk_dev(int v) {
	if (v < 0) { fprintf(stderr, "Error %d writing to IIO device\n"); shutdown(); }
}

/* check return value of iio_channel_attr_write function */
static void errchk_chn(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk_chn(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk_chn(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 decimation/interpolation configuration channels, I hope */
static bool get_dec8_int8_chan(enum iodev d, struct iio_channel **chn)
{
	struct iio_device *dev;

	IIO_ENSURE(get_ad9361_stream_dev(d, &dev) && "No dec/int dev found");

	switch (d) {
	case RX: *chn = iio_device_find_channel(dev, get_ch_name("voltage", 0), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(dev, get_ch_name("voltage", 0), true); return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;
	struct iio_channel *chn2 = NULL;
	struct iio_channel *chn3 = NULL;
	long long sf1, sf2;
	size_t response_buf_len = 40;
	char response_buf[response_buf_len];

	// Configure phy and lo channels
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rfport);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	if (cfg->fs_hz >= 2083334) {		// Pluto minimum without extra interpolation
		IIO_ENSURE(0 && "high sample rates not implemented");
	} else if (cfg->fs_hz < 260417) {	// Pluto minimum with 8x interpolation
		IIO_ENSURE(0 && "Pluto can't sample that low, even with 8x interpolation");
	} else {
		// Enable the AD9361's TX FIR 8x interpolator
		// The interpolator is set by channel-specific attributes
		// of the channel named voltage0
		// of the device named cf-ad9361-dds-core-lpc.
		// Two attributes are involved: sampling_frequency and sampling_frequency_available.
		// The attribute sampling_frequency_available returns a list of two available values,
		// which should differ by a factor of 8. The higher value is the sample clock rate
		// when interpolation is not used; the lower value is the effective sample clock rate
		// when interpolation is used.
		// Set the attribute sampling_frequency to the value corresponding to the range of
		// sample rates you want to use.

		printf("* Setting 8x transmit interpolation\n");

		// obtain the transmit interpolator's config channel,
		// namely device cf-ad9361-dds-core-lpc, channel voltage0
		if (!get_dec8_int8_chan(TX, &chn3)) { return false; }
		
		// read the attribute sampling_frequency_available,
		// which should be a list of two, like '30720000 3840000 '
		IIO_ENSURE(iio_channel_attr_read(chn3,
			"sampling_frequency_available",
			response_buf, response_buf_len) > 0 && "No sampling_frequency_available attribute");
		IIO_ENSURE(2 == sscanf(response_buf, "%lld %lld ", &sf1, &sf2) && "sampling_frequency_available format unexpected");
		IIO_ENSURE(abs(8 * sf2 - sf1) < 20 && "sampling_frequency_available values not in 8x ratio");

		// now write the lower of the two values into attribute sampling_frequency
		// to set the interpolator to active.
		printf("* Writing %lld to set 8x interpolation\n", sf2);
		wr_ch_lli(chn3, "sampling_frequency", sf2);

		// Now we can set the actual sample rate within the range
		// supported by this transmit interpolation mode.
		printf("* Setting AD9361 sample rate to %lld Hz\n", cfg->fs_hz * 8);
		wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz * 8);
	}
	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(type, &chn2)) { return false; }
	wr_ch_lli(chn2, "frequency", cfg->lo_hz);
	return true;
}

/* turns off the transmit local oscillator */
static bool cfg_ad9361_txlo_powerdown(long long val)
{
	struct iio_channel *chn = NULL;
	
	if (!get_lo_chan(TX, &chn)) { return false; }
	wr_ch_lli(chn, "powerdown", val);
	return true;
}

/* adjusts single-ended crystal oscillator compensation for Pluto SDR */
static bool cfg_ad9361_xo_correction(int delta)
{
	errchk_dev(iio_device_attr_write_longlong(get_ad9361_phy(), "xo_correction", 40000000 + delta));
}

/* turns off automatic transmit calibration for Pluto SDR */
static bool cfg_ad9361_manual_tx_quad(void)
{
	errchk_dev(iio_device_attr_write(get_ad9361_phy(), "calib_mode", "manual_tx_quad"));
}

/* simple configuration and streaming */
/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 * 
 */
int main (int argc, char **argv)
{
	// Streaming devices
	struct iio_device *tx;
//	struct iio_device *rx;

	// RX and TX sample counters
	size_t nrx = 0;
	size_t ntx = 0;
	
	// Buffer pointers
	char *p_dat, *p_end;
	ptrdiff_t p_inc;

	// Stream configurations
//	struct stream_cfg rxcfg;
	struct stream_cfg txcfg;

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);

	// RX stream config
//	rxcfg.bw_hz = MHZ(2);   // 2 MHz rf bandwidth
//	rxcfg.fs_hz = MHZ(2.5);   // 2.5 MS/s rx sample rate
//	rxcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
//	rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

	// TX stream config
//	txcfg.bw_hz = MHZ(1.5); // 1.5 MHz rf bandwidth
	txcfg.bw_hz = 200000;	// 200 kHz RF bandwidth, Pluto's minimum
//	txcfg.fs_hz = MHZ(2.5);	// 2.5 MS/s tx sample rate
	txcfg.fs_hz = SAMPLES_PER_SECOND;	// baseband sample rate
//	txcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
	txcfg.lo_hz = MHZ(905.05);	// 905.05 MHz RF frequency
	txcfg.rfport = "A"; // port A (select for rf freq.)

	printf("* Acquiring IIO context\n");
	if (argc == 1) {
		IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
	}
	else if (argc == 2) {
		IIO_ENSURE((ctx = iio_create_context_from_uri(argv[1])) && "No context");
	}
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");
	unsigned int attrs_count = iio_context_get_attrs_count(ctx);
	IIO_ENSURE(attrs_count > 0 && "No context attributes");
	printf("Found IIO context:\n");
	for (unsigned int index=0; index < attrs_count; index++) {
		const char *attr_name;
		const char *attr_val;
		if (iio_context_get_attr(ctx, index, &attr_name, &attr_val) == 0) {
			printf("%s: %s\n", attr_name, attr_val);
		}
	}

	printf("* Acquiring AD9361 streaming devices\n");
	IIO_ENSURE(get_ad9361_stream_dev(TX, &tx) && "No tx dev found");
//	IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

	printf("* Configuring Pluto SDR for transmitting\n");
	cfg_ad9361_manual_tx_quad();	// disable automatic TX calibration
	cfg_ad9361_xo_correction(-465);	// -465 out of 40e6 for remote lab's Pluto S/N b83991001015001f00c7a0653f04
	cfg_ad9361_txlo_powerdown(0);	// enable transmit LO

	printf("* Configuring AD9361 for streaming\n");
//	IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&txcfg, TX, 0) && "TX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");
//	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");
//	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 0, &tx0_i) && "TX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 1, &tx0_q) && "TX chan q not found");

	printf("* Enabling IIO streaming channels\n");
//	iio_channel_enable(rx0_i);
//	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);
	
//	cfg_ad9361_txlo_powerdown(1);	// !!! try to suppress noise burst

	printf("* Creating non-cyclic IIO buffers of %d symbols (1 40ms frame)\n", SYMBOLS_PER_40MS);
//	rxbuf = iio_device_create_buffer(rx, 1024*1024, false);
//	if (!rxbuf) {
//		perror("Could not create RX buffer");
//		shutdown();
//	}
	txbuf = iio_device_create_buffer(tx, SYMBOLS_PER_40MS, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		shutdown();
		return 0;
	}
	
	// Write first TX buf with all zeroes, for cleaner startup 
	p_inc = iio_buffer_step(txbuf);
	p_end = iio_buffer_end(txbuf);
	for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			((int16_t*)p_dat)[0] = 0 << 4; // Real (I)
			((int16_t*)p_dat)[1] = 0 << 4; // Imag (Q)
		}

//	cfg_ad9361_txlo_powerdown(0);

	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	while (!stop)
	{
		ssize_t nbytes_rx, nbytes_tx;

		// Schedule TX buffer
		nbytes_tx = iio_buffer_push(txbuf);
		if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); shutdown(); }

		// Refill RX buffer
//		nbytes_rx = iio_buffer_refill(rxbuf);
//		if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); shutdown(); }

		// READ: Get pointers to RX buf and read IQ from RX buf port 0
//		p_inc = iio_buffer_step(rxbuf);
//		p_end = iio_buffer_end(rxbuf);
//		for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: swap I and Q
//			const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
//			const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)
//			((int16_t*)p_dat)[0] = q;
//			((int16_t*)p_dat)[1] = i;
//		}

		// WRITE: Get pointers to TX buf and write IQ to TX buf port 0
		p_inc = iio_buffer_step(txbuf);
		p_end = iio_buffer_end(txbuf);
		for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: fill with zeros
			// 12-bit sample needs to be MSB aligned so shift by 4
			// https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
			((int16_t*)p_dat)[0] = 0 << 4; // Real (I)
			((int16_t*)p_dat)[1] = 0 << 4; // Imag (Q)
		}

		// Sample counter increment and status output
		nrx += 0; // nbytes_rx / iio_device_get_sample_size(rx);
		ntx += nbytes_tx / iio_device_get_sample_size(tx);
		printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx/1e6, ntx/1e6);
		
		// if (ntx > 6e6) cfg_ad9361_txlo_powerdown(0);	// delayed start
	}

	cfg_ad9361_txlo_powerdown(1);
	shutdown();

	return 0;
}
