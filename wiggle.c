#include <math.h>
#include <stdint.h>
#include <stdio.h>

const double time_per_sample = 1.0 / 273000;  // OPV_RPC sample rate

const double wiggle_freq = 0.1;       // 10 second period sweeping tone back and forth
const double wiggle_extent = 20e3;    // plus-and-minus 20 kHz sweep

double wiggle_radians_per_sample = 2 * M_PI * wiggle_freq * time_per_sample;

void next_tx_sample(int16_t * const i_sample, int16_t * const q_sample)
{
    static double wiggle = 0.0;
    static double signal = 0.0;

    wiggle = fmod(wiggle + wiggle_radians_per_sample, 2 * M_PI);
    double tone_freq = sin(wiggle) * wiggle_extent;
    double tone_radians_per_sample = 2 * M_PI * tone_freq * time_per_sample;
    signal = fmod(signal + tone_radians_per_sample, 2 * M_PI);
    *i_sample = (int16_t)(cos(signal) * 32767);  // scale to 16-bit integer (12 MSbits used)
    *q_sample = (int16_t)(sin(signal) * 32767);
    // printf("%f %f\n", wiggle, signal);
}