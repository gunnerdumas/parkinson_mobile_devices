#include "tremor_detect.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void compute_hann(float *w, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
        w[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
}

static float prefilter(TremorDetector_t *td, float val)
{
    td->prefilter_buf[td->prefilter_idx] = val;
    td->prefilter_idx = (td->prefilter_idx + 1) % PREFILTER_SIZE;
    float sum = 0.0f;
    for (uint8_t i = 0; i < PREFILTER_SIZE; i++) sum += td->prefilter_buf[i];
    return sum / PREFILTER_SIZE;
}

void TremorDetect_Init(TremorDetector_t *td)
{
    memset(td, 0, sizeof(TremorDetector_t));
    arm_status status = arm_rfft_fast_init_f32(&td->fft_inst, FFT_SIZE);
    printf("FFT INIT STATUS: %d\r\n", status);
    compute_hann(td->hann, FFT_SIZE);
    td->data.bin_width = FFT_BIN_WIDTH;
}


void TremorDetect_AddSample(TremorDetector_t *td,
                             float gx, float gy, float gz,
                             float ax, float ay, float az)
{
    ParkinsonsData_t *d = &td->data;

    /*  Raw gyro axes  */
    d->gx_raw = gx;
    d->gy_raw = gy;
    d->gz_raw = gz;

    /*  Gyro vector magnitude  */
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    d->gyro_magnitude = gyro_mag;

    /*  Raw accel axes  */
    d->ax_raw = ax;
    d->ay_raw = ay;
    d->az_raw = az;

    /*  Accel vector magnitude  */
    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    d->accel_magnitude = accel_mag;

    /*  Vertical component (gravity reference)  */
    d->accel_vertical = az;

    /*  Jerk magnitude (change in accel / dt)
      Compares current accel magnitude to previous sample.
      dt = 0.01s (10ms sample period at 104 Hz).            */
    static float prev_accel_mag = 0.0f;
    d->jerk_magnitude = fabsf(accel_mag - prev_accel_mag) / 0.01f;
    prev_accel_mag = accel_mag;

    /*  Feed gyro magnitude into FFT buffer  */
    float filtered = prefilter(td, gyro_mag);
    if (td->sample_count < FFT_SIZE)
        td->raw_buf[td->sample_count++] = filtered;
}


bool TremorDetect_Process(TremorDetector_t *td)
{
    if (td->sample_count < FFT_SIZE) return false;

    ParkinsonsData_t *d = &td->data;

    /*  DC removal */
    float mean = 0.0f;
    for (uint16_t i = 0; i < FFT_SIZE; i++) mean += td->raw_buf[i];
    mean /= FFT_SIZE;

    /*  Hann window */
    for (uint16_t i = 0; i < FFT_SIZE; i++)
        td->windowed[i] = (td->raw_buf[i] - mean) * td->hann[i];

    /*  Real FFT */
    arm_rfft_fast_f32(&td->fft_inst, td->windowed, td->fft_out, 0);
    /*  Magnitude spectrum */
    td->fft_mag[0] = 0.0f;   /* Zero DC bin */
    arm_cmplx_mag_f32(&td->fft_out[2], &td->fft_mag[1], (FFT_SIZE / 2) - 1);

    /*  Find peak bin and sum band energy in tremor range */
    float    peak_mag    = 0.0f;
    float    band_energy = 0.0f;
    uint16_t peak_bin    = TREMOR_BIN_MIN;

    for (uint16_t bin = TREMOR_BIN_MIN; bin <= TREMOR_BIN_MAX; bin++) {
        band_energy += td->fft_mag[bin];
        if (td->fft_mag[bin] > peak_mag) {
            peak_mag = td->fft_mag[bin];
            peak_bin = bin;
        }
    }

    /*  Populate ParkinsonsData_t */
    d->dominant_freq      = peak_bin * FFT_BIN_WIDTH;
    d->peak_fft_mag       = peak_mag;
    d->tremor_band_energy = band_energy;
    d->timestamp_ms       = HAL_GetTick();
    d->total_windows++;

    /* Tremor decision with hysteresis */
    bool raw_tremor = (peak_mag >= TREMOR_FFT_THRESHOLD);

    if (raw_tremor) {
        td->clear_count = 0;
        if (td->confirm_count < CONFIRM_WINDOWS) td->confirm_count++;
        if (td->confirm_count >= CONFIRM_WINDOWS) {
            d->tremor_detected = true;
            d->tremor_windows++;
        }
    } else {
        td->confirm_count = 0;
        if (td->clear_count < CLEAR_WINDOWS) td->clear_count++;
        if (td->clear_count >= CLEAR_WINDOWS)
            d->tremor_detected = false;
    }

    d->tremor_percent = (d->total_windows > 0)
                        ? (100.0f * d->tremor_windows) / d->total_windows
                        : 0.0f;

    /* Reset for next window */
    td->sample_count = 0;
    memset(td->raw_buf, 0, sizeof(td->raw_buf));


    return true;
}

void TremorDetect_GetSpectrum(TremorDetector_t *td, float *out, uint16_t len)
{
    uint16_t n = (len < FFT_SIZE / 2) ? len : FFT_SIZE / 2;
    memcpy(out, td->fft_mag, n * sizeof(float));
}
